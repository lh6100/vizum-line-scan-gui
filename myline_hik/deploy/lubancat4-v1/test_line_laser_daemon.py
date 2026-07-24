#!/usr/bin/env python3
"""Protocol and fail-safe tests that never touch real GPIO hardware."""

import io
import json
import time
import types
import unittest

import line_laser_daemon as daemon


class FakeGpio:
    def __init__(self):
        self.offset_450 = 15
        self.offset_650 = 16
        self.high_450 = False
        self.high_650 = False
        self.closed = False
        self.history = []

    def set_state(self, state):
        if self.closed:
            raise OSError("fake GPIO is closed")
        self.high_450 = state == daemon.STATE_450
        self.high_650 = state == daemon.STATE_650
        self.history.append((self.high_450, self.high_650))
        if self.high_450 and self.high_650:
            raise AssertionError("mutual exclusion was violated")

    def values(self):
        return self.high_450, self.high_650

    def close(self):
        self.high_450 = False
        self.high_650 = False
        self.closed = True


class ServiceTest(unittest.TestCase):
    def setUp(self):
        self.gpio = FakeGpio()
        self.service = daemon.LaserService(
            self.gpio, lease_ms=500, board_model="LubanCat-4 V1 test")

    def tearDown(self):
        if not self.gpio.closed:
            self.service.shutdown()

    def test_mutual_exclusion_and_mapping_state(self):
        self.service.acquire("a", "test-client")
        status = self.service.set_state("a", daemon.STATE_450)
        self.assertTrue(status["ttl450_high"])
        self.assertFalse(status["ttl650_high"])
        status = self.service.set_state("a", daemon.STATE_650)
        self.assertFalse(status["ttl450_high"])
        self.assertTrue(status["ttl650_high"])
        self.assertEqual(
            self.gpio.history[-3:],
            [(True, False), (False, False), (False, True)])
        self.assertNotIn((True, True), self.gpio.history)
        self.assertEqual(
            status["pin_map"]["laser450"]["physical_pin"], 11)
        self.assertEqual(
            status["pin_map"]["laser650"]["physical_pin"], 7)
        self.assertEqual(
            status["pin_map"]["laser450"]["chip_offset"], 15)
        self.assertEqual(
            status["pin_map"]["laser650"]["chip_offset"], 16)

    def test_second_client_cannot_take_live_lease(self):
        self.service.acquire("owner", "first")
        with self.assertRaises(daemon.LaserError) as caught:
            self.service.acquire("other", "second")
        self.assertEqual(caught.exception.code, "BUSY")

    def test_disconnect_forces_low(self):
        self.service.acquire("owner", "first")
        self.service.set_state("owner", daemon.STATE_450)
        self.service.disconnect("owner")
        self.assertEqual(self.gpio.values(), (False, False))
        self.assertFalse(self.service.status()["lease_active"])

    def test_expired_lease_forces_low(self):
        self.service.acquire("owner", "first")
        self.service.set_state("owner", daemon.STATE_650)
        time.sleep(0.55)
        self.assertTrue(self.service.expire_if_needed())
        self.assertEqual(self.gpio.values(), (False, False))
        self.assertFalse(self.service.status()["lease_active"])

    def test_invalid_request_cannot_change_output(self):
        self.service.acquire("owner", "first")
        response = daemon.process_request(
            self.service,
            "owner",
            {"v": 1, "id": 4, "op": "set", "state": "both"})
        self.assertFalse(response["ok"])
        self.assertEqual(response["error"]["code"], "INVALID_STATE")
        self.assertEqual(self.gpio.values(), (False, False))


class FakeStreamConnection:
    """socket-like byte streams for exercising StreamRequestHandler."""

    def __init__(self, incoming):
        self.incoming = io.BytesIO(incoming)
        self.outgoing = io.BytesIO()

    def makefile(self, mode, _buffering=None):
        if "r" in mode:
            return self.incoming
        return self.outgoing

    def sendall(self, data):
        self.outgoing.write(data)


class ProtocolHandlerTest(unittest.TestCase):
    def setUp(self):
        self.gpio = FakeGpio()
        self.service = daemon.LaserService(
            self.gpio, lease_ms=500, board_model="LubanCat-4 V1 test")
        self.server = types.SimpleNamespace(laser_service=self.service)

    def tearDown(self):
        if not self.gpio.closed:
            self.service.shutdown()

    def _run_handler(self, requests):
        incoming = b"".join(
            json.dumps(item, separators=(",", ":")).encode("utf-8") + b"\n"
            for item in requests)
        return self._run_raw_handler(incoming)

    def _run_raw_handler(self, incoming):
        connection = FakeStreamConnection(incoming)
        daemon.LaserRequestHandler(
            connection, ("local", 0), self.server)
        return [
            json.loads(line.decode("utf-8"))
            for line in connection.outgoing.getvalue().splitlines()
        ]

    def test_handler_eof_forces_low(self):
        responses = self._run_handler([
            {"v": 1, "id": 1, "op": "hello", "client": "test"},
            {"v": 1, "id": 2, "op": "set", "state": "laser450"},
        ])
        self.assertEqual(len(responses), 2)
        self.assertTrue(responses[0]["ok"])
        self.assertTrue(responses[1]["ok"])
        self.assertTrue(responses[1]["status"]["ttl450_high"])
        # Handler.finish() models gateway EOF and must revoke the lease.
        self.assertEqual(self.gpio.values(), (False, False))
        self.assertFalse(self.service.status()["lease_active"])

    def test_handler_protocol_error_forces_low_and_closes(self):
        responses = self._run_handler([
            {"v": 1, "id": 1, "op": "hello", "client": "test"},
            {"v": 1, "id": 2, "op": "set", "state": "laser650"},
            {"v": 1, "id": 9, "op": "status", "shell": "id"},
            {"v": 1, "id": 10, "op": "heartbeat"},
        ])
        self.assertEqual(len(responses), 3)
        self.assertTrue(responses[1]["status"]["ttl650_high"])
        self.assertFalse(responses[2]["ok"])
        self.assertEqual(responses[2]["error"]["code"], "UNKNOWN_FIELD")
        self.assertEqual(self.gpio.values(), (False, False))
        self.assertFalse(self.service.status()["lease_active"])

    def test_handler_invalid_json_forces_low(self):
        hello = json.dumps(
            {"v": 1, "id": 1, "op": "hello", "client": "test"},
            separators=(",", ":")).encode("utf-8") + b"\n"
        turn_on = json.dumps(
            {"v": 1, "id": 2, "op": "set", "state": "laser450"},
            separators=(",", ":")).encode("utf-8") + b"\n"
        responses = self._run_raw_handler(
            hello + turn_on + b"{not-json}\n")
        self.assertEqual(len(responses), 3)
        self.assertEqual(responses[2]["error"]["code"], "INVALID_JSON")
        self.assertEqual(self.gpio.values(), (False, False))
        self.assertFalse(self.service.status()["lease_active"])


if __name__ == "__main__":
    unittest.main()
