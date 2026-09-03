#!/usr/bin/env python3
"""Fail-safe line-laser GPIO service for LubanCat-4 V1.

The daemon is the sole owner of physical Pin 11 / GPIO15 (450 nm) and
physical Pin 7 / GPIO16 (650 nm).  It exposes a deliberately small,
newline-delimited JSON protocol on a Unix domain socket.

The protocol supports each laser independently and an explicit dual-laser
state.  A control lease and connection-scoped ownership ensure that a dead
client, broken SSH connection, or missed heartbeat turns both outputs off.
"""

import argparse
import fcntl
import grp
import json
import logging
import os
import signal
import socketserver
import stat
import struct
import threading
import time
import uuid


PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 4096
STATE_OFF = "off"
STATE_450 = "laser450"
STATE_650 = "laser650"
STATE_BOTH = "both"
VALID_STATES = frozenset((STATE_OFF, STATE_450, STATE_650, STATE_BOTH))


class LaserError(Exception):
    """An error that is safe to return to a protocol client."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)
        self.message = str(message)


def _iowr(ioctl_type, number, size):
    # Linux _IOC: read|write direction=3, size bits start at bit 16.
    return (3 << 30) | (int(size) << 16) | (int(ioctl_type) << 8) | int(number)


class CharacterDeviceGpio:
    """Minimal GPIO character-device v1 client with an atomic two-line handle."""

    GPIOHANDLE_REQUEST_OUTPUT = 1 << 1
    GPIOHANDLE_REQUEST_SIZE = 364
    GPIOHANDLE_DATA_SIZE = 64
    GPIO_GET_LINEHANDLE_IOCTL = _iowr(0xB4, 0x03, GPIOHANDLE_REQUEST_SIZE)
    GPIOHANDLE_GET_LINE_VALUES_IOCTL = _iowr(0xB4, 0x08, GPIOHANDLE_DATA_SIZE)
    GPIOHANDLE_SET_LINE_VALUES_IOCTL = _iowr(0xB4, 0x09, GPIOHANDLE_DATA_SIZE)

    def __init__(self, chip_path, offset_450=15, offset_650=16):
        if int(offset_450) == int(offset_650):
            raise ValueError("450 nm and 650 nm GPIO offsets must differ")
        self.chip_path = str(chip_path)
        self.offset_450 = int(offset_450)
        self.offset_650 = int(offset_650)
        self._handle_fd = -1

        flags = os.O_RDONLY
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        chip_fd = os.open(self.chip_path, flags)
        try:
            request = bytearray(self.GPIOHANDLE_REQUEST_SIZE)
            struct.pack_into("=I", request, 0, self.offset_450)
            struct.pack_into("=I", request, 4, self.offset_650)
            struct.pack_into("=I", request, 256, self.GPIOHANDLE_REQUEST_OUTPUT)
            # default_values begins at 260 and is already all zero (safe LOW).
            label = b"myline-line-laser"
            request[324:324 + len(label)] = label
            struct.pack_into("=I", request, 356, 2)
            fcntl.ioctl(
                chip_fd, self.GPIO_GET_LINEHANDLE_IOCTL, request, True)
            self._handle_fd = struct.unpack_from("=i", request, 360)[0]
            if self._handle_fd < 0:
                raise OSError("GPIO line request returned an invalid handle")
        finally:
            os.close(chip_fd)

        try:
            self.set_state(STATE_OFF)
        except Exception:
            self.close()
            raise

    def set_state(self, state):
        if state not in VALID_STATES:
            raise ValueError("invalid laser state: {}".format(state))
        values = bytearray(self.GPIOHANDLE_DATA_SIZE)
        values[0] = 1 if state in (STATE_450, STATE_BOTH) else 0
        values[1] = 1 if state in (STATE_650, STATE_BOTH) else 0
        fcntl.ioctl(
            self._handle_fd,
            self.GPIOHANDLE_SET_LINE_VALUES_IOCTL,
            values,
            True)
        actual_450, actual_650 = self.values()
        expected = (
            state in (STATE_450, STATE_BOTH),
            state in (STATE_650, STATE_BOTH))
        if (actual_450, actual_650) != expected:
            raise OSError(
                "GPIO readback mismatch: expected={} actual={}".format(
                    expected, (actual_450, actual_650)))

    def values(self):
        values = bytearray(self.GPIOHANDLE_DATA_SIZE)
        fcntl.ioctl(
            self._handle_fd,
            self.GPIOHANDLE_GET_LINE_VALUES_IOCTL,
            values,
            True)
        return bool(values[0]), bool(values[1])

    def close(self):
        if self._handle_fd < 0:
            return
        try:
            self.set_state(STATE_OFF)
        except Exception:
            logging.exception("Could not force GPIO LOW while closing")
        try:
            os.close(self._handle_fd)
        finally:
            self._handle_fd = -1


class LaserService:
    """Thread-safe laser state, ownership, and lease policy."""

    def __init__(self, backend, lease_ms=2000, board_model="unknown"):
        if int(lease_ms) < 500 or int(lease_ms) > 10000:
            raise ValueError("lease_ms must be in [500, 10000]")
        self._backend = backend
        self._lease_seconds = float(lease_ms) / 1000.0
        self._board_model = str(board_model)
        self._generation = uuid.uuid4().hex
        self._lock = threading.RLock()
        self._owner = None
        self._owner_label = ""
        self._lease_deadline = 0.0
        self._state = STATE_OFF
        self._last_error = ""
        self._fatal_error = ""
        with self._lock:
            self._force_off_locked()

    @property
    def generation(self):
        return self._generation

    def _force_off_locked(self):
        try:
            self._backend.set_state(STATE_OFF)
            actual = self._backend.values()
            if actual != (False, False):
                raise OSError("LOW readback failed: {}".format(actual))
            self._state = STATE_OFF
            self._last_error = ""
        except Exception as exc:
            self._state = STATE_OFF
            self._last_error = "failed to drive both GPIOs LOW: {}".format(exc)
            self._fatal_error = self._last_error
            logging.exception(self._last_error)
            raise LaserError("GPIO_FAILURE", self._last_error)

    def _clear_owner_locked(self):
        self._owner = None
        self._owner_label = ""
        self._lease_deadline = 0.0

    def _expire_locked(self, now=None):
        now = time.monotonic() if now is None else float(now)
        if self._owner is None or now < self._lease_deadline:
            return False
        logging.warning("Laser control lease expired; forcing both outputs LOW")
        try:
            self._force_off_locked()
        finally:
            self._clear_owner_locked()
        return True

    def expire_if_needed(self):
        with self._lock:
            return self._expire_locked()

    def acquire(self, session_id, client_label):
        with self._lock:
            self._expire_locked()
            if self._owner is not None and self._owner != session_id:
                raise LaserError(
                    "BUSY",
                    "another client holds the laser control lease")
            # Every new/repeated handshake starts from a known safe state.
            self._force_off_locked()
            self._owner = str(session_id)
            self._owner_label = str(client_label)[:64]
            self._lease_deadline = time.monotonic() + self._lease_seconds
            logging.info("Control lease acquired by %s", self._owner_label)
            return self._snapshot_locked()

    def heartbeat(self, session_id):
        with self._lock:
            self._require_owner_locked(session_id)
            self._lease_deadline = time.monotonic() + self._lease_seconds
            return self._snapshot_locked()

    def _require_owner_locked(self, session_id):
        self._expire_locked()
        if self._owner != str(session_id):
            raise LaserError(
                "NOT_OWNER",
                "this connection does not hold the control lease")

    def set_state(self, session_id, state):
        if state not in VALID_STATES:
            raise LaserError("INVALID_STATE", "unsupported laser state")
        with self._lock:
            self._require_owner_locked(session_id)
            try:
                # Enforce break-before-make whenever changing between two
                # non-OFF states. Each write updates both lines in one ioctl,
                # so the intermediate state is explicitly (LOW, LOW).
                actual_before = self._backend.values()
                expected = (
                    state in (STATE_450, STATE_BOTH),
                    state in (STATE_650, STATE_BOTH))
                if (state != STATE_OFF and
                        actual_before != (False, False) and
                        actual_before != expected):
                    self._backend.set_state(STATE_OFF)
                    if self._backend.values() != (False, False):
                        raise OSError(
                            "break-before-make LOW readback failed")
                self._backend.set_state(state)
                actual = self._backend.values()
                if actual != expected:
                    raise OSError(
                        "state readback mismatch: expected={} actual={}".format(
                            expected, actual))
                self._state = state
                self._last_error = ""
            except Exception as exc:
                try:
                    self._force_off_locked()
                finally:
                    self._clear_owner_locked()
                raise LaserError(
                    "GPIO_FAILURE",
                    "could not set {}: {}".format(state, exc))
            self._lease_deadline = time.monotonic() + self._lease_seconds
            logging.info("Laser state -> %s by %s", state, self._owner_label)
            return self._snapshot_locked()

    def off(self, session_id):
        with self._lock:
            self._expire_locked()
            is_owner = self._owner == str(session_id)
            self._force_off_locked()
            if is_owner:
                self._lease_deadline = time.monotonic() + self._lease_seconds
            else:
                # An emergency OFF from a non-owner revokes stale authority.
                self._clear_owner_locked()
            logging.info("Laser state -> off")
            return self._snapshot_locked()

    def disconnect(self, session_id):
        with self._lock:
            if self._owner != str(session_id):
                return
            try:
                self._force_off_locked()
            finally:
                logging.info("Control connection closed; lease released")
                self._clear_owner_locked()

    def status(self):
        with self._lock:
            self._expire_locked()
            return self._snapshot_locked()

    @property
    def fatal_error(self):
        with self._lock:
            return self._fatal_error

    def _snapshot_locked(self):
        try:
            high_450, high_650 = self._backend.values()
        except Exception as exc:
            high_450, high_650 = False, False
            self._last_error = "GPIO readback failed: {}".format(exc)
            self._fatal_error = self._last_error
        actual_state = STATE_OFF
        if high_450 and high_650:
            actual_state = STATE_BOTH
        elif high_450:
            actual_state = STATE_450
        elif high_650:
            actual_state = STATE_650
        remaining_ms = 0
        if self._owner is not None:
            remaining_ms = max(
                0, int(round(
                    (self._lease_deadline - time.monotonic()) * 1000.0)))
        return {
            "generation": self._generation,
            "board_model": self._board_model,
            "state": actual_state,
            "ttl450_high": bool(high_450),
            "ttl650_high": bool(high_650),
            "lease_active": self._owner is not None,
            "lease_owner": self._owner_label if self._owner is not None else "",
            "lease_remaining_ms": remaining_ms,
            "pin_map": {
                "laser450": {
                    "physical_pin": 11,
                    "gpio": self._backend.offset_450,
                    "chip_offset": self._backend.offset_450,
                },
                "laser650": {
                    "physical_pin": 7,
                    "gpio": self._backend.offset_650,
                    "chip_offset": self._backend.offset_650,
                },
            },
            "fault": self._last_error,
        }

    def shutdown(self):
        with self._lock:
            try:
                self._force_off_locked()
            finally:
                self._clear_owner_locked()
                self._backend.close()


def _validate_request(request):
    if not isinstance(request, dict):
        raise LaserError("INVALID_REQUEST", "request must be a JSON object")
    if request.get("v") != PROTOCOL_VERSION:
        raise LaserError("BAD_VERSION", "protocol version must be 1")
    request_id = request.get("id")
    if (isinstance(request_id, bool) or
            not isinstance(request_id, int) or
            request_id < 0 or request_id > 0x7FFFFFFFFFFFFFFF):
        raise LaserError("INVALID_ID", "id must be a non-negative integer")
    operation = request.get("op")
    if not isinstance(operation, str):
        raise LaserError("INVALID_OPERATION", "op must be a string")

    allowed = {
        "hello": frozenset(("v", "id", "op", "client")),
        "heartbeat": frozenset(("v", "id", "op")),
        "status": frozenset(("v", "id", "op")),
        "set": frozenset(("v", "id", "op", "state")),
        "off": frozenset(("v", "id", "op")),
        "goodbye": frozenset(("v", "id", "op")),
    }
    if operation not in allowed:
        raise LaserError("INVALID_OPERATION", "unsupported operation")
    unknown = set(request.keys()) - allowed[operation]
    if unknown:
        raise LaserError(
            "UNKNOWN_FIELD",
            "unknown request field(s): {}".format(
                ", ".join(sorted(str(item) for item in unknown))))
    return request_id, operation


def process_request(service, session_id, request):
    request_id = request.get("id", 0) if isinstance(request, dict) else 0
    try:
        request_id, operation = _validate_request(request)
        if operation == "hello":
            label = request.get("client")
            if not isinstance(label, str) or not label or len(label) > 64:
                raise LaserError(
                    "INVALID_CLIENT",
                    "client must be a non-empty string of at most 64 characters")
            status = service.acquire(session_id, label)
        elif operation == "heartbeat":
            status = service.heartbeat(session_id)
        elif operation == "status":
            status = service.status()
        elif operation == "set":
            state = request.get("state")
            if state not in VALID_STATES:
                raise LaserError("INVALID_STATE", "unsupported laser state")
            status = service.set_state(session_id, state)
        elif operation == "off":
            status = service.off(session_id)
        else:
            service.disconnect(session_id)
            status = service.status()
        return {
            "v": PROTOCOL_VERSION,
            "id": request_id,
            "ok": True,
            "status": status,
        }
    except LaserError as exc:
        try:
            status = service.status()
        except Exception:
            status = {}
        return {
            "v": PROTOCOL_VERSION,
            "id": request_id if isinstance(request_id, int) else 0,
            "ok": False,
            "error": {"code": exc.code, "message": exc.message},
            "status": status,
        }


class LaserRequestHandler(socketserver.StreamRequestHandler):
    def setup(self):
        super().setup()
        self.session_id = uuid.uuid4().hex

    def _write_response(self, response):
        payload = json.dumps(
            response, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.wfile.write(payload + b"\n")
        self.wfile.flush()

    def handle(self):
        while True:
            frame = self.rfile.readline(MAX_FRAME_BYTES + 2)
            if not frame:
                return
            if len(frame) > MAX_FRAME_BYTES or not frame.endswith(b"\n"):
                self._write_response({
                    "v": PROTOCOL_VERSION,
                    "id": 0,
                    "ok": False,
                    "error": {
                        "code": "FRAME_TOO_LARGE",
                        "message": "request frame exceeds 4096 bytes",
                    },
                    "status": self.server.laser_service.status(),
                })
                return
            try:
                request = json.loads(frame.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._write_response({
                    "v": PROTOCOL_VERSION,
                    "id": 0,
                    "ok": False,
                    "error": {
                        "code": "INVALID_JSON",
                        "message": "request is not valid UTF-8 JSON",
                    },
                    "status": self.server.laser_service.status(),
                })
                return
            response = process_request(
                self.server.laser_service, self.session_id, request)
            self._write_response(response)
            if not response["ok"]:
                return
            if isinstance(request, dict) and request.get("op") == "goodbye":
                return

    def finish(self):
        try:
            self.server.laser_service.disconnect(self.session_id)
        finally:
            super().finish()


class LaserUnixServer(
        socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True
    block_on_close = True

    def __init__(self, socket_path, laser_service):
        self.laser_service = laser_service
        super().__init__(socket_path, LaserRequestHandler)


def _remove_stale_socket(socket_path):
    try:
        info = os.lstat(socket_path)
    except FileNotFoundError:
        return
    if not stat.S_ISSOCK(info.st_mode):
        raise RuntimeError(
            "refusing to replace non-socket path: {}".format(socket_path))
    os.unlink(socket_path)


def create_server(socket_path, service, socket_group=None):
    socket_path = os.path.abspath(socket_path)
    parent = os.path.dirname(socket_path)
    if not os.path.isdir(parent):
        raise RuntimeError("socket directory does not exist: {}".format(parent))
    _remove_stale_socket(socket_path)
    old_umask = os.umask(0o117)
    try:
        server = LaserUnixServer(socket_path, service)
    finally:
        os.umask(old_umask)
    try:
        os.chmod(socket_path, 0o660)
        if socket_group:
            group_id = grp.getgrnam(socket_group).gr_gid
            os.chown(socket_path, -1, group_id)
    except Exception:
        server.server_close()
        _remove_stale_socket(socket_path)
        raise
    return server


def _read_board_model():
    try:
        with open("/proc/device-tree/model", "rb") as model_file:
            return model_file.read().replace(b"\x00", b"").decode(
                "utf-8", errors="replace").strip()
    except OSError:
        return ""


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--socket", default="/run/myline-hik/laser.sock")
    parser.add_argument(
        "--socket-group", default="line-laser-control")
    parser.add_argument("--gpio-chip", default="/dev/gpiochip0")
    parser.add_argument("--laser450-offset", type=int, default=15)
    parser.add_argument("--laser650-offset", type=int, default=16)
    parser.add_argument("--lease-ms", type=int, default=2000)
    parser.add_argument(
        "--skip-model-check", action="store_true",
        help="development only; never use for production installation")
    return parser.parse_args()


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s line-laser: %(message)s")
    args = parse_arguments()
    if args.laser450_offset != 15 or args.laser650_offset != 16:
        logging.error(
            "Refusing non-V1 pin mapping: 450 offset=%s, 650 offset=%s",
            args.laser450_offset,
            args.laser650_offset)
        return 1
    board_model = _read_board_model()
    if not args.skip_model_check and "LubanCat-4" not in board_model:
        logging.error(
            "Refusing to run on unexpected board model: %s",
            board_model or "unknown")
        return 1

    backend = None
    service = None
    server = None
    server_thread = None
    stop_event = threading.Event()

    def request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    try:
        backend = CharacterDeviceGpio(
            args.gpio_chip, args.laser450_offset, args.laser650_offset)
        service = LaserService(backend, args.lease_ms, board_model)
        server = create_server(args.socket, service, args.socket_group)
        server_thread = threading.Thread(
            target=server.serve_forever,
            name="line-laser-unix-server",
            kwargs={"poll_interval": 0.1},
            daemon=True)
        server_thread.start()
        logging.info(
            "Ready: 450nm=Pin11/GPIO15, 650nm=Pin7/GPIO16, socket=%s",
            args.socket)
        while not stop_event.wait(0.1):
            service.expire_if_needed()
            if service.fatal_error:
                raise RuntimeError(service.fatal_error)
    except Exception:
        logging.exception("Fatal line-laser daemon error")
        return 1
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        if server_thread is not None:
            server_thread.join(timeout=2.0)
        if service is not None:
            try:
                service.shutdown()
            except Exception:
                logging.exception("Failed to complete safe shutdown")
        elif backend is not None:
            backend.close()
        try:
            _remove_stale_socket(args.socket)
        except Exception:
            logging.exception("Could not remove Unix socket")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
