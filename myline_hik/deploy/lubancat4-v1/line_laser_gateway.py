#!/usr/bin/env python3
"""Restricted SSH forced-command gateway for the line-laser Unix socket."""

import os
import selectors
import socket
import sys


SOCKET_PATH = "/run/myline-hik/laser.sock"
COPY_CHUNK = 65536


def main():
    # A forced-command key must never become a generic remote command channel.
    if os.environ.get("SSH_ORIGINAL_COMMAND", "").strip():
        print("Remote commands are not accepted.", file=sys.stderr)
        return 2

    connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        connection.connect(SOCKET_PATH)
    except OSError as exc:
        print("Laser service unavailable: {}".format(exc), file=sys.stderr)
        return 1

    selector = selectors.DefaultSelector()
    stdin_open = True
    try:
        selector.register(sys.stdin.buffer, selectors.EVENT_READ, "stdin")
        selector.register(connection, selectors.EVENT_READ, "socket")
        while True:
            events = selector.select()
            for key, _mask in events:
                if key.data == "stdin":
                    data = os.read(sys.stdin.fileno(), COPY_CHUNK)
                    if not data:
                        selector.unregister(sys.stdin.buffer)
                        stdin_open = False
                        connection.shutdown(socket.SHUT_WR)
                        continue
                    connection.sendall(data)
                else:
                    data = connection.recv(COPY_CHUNK)
                    if not data:
                        return 0
                    os.write(sys.stdout.fileno(), data)
            if not stdin_open:
                # The daemon closes after seeing EOF; keep draining its final
                # response so one-shot diagnostics also work.
                continue
    except (BrokenPipeError, ConnectionError, OSError):
        return 1
    finally:
        selector.close()
        connection.close()


if __name__ == "__main__":
    raise SystemExit(main())
