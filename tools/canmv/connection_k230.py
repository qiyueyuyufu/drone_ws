#!/usr/bin/env python3
"""Receive UDP messages from a K230/CanMV device."""

import socket


LISTEN_HOST = "0.0.0.0"
LISTEN_PORT = 9000
BUFFER_SIZE = 4096


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_HOST, LISTEN_PORT))

    print(f"waiting udp on {LISTEN_HOST}:{LISTEN_PORT}...")

    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        print("from", addr, data.decode("utf-8"))


if __name__ == "__main__":
    main()
