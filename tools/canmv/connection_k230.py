#!/usr/bin/env python3
"""Receive UDP messages from a K230/CanMV device."""

import socket
import json


LISTEN_HOST = "0.0.0.0"
LISTEN_PORT = 9000
BUFFER_SIZE = 4096


def get_local_ips() -> list[str]:
    ips = []
    hostname = socket.gethostname()

    try:
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ip = info[4][0]
            if ip not in ips:
                ips.append(ip)
    except OSError:
        pass

    return ips


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_HOST, LISTEN_PORT))
    sock.settimeout(2.0)

    print(f"waiting udp on {LISTEN_HOST}:{LISTEN_PORT}...")
    print("local ips:", get_local_ips())

    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
        except socket.timeout:
            print("still waiting...")
            continue

        text = data.decode("utf-8", errors="replace")
        print("from", addr, text)

        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            continue

        print("type:", msg.get("type"))
        if msg.get("type") == "qrcode":
            print("payload:", msg.get("payload"))
            print("rect:", msg.get("rect"))


if __name__ == "__main__":
    main()
