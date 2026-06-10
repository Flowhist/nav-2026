#!/usr/bin/env python3
"""Configure the FREE lidar network address over its TCP protocol."""

import argparse
import ipaddress
import socket
import sys
import time
from pathlib import Path


READ_DOWN = 0x00
WRITE_DOWN = 0x01
METHOD_DOWN = 0x02
READ_UP = 0x10
WRITE_UP = 0x11
METHOD_UP = 0x12

CMD_LOGIN = 0x01
CMD_LOGOUT = 0x02
CMD_SAVE_USER_PARAMS = 0x06
CMD_REBOOT = 0x00
CMD_SET_IP = 0x0F
CMD_GET_IP = 0x10

LOGIN_CANDIDATES = (
    (0x02, bytes.fromhex("20 21 05 18"), "driver password"),
    (0x03, bytes.fromhex("F4 72 47 44"), "manual default password"),
)


class LidarProtocolError(RuntimeError):
    pass


def build_frame(op_code, cmd_code, payload=b""):
    frame = bytearray(b"\x02\x02\x02\x02")
    length = 4 + 2 + 1 + 1 + len(payload) + 1
    frame.extend(length.to_bytes(2, "big"))
    frame.append(op_code)
    frame.append(cmd_code)
    frame.extend(payload)
    frame.append(sum(frame) & 0xFF)
    return bytes(frame)


def parse_target_ip(value, current_ip):
    value = value.strip()
    if value.isdigit():
        octet = int(value)
        if octet < 0 or octet > 255:
            raise ValueError(f"invalid last octet: {value}")
        parts = current_ip.split(".")
        if len(parts) != 4:
            raise ValueError(f"current IP is invalid: {current_ip}")
        parts[-1] = str(octet)
        value = ".".join(parts)
    return str(ipaddress.IPv4Address(value))


def ip_payload(ip_text):
    return ipaddress.IPv4Address(ip_text).packed


def validate_response(frame, expected_op, expected_cmd, min_len=9):
    if len(frame) < min_len:
        raise LidarProtocolError(f"short response: {frame.hex(' ')}")
    if frame[:4] != b"\x02\x02\x02\x02":
        raise LidarProtocolError(f"invalid header: {frame.hex(' ')}")
    declared = int.from_bytes(frame[4:6], "big")
    if declared != len(frame):
        raise LidarProtocolError(f"length mismatch: declared={declared}, actual={len(frame)}")
    checksum = sum(frame[:-1]) & 0xFF
    if frame[-1] != checksum:
        raise LidarProtocolError(f"checksum mismatch: expected={checksum:02x}, got={frame[-1]:02x}")
    if frame[6] != expected_op or frame[7] != expected_cmd:
        raise LidarProtocolError(
            f"unexpected response op/cmd: got={frame[6]:02x}/{frame[7]:02x}, "
            f"expected={expected_op:02x}/{expected_cmd:02x}"
        )
    return frame[8:-1]


def transact(sock, op_code, cmd_code, payload=b"", expected_op=None, timeout=2.0):
    expected_op = expected_op if expected_op is not None else op_code + 0x10
    sock.settimeout(timeout)
    tx = build_frame(op_code, cmd_code, payload)
    sock.sendall(tx)
    rx = sock.recv(256)
    return validate_response(rx, expected_op, cmd_code)


def login(sock):
    last_error = None
    for user_level, password, label in LOGIN_CANDIDATES:
        try:
            payload = bytes([user_level]) + password
            body = transact(sock, METHOD_DOWN, CMD_LOGIN, payload, METHOD_UP)
            if body and body[0] == 0x01:
                print(f"[OK] login success using {label}")
                return
            last_error = LidarProtocolError(f"login rejected using {label}: {body.hex(' ')}")
        except (OSError, LidarProtocolError) as exc:
            last_error = exc
    raise LidarProtocolError(f"login failed: {last_error}")


def logout(sock):
    try:
        transact(sock, METHOD_DOWN, CMD_LOGOUT, expected_op=METHOD_UP, timeout=1.0)
    except Exception:
        pass


def read_device_ip(sock):
    body = transact(sock, READ_DOWN, CMD_GET_IP, expected_op=READ_UP)
    if len(body) != 4:
        raise LidarProtocolError(f"unexpected IP payload: {body.hex(' ')}")
    return str(ipaddress.IPv4Address(body))


def set_device_ip(sock, new_ip):
    body = transact(sock, WRITE_DOWN, CMD_SET_IP, ip_payload(new_ip), WRITE_UP)
    if len(body) != 4:
        raise LidarProtocolError(f"unexpected set-IP payload: {body.hex(' ')}")
    returned_ip = str(ipaddress.IPv4Address(body))
    if returned_ip != new_ip:
        raise LidarProtocolError(f"device returned {returned_ip}, expected {new_ip}")
    print(f"[OK] device accepted new IP: {returned_ip}")


def save_user_params(sock):
    body = transact(sock, METHOD_DOWN, CMD_SAVE_USER_PARAMS, expected_op=METHOD_UP)
    if not body or body[0] != 0x01:
        raise LidarProtocolError(f"save rejected: {body.hex(' ')}")
    print("[OK] user parameters saved")


def reboot_device(sock):
    body = transact(sock, METHOD_DOWN, CMD_REBOOT, expected_op=METHOD_UP, timeout=1.0)
    if body and body[0] != 0x01:
        raise LidarProtocolError(f"reboot rejected: {body.hex(' ')}")
    print("[OK] reboot command accepted")


def can_connect(ip_text, port, timeout):
    try:
        with socket.create_connection((ip_text, port), timeout=timeout):
            return True
    except OSError:
        return False


def update_lidar_yaml(config_path, old_ip, new_ip):
    text = config_path.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    replaced = False
    for idx, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("scanner_ip:") and old_ip in stripped:
            newline = "\n" if line.endswith("\n") else ""
            indent = line[: len(line) - len(line.lstrip())]
            lines[idx] = f"{indent}scanner_ip: {new_ip}{newline}"
            replaced = True
            break

    if not replaced:
        for idx, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith("lidar_ip:"):
                newline = "\n" if line.endswith("\n") else ""
                lines[idx] = f"lidar_ip: {new_ip}{newline}"
                replaced = True
                break

    if not replaced:
        for idx, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith("scanner_ip:"):
                newline = "\n" if line.endswith("\n") else ""
                indent = line[: len(line) - len(line.lstrip())]
                lines[idx] = f"{indent}scanner_ip: {new_ip}{newline}"
                replaced = True
                break

    if not replaced:
        has_dual_config = any(line.strip() in ("left:", "right:") for line in lines)
        if has_dual_config:
            raise LidarProtocolError(
                f"could not find scanner_ip matching {old_ip} in dual lidar config"
            )
        lines.append(f"\nlidar_ip: {new_ip}\n")
    config_path.write_text("".join(lines), encoding="utf-8")
    print(f"[OK] updated {config_path} ({old_ip} -> {new_ip})")


def default_config_path():
    return Path(__file__).resolve().parents[2] / "config" / "lidar.yaml"


def configure_lidar_ip(args):
    current_ip = str(ipaddress.IPv4Address(args.current_ip))
    new_ip = parse_target_ip(args.new_ip, current_ip)
    if current_ip == new_ip:
        raise ValueError("new IP is the same as current IP")

    print(f"[INFO] connecting {current_ip}:{args.port}")
    with socket.create_connection((current_ip, args.port), timeout=args.timeout) as sock:
        login(sock)
        before_ip = read_device_ip(sock)
        print(f"[INFO] device reports current IP: {before_ip}")
        if args.read_only:
            logout(sock)
            return before_ip
        if before_ip != current_ip:
            print(f"[WARN] connected IP differs from reported IP: {current_ip} != {before_ip}")
        set_device_ip(sock, new_ip)
        save_user_params(sock)
        if args.reboot_after_save:
            reboot_device(sock)
        logout(sock)

    if args.config:
        update_lidar_yaml(Path(args.config), current_ip, new_ip)

    print("[INFO] wait a few seconds, then reconnect using the new IP")
    time.sleep(args.post_wait)
    if can_connect(new_ip, args.port, args.timeout):
        print(f"[OK] active TCP endpoint is reachable at {new_ip}:{args.port}")
    else:
        print(
            f"[WARN] {new_ip}:{args.port} is not reachable yet; "
            "the device may require a power cycle for the new IP to take effect"
        )
    return new_ip


def main(argv=None):
    parser = argparse.ArgumentParser(description="Set FREE lidar IP and save it on the device.")
    parser.add_argument("--current-ip", default="10.86.81.111", help="current reachable lidar IP")
    parser.add_argument("--new-ip", required=True, help="new full IP or last octet shortcut, e.g. 112")
    parser.add_argument("--port", type=int, default=2111, help="lidar TCP command port")
    parser.add_argument("--timeout", type=float, default=3.0, help="socket timeout in seconds")
    parser.add_argument("--post-wait", type=float, default=2.0, help="seconds to wait after saving")
    parser.add_argument("--read-only", action="store_true", help="only read the device-reported IP")
    parser.add_argument(
        "--reboot-after-save",
        action="store_true",
        help="send the device reboot command after saving; the manual marks this command unsupported",
    )
    parser.add_argument(
        "--config",
        default=str(default_config_path()),
        help="lidar.yaml path to update after a successful device write; use '' to skip",
    )
    args = parser.parse_args(argv)
    if args.config == "":
        args.config = None

    try:
        new_ip = configure_lidar_ip(args)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    print(f"[OK] lidar IP configured to {new_ip}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
