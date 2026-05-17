import importlib.util
from pathlib import Path


def _load_module():
    repo_root = Path(__file__).resolve().parents[1]
    script = repo_root / "scripts" / "tool" / "config_lidar_ip.py"
    spec = importlib.util.spec_from_file_location("config_lidar_ip", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_frame_adds_length_and_checksum():
    mod = _load_module()

    frame = mod.build_frame(0x01, 0x0F, b"\xC0\xA8\x01\x70")

    assert frame == bytes.fromhex("02 02 02 02 00 0D 01 0F C0 A8 01 70 FE")


def test_parse_ip_accepts_last_octet_shortcut():
    mod = _load_module()

    assert mod.parse_target_ip("112", "192.168.1.111") == "192.168.1.112"
