import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "handle"))

from handle_modbus import (  # noqa: E402
    ModbusProtocolError,
    ModbusRtuClient,
    ModbusTimeout,
)
from handle_protocol import crc16_modbus  # noqa: E402


def _rtu_frame(payload: bytes) -> bytes:
    return payload + crc16_modbus(payload).to_bytes(2, "little")


class FakeSerial:
    def __init__(self, response: bytes):
        self._response = bytearray(response)
        self.writes = []
        self.reset_count = 0
        self.flush_count = 0
        self.closed = False

    def reset_input_buffer(self):
        self.reset_count += 1

    def write(self, data: bytes):
        self.writes.append(bytes(data))
        return len(data)

    def flush(self):
        self.flush_count += 1

    def read(self, size: int):
        data = bytes(self._response[:size])
        del self._response[:size]
        return data

    def close(self):
        self.closed = True


def _client(response: bytes):
    fake = FakeSerial(response)
    client = ModbusRtuClient(
        port="/dev/fake",
        baudrate=115200,
        slave_id=1,
        timeout=0.1,
        serial_factory=lambda **_: fake,
    )
    return client, fake


def test_read_holding_registers_builds_request_and_decodes_words():
    response = _rtu_frame(
        bytes.fromhex("01 03 08 00 03 03 E8 0B B8 00 04")
    )
    client, fake = _client(response)

    words = client.read_holding_registers(0x0003, 4)

    expected_request = _rtu_frame(bytes.fromhex("01 03 00 03 00 04"))
    assert fake.writes == [expected_request]
    assert fake.reset_count == 1
    assert fake.flush_count == 1
    assert words == [3, 1000, 3000, 4]


def test_write_single_register_validates_echo_response():
    response = _rtu_frame(bytes.fromhex("01 06 00 02 02 58"))
    client, fake = _client(response)

    client.write_single_register(0x0002, 600)

    assert fake.writes == [response]


def test_read_rejects_bad_crc():
    response = bytearray(
        _rtu_frame(bytes.fromhex("01 03 02 00 03"))
    )
    response[-1] ^= 0xFF
    client, _ = _client(bytes(response))

    with pytest.raises(ModbusProtocolError, match="CRC"):
        client.read_holding_registers(0x0003, 1)


def test_read_reports_modbus_exception_response():
    response = _rtu_frame(bytes.fromhex("01 83 02"))
    client, _ = _client(response)

    with pytest.raises(ModbusProtocolError, match="exception 0x02"):
        client.read_holding_registers(0x0003, 1)


def test_read_reports_short_response_as_timeout():
    client, _ = _client(bytes.fromhex("01 03"))

    with pytest.raises(ModbusTimeout, match="expected"):
        client.read_holding_registers(0x0003, 1)


def test_write_rejects_mismatched_echo():
    response = _rtu_frame(bytes.fromhex("01 06 00 02 00 64"))
    client, _ = _client(response)

    with pytest.raises(ModbusProtocolError, match="echo"):
        client.write_single_register(0x0002, 600)


def test_close_closes_serial_port():
    client, fake = _client(b"")

    client.close()

    assert fake.closed is True
