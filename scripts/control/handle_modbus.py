#!/usr/bin/env python3
"""Small Modbus RTU client for the STM32 handle protocol."""

import threading
from typing import Callable, Optional

import serial

from handle_protocol import crc16_modbus


class ModbusError(RuntimeError):
    """Base class for handle communication failures."""


class ModbusTimeout(ModbusError):
    """Raised when the serial response is incomplete."""


class ModbusProtocolError(ModbusError):
    """Raised when an RTU response is malformed or reports an exception."""


class ModbusRtuClient:
    """Blocking single-device client supporting functions 0x03 and 0x06."""

    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        slave_id: int,
        timeout: float,
        serial_factory: Optional[Callable[..., object]] = None,
    ):
        if not 1 <= int(slave_id) <= 247:
            raise ValueError("slave_id must be in range 1..247")
        self.slave_id = int(slave_id)
        factory = serial_factory or serial.Serial
        self._serial = factory(
            port=str(port),
            baudrate=int(baudrate),
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=float(timeout),
            write_timeout=float(timeout),
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self._lock = threading.Lock()

    @staticmethod
    def _frame(payload: bytes) -> bytes:
        return payload + crc16_modbus(payload).to_bytes(2, "little")

    def _read_exact(self, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = self._serial.read(size - len(data))
            if not chunk:
                raise ModbusTimeout(f"expected {size} bytes, got {len(data)}")
            data.extend(chunk)
        return bytes(data)

    def _exchange(self, request: bytes, function: int) -> bytes:
        with self._lock:
            self._serial.reset_input_buffer()
            written = self._serial.write(request)
            if written != len(request):
                raise ModbusTimeout(
                    f"expected to write {len(request)} bytes, wrote {written}"
                )
            self._serial.flush()

            header = self._read_exact(2)
            if header[0] != self.slave_id:
                raise ModbusProtocolError(
                    f"unexpected slave 0x{header[0]:02X}"
                )
            if header[1] == (function | 0x80):
                response = header + self._read_exact(3)
                self._validate_crc(response)
                raise ModbusProtocolError(
                    f"Modbus exception 0x{response[2]:02X}"
                )
            if header[1] != function:
                raise ModbusProtocolError(
                    f"unexpected function 0x{header[1]:02X}"
                )

            if function == 0x03:
                byte_count = self._read_exact(1)
                return header + byte_count + self._read_exact(byte_count[0] + 2)
            return header + self._read_exact(6)

    @staticmethod
    def _validate_crc(response: bytes) -> None:
        if len(response) < 4:
            raise ModbusProtocolError("response is too short for CRC")
        expected = crc16_modbus(response[:-2])
        received = int.from_bytes(response[-2:], "little")
        if received != expected:
            raise ModbusProtocolError(
                f"CRC mismatch: expected 0x{expected:04X}, got 0x{received:04X}"
            )

    def read_holding_registers(self, address: int, count: int) -> list[int]:
        if not 0 <= int(address) <= 0xFFFF:
            raise ValueError("address must fit in uint16")
        if not 1 <= int(count) <= 125:
            raise ValueError("count must be in range 1..125")

        payload = bytes((self.slave_id, 0x03))
        payload += int(address).to_bytes(2, "big")
        payload += int(count).to_bytes(2, "big")
        response = self._exchange(self._frame(payload), 0x03)
        self._validate_crc(response)

        byte_count = response[2]
        expected_count = int(count) * 2
        if byte_count != expected_count:
            raise ModbusProtocolError(
                f"expected {expected_count} data bytes, got {byte_count}"
            )
        data = response[3:-2]
        return [
            int.from_bytes(data[index:index + 2], "big")
            for index in range(0, len(data), 2)
        ]

    def write_single_register(self, address: int, value: int) -> None:
        if not 0 <= int(address) <= 0xFFFF:
            raise ValueError("address must fit in uint16")
        if not 0 <= int(value) <= 0xFFFF:
            raise ValueError("value must fit in uint16")

        payload = bytes((self.slave_id, 0x06))
        payload += int(address).to_bytes(2, "big")
        payload += int(value).to_bytes(2, "big")
        request = self._frame(payload)
        response = self._exchange(request, 0x06)
        self._validate_crc(response)
        if response != request:
            raise ModbusProtocolError("write response does not echo request")

    def close(self) -> None:
        self._serial.close()
