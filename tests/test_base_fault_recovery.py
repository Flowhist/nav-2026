import sys
import time
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "control"))

from base_control import advance_fault_clear_count, should_reset_fault  # noqa: E402
from base_driver_worker import (  # noqa: E402
    DriverWorkerError,
    DriverWorkerBusy,
    DriverWorkerClient,
)


def _error_worker(connection, _config):
    connection.send({"kind": "ready"})
    request = connection.recv()
    connection.send(
        {
            "kind": "response",
            "id": request["id"],
            "ok": False,
            "error": "RuntimeError: Motor STO fault",
        }
    )
    connection.recv()


def _slow_recovery_worker(connection, _config):
    connection.send({"kind": "ready"})
    request = connection.recv()
    time.sleep(0.7)
    connection.send(
        {
            "kind": "response",
            "id": request["id"],
            "ok": True,
            "result": None,
        }
    )
    connection.recv()


def test_fault_requires_three_consecutive_clear_samples():
    clear = [{"fault": 0}, {"fault": 0}]
    fault = [{"fault": 33555}, {"fault": 0}]

    count, recovered = advance_fault_clear_count(0, clear)
    assert (count, recovered) == (1, False)
    count, recovered = advance_fault_clear_count(count, clear)
    assert (count, recovered) == (2, False)
    count, recovered = advance_fault_clear_count(count, fault)
    assert (count, recovered) == (0, False)
    count, _ = advance_fault_clear_count(count, clear)
    count, _ = advance_fault_clear_count(count, clear)
    count, recovered = advance_fault_clear_count(count, clear)
    assert (count, recovered) == (3, True)


def test_fault_reset_only_after_physical_estop_is_released():
    assert should_reset_fault([{"fault": 33555, "emergency": 0}]) is True
    assert should_reset_fault([{"fault": 33555, "emergency": 1}]) is False
    assert should_reset_fault([{"fault": 0, "emergency": 0}]) is False


def test_busy_request_does_not_terminate_worker(monkeypatch):
    worker = DriverWorkerClient({}, request_timeout_s=0.05)
    terminated = False

    def mark_terminated():
        nonlocal terminated
        terminated = True

    monkeypatch.setattr(worker, "terminate", mark_terminated)
    worker._request_lock.acquire()
    try:
        with pytest.raises(DriverWorkerBusy):
            worker._request("get_velocity")
    finally:
        worker._request_lock.release()

    assert terminated is False


def test_reported_motor_fault_does_not_terminate_healthy_worker():
    worker = DriverWorkerClient(
        {},
        worker_target=_error_worker,
        context_name="fork",
        request_timeout_s=0.2,
    )
    worker.start()
    try:
        with pytest.raises(DriverWorkerError, match="STO"):
            worker._request("set_velocity")
        assert worker.is_healthy is True
    finally:
        worker.terminate()


def test_fault_recovery_has_its_own_longer_timeout():
    worker = DriverWorkerClient(
        {},
        worker_target=_slow_recovery_worker,
        context_name="fork",
        request_timeout_s=0.2,
    )
    worker.start()
    try:
        worker.recover_fault([1, 2])
        assert worker.is_healthy is True
    finally:
        worker.terminate()
