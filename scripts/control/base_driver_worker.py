#!/usr/bin/env python3
"""Run blocking omnilibs CAN operations in a restartable child process."""

import multiprocessing
import os
import sys
import threading
import time
from typing import Callable, Optional


class DriverWorkerError(RuntimeError):
    pass


class DriverWorkerTimeout(DriverWorkerError):
    pass


class DriverWorkerBusy(DriverWorkerTimeout):
    pass


def _normalized_acceleration(value):
    if isinstance(value, (list, tuple)):
        return [int(item) for item in value]
    return int(value)


def select_wheel_acceleration(
    previous_degps: float,
    target_degps: float,
    acceleration: int,
    deceleration: int,
    acceleration_reverse: int,
    acceleration_stop: int,
) -> int:
    previous = float(previous_degps)
    target = float(target_degps)
    if abs(target) <= 1e-6:
        return int(acceleration_stop)
    if abs(previous) <= 1e-6:
        return int(acceleration)
    if previous * target < 0.0:
        return int(acceleration_reverse)
    if abs(target) < abs(previous):
        return int(deceleration)
    return int(acceleration)


def select_wheel_accelerations(
    previous_left_degps: float,
    previous_right_degps: float,
    target_left_degps: float,
    target_right_degps: float,
    acceleration: int,
    deceleration: int,
    acceleration_reverse: int,
    acceleration_stop: int,
):
    return [
        select_wheel_acceleration(
            previous_left_degps,
            target_left_degps,
            acceleration,
            deceleration,
            acceleration_reverse,
            acceleration_stop,
        ),
        select_wheel_acceleration(
            previous_right_degps,
            target_right_degps,
            acceleration,
            deceleration,
            acceleration_reverse,
            acceleration_stop,
        ),
    ]


def _create_driver(config):
    sys.path.insert(0, str(config["omnilibs_path"]))
    from omnilibs.driver.driver import CAN, ONLINE, Driver

    driver = Driver()
    previous_cwd = os.getcwd()
    try:
        os.chdir(str(config["project_root"]))
        driver.load(
            "Whill",
            mode=ONLINE,
            parameters={
                CAN: {
                    "channel_name": config["can_channel"],
                    "interface": "pcan",
                    "baud_rate": int(config["can_baud_rate"]),
                    "canopen": 1,
                }
            },
        )
    finally:
        os.chdir(previous_cwd)

    motor_ids = list(config["motor_ids"])
    driver.move_velocity(
        motor_ids,
        [0.0] * len(motor_ids),
        int(config["acceleration_stop"]),
        wait_target=False,
    )
    return driver


def driver_worker_main(connection, config):
    driver = None
    try:
        driver = _create_driver(config)
        connection.send({"kind": "ready", "pid": os.getpid()})

        while True:
            request = connection.recv()
            request_id = request["id"]
            operation = request["operation"]
            try:
                if operation == "set_velocity":
                    result = driver.move_velocity(
                        list(request["motor_ids"]),
                        [float(request["left"]), float(request["right"])],
                        _normalized_acceleration(request["acceleration"]),
                        wait_target=False,
                    )
                elif operation == "get_velocity":
                    result = list(driver.get_velocity(list(request["motor_ids"])))
                elif operation == "get_fault_status":
                    result = list(driver.get_fault_status(list(request["motor_ids"])))
                elif operation == "reset_fault_status":
                    motor_ids = list(request["motor_ids"])
                    result = driver.reset_fault_status(motor_ids)
                elif operation == "shutdown":
                    motor_ids = list(config["motor_ids"])
                    driver.move_velocity(
                        motor_ids,
                        [0.0] * len(motor_ids),
                        int(config["acceleration_stop"]),
                        wait_target=False,
                    )
                    connection.send(
                        {
                            "kind": "response",
                            "id": request_id,
                            "ok": True,
                            "result": None,
                        }
                    )
                    break
                else:
                    raise ValueError(f"unsupported driver operation: {operation}")
            except Exception as exc:
                connection.send(
                    {
                        "kind": "response",
                        "id": request_id,
                        "ok": False,
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                )
            else:
                connection.send(
                    {
                        "kind": "response",
                        "id": request_id,
                        "ok": True,
                        "result": result,
                    }
                )
    except EOFError:
        pass
    except Exception as exc:
        try:
            connection.send(
                {
                    "kind": "startup_error",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )
        except Exception:
            pass
    finally:
        if driver is not None:
            try:
                driver.finalize()
            except Exception:
                pass
        try:
            connection.close()
        except Exception:
            pass


class DriverWorkerClient:
    def __init__(
        self,
        config: dict,
        *,
        worker_target: Callable = driver_worker_main,
        context_name: str = "spawn",
        startup_timeout_s: float = 8.0,
        request_timeout_s: float = 0.5,
        terminate_timeout_s: float = 0.5,
    ):
        self.config = dict(config)
        self._worker_target = worker_target
        self._context = multiprocessing.get_context(context_name)
        self.startup_timeout_s = max(0.1, float(startup_timeout_s))
        self.request_timeout_s = max(0.05, float(request_timeout_s))
        self.terminate_timeout_s = max(0.05, float(terminate_timeout_s))

        self._process = None
        self._connection = None
        self._healthy = False
        self._next_request_id = 1
        self._request_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._velocity_condition = threading.Condition()
        self._velocity_thread = None
        self._velocity_running = False
        self._pending_stop = None
        self._pending_motion = None
        self._velocity_error = None
        self._velocity_result = None
        self._last_dispatched_left = 0.0
        self._last_dispatched_right = 0.0

    @property
    def is_healthy(self) -> bool:
        process = self._process
        return bool(
            self._healthy
            and process is not None
            and process.is_alive()
            and self._connection is not None
        )

    def start(self):
        self.terminate()
        parent_connection, child_connection = self._context.Pipe(duplex=True)
        process = self._context.Process(
            target=self._worker_target,
            args=(child_connection, dict(self.config)),
            daemon=True,
        )
        process.start()
        child_connection.close()

        with self._lifecycle_lock:
            self._process = process
            self._connection = parent_connection
            self._healthy = False

        if not parent_connection.poll(self.startup_timeout_s):
            self.terminate()
            raise DriverWorkerTimeout(
                f"driver worker startup timed out after {self.startup_timeout_s:.2f}s"
            )

        try:
            response = parent_connection.recv()
        except (EOFError, OSError) as exc:
            self.terminate()
            raise DriverWorkerError(f"driver worker exited during startup: {exc}") from exc

        if response.get("kind") != "ready":
            self.terminate()
            raise DriverWorkerError(
                response.get("error", f"unexpected startup response: {response!r}")
            )
        self._healthy = True
        self._start_velocity_dispatcher()

    def restart(self):
        self.start()

    def terminate(self):
        self._stop_velocity_dispatcher()
        self._terminate_process()

    def _terminate_process(self):
        with self._lifecycle_lock:
            process = self._process
            connection = self._connection
            self._process = None
            self._connection = None
            self._healthy = False

        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

        if process is None:
            return
        if process.is_alive():
            process.terminate()
            process.join(self.terminate_timeout_s)
        if process.is_alive():
            process.kill()
            process.join(self.terminate_timeout_s)
        else:
            process.join(timeout=0)

    def stop(self):
        self._stop_velocity_dispatcher()
        if self.is_healthy:
            try:
                self._request("shutdown", timeout_s=self.request_timeout_s)
            except DriverWorkerError:
                pass
        self._terminate_process()

    def _request(
        self,
        operation: str,
        *,
        timeout_s: Optional[float] = None,
        lock_timeout_s: Optional[float] = None,
        **payload,
    ):
        timeout = self.request_timeout_s if timeout_s is None else max(0.05, timeout_s)
        deadline = time.monotonic() + timeout
        lock_timeout = timeout if lock_timeout_s is None else max(0.0, lock_timeout_s)
        if not self._request_lock.acquire(timeout=lock_timeout):
            raise DriverWorkerBusy(
                f"driver worker busy for more than {lock_timeout:.2f}s"
            )

        try:
            if not self.is_healthy:
                raise DriverWorkerError("driver worker is not healthy")

            request_id = self._next_request_id
            self._next_request_id += 1
            request = {"id": request_id, "operation": operation}
            request.update(payload)
            connection = self._connection

            try:
                connection.send(request)
            except (BrokenPipeError, EOFError, OSError) as exc:
                self.terminate()
                raise DriverWorkerError(f"driver worker send failed: {exc}") from exc

            remaining = max(0.0, deadline - time.monotonic())
            if not connection.poll(remaining):
                self.terminate()
                raise DriverWorkerTimeout(
                    f"driver operation {operation} timed out after {timeout:.2f}s"
                )

            try:
                response = connection.recv()
            except (EOFError, OSError) as exc:
                self.terminate()
                raise DriverWorkerError(f"driver worker response failed: {exc}") from exc

            if response.get("kind") != "response" or response.get("id") != request_id:
                self.terminate()
                raise DriverWorkerError(
                    f"driver worker response mismatch: {response!r}"
                )
            if not response.get("ok"):
                raise DriverWorkerError(
                    response.get("error", f"driver operation {operation} failed")
                )
            return response.get("result")
        finally:
            self._request_lock.release()

    def _start_velocity_dispatcher(self):
        with self._velocity_condition:
            self._velocity_running = True
            self._pending_stop = None
            self._pending_motion = None
            self._velocity_error = None
            self._velocity_result = None
            self._last_dispatched_left = 0.0
            self._last_dispatched_right = 0.0
            thread = threading.Thread(
                target=self._velocity_loop,
                name="finav-velocity-dispatcher",
                daemon=True,
            )
            self._velocity_thread = thread
        thread.start()

    def _stop_velocity_dispatcher(self):
        with self._velocity_condition:
            self._velocity_running = False
            self._pending_stop = None
            self._pending_motion = None
            thread = self._velocity_thread
            self._velocity_thread = None
            self._velocity_condition.notify_all()
        if thread is not None and thread is not threading.current_thread():
            thread.join(self.terminate_timeout_s)

    def set_velocity_latest(
        self,
        motor_ids,
        left,
        right,
        acceleration,
        deceleration,
        acceleration_reverse,
        acceleration_stop,
        *,
        urgent_stop: bool = False,
        source_age_s: float = -1.0,
    ):
        if not self.is_healthy:
            raise DriverWorkerError("driver worker is not healthy")

        urgent_stop = bool(urgent_stop)
        command = {
            "motor_ids": list(motor_ids),
            "left": float(left),
            "right": float(right),
            "acceleration": int(acceleration),
            "deceleration": int(deceleration),
            "acceleration_reverse": int(acceleration_reverse),
            "acceleration_stop": int(acceleration_stop),
            "source_age_s": float(source_age_s),
            "queued_mono": time.monotonic(),
            "replaced_count": 0,
        }
        with self._velocity_condition:
            if self._velocity_error is not None:
                error = self._velocity_error
                self._velocity_error = None
                raise error
            if not self._velocity_running:
                raise DriverWorkerError("velocity dispatcher is not running")

            if urgent_stop:
                replaced_count = 0
                for pending in (self._pending_stop, self._pending_motion):
                    if pending is not None:
                        replaced_count += pending["replaced_count"] + 1
                command["replaced_count"] = replaced_count
                self._pending_stop = command
                self._pending_motion = None
            else:
                replaced = self._pending_motion
                if replaced is not None:
                    command["replaced_count"] = replaced["replaced_count"] + 1
                self._pending_motion = command
            self._velocity_condition.notify()

    def _velocity_loop(self):
        while True:
            with self._velocity_condition:
                while (
                    self._velocity_running
                    and self._pending_stop is None
                    and self._pending_motion is None
                ):
                    self._velocity_condition.wait()
                if not self._velocity_running:
                    return
                if self._pending_stop is not None:
                    command = self._pending_stop
                    self._pending_stop = None
                else:
                    command = self._pending_motion
                    self._pending_motion = None
            if command is None:
                continue

            started_mono = time.monotonic()
            accelerations = select_wheel_accelerations(
                self._last_dispatched_left,
                self._last_dispatched_right,
                command["left"],
                command["right"],
                command["acceleration"],
                command["deceleration"],
                command["acceleration_reverse"],
                command["acceleration_stop"],
            )
            try:
                self._request(
                    "set_velocity",
                    motor_ids=command["motor_ids"],
                    left=command["left"],
                    right=command["right"],
                    acceleration=accelerations,
                )
            except DriverWorkerError as exc:
                with self._velocity_condition:
                    self._velocity_error = exc
                    self._pending_stop = None
                    self._pending_motion = None
                    if not self.is_healthy:
                        self._velocity_running = False
                    self._velocity_condition.notify_all()
                if not self.is_healthy:
                    return
                continue

            finished_mono = time.monotonic()
            self._last_dispatched_left = command["left"]
            self._last_dispatched_right = command["right"]
            source_age_s = command["source_age_s"]
            result = {
                "source_age_ms": source_age_s * 1000.0 if source_age_s >= 0.0 else -1.0,
                "queue_delay_ms": (started_mono - command["queued_mono"]) * 1000.0,
                "driver_duration_ms": (finished_mono - started_mono) * 1000.0,
                "total_duration_ms": (finished_mono - command["queued_mono"]) * 1000.0,
                "left": command["left"],
                "right": command["right"],
                "accelerations": accelerations,
                "replaced_count": command["replaced_count"],
            }
            with self._velocity_condition:
                self._velocity_result = result

    def take_velocity_error(self):
        with self._velocity_condition:
            error = self._velocity_error
            self._velocity_error = None
            return error

    def take_velocity_result(self):
        with self._velocity_condition:
            result = self._velocity_result
            self._velocity_result = None
            return result

    def get_velocity(self, motor_ids):
        return self._request(
            "get_velocity",
            lock_timeout_s=0.0,
            motor_ids=list(motor_ids),
        )

    def get_fault_status(self, motor_ids):
        return self._request(
            "get_fault_status",
            lock_timeout_s=0.0,
            motor_ids=list(motor_ids),
        )

    def reset_fault_status(self, motor_ids):
        return self._request(
            "reset_fault_status",
            timeout_s=max(3.5, self.request_timeout_s),
            motor_ids=list(motor_ids),
        )
