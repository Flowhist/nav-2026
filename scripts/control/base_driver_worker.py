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
                        int(request["acceleration"]),
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

    @property
    def pid(self) -> Optional[int]:
        process = self._process
        return process.pid if process is not None else None

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

    def restart(self):
        self.terminate()
        self.start()

    def terminate(self):
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
        if self.is_healthy:
            try:
                self._request("shutdown", timeout_s=self.terminate_timeout_s)
            except DriverWorkerError:
                pass
        self.terminate()

    def _request(self, operation: str, *, timeout_s: Optional[float] = None, **payload):
        timeout = self.request_timeout_s if timeout_s is None else max(0.05, timeout_s)
        deadline = time.monotonic() + timeout
        if not self._request_lock.acquire(timeout=timeout):
            raise DriverWorkerBusy(
                f"driver worker busy for more than {timeout:.2f}s"
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

    def set_velocity(self, motor_ids, left, right, acceleration):
        return self._request(
            "set_velocity",
            motor_ids=list(motor_ids),
            left=float(left),
            right=float(right),
            acceleration=int(acceleration),
        )

    def get_velocity(self, motor_ids):
        return self._request("get_velocity", motor_ids=list(motor_ids))

    def get_fault_status(self, motor_ids):
        return self._request("get_fault_status", motor_ids=list(motor_ids))

    def reset_fault_status(self, motor_ids):
        return self._request(
            "reset_fault_status",
            timeout_s=max(3.5, self.request_timeout_s),
            motor_ids=list(motor_ids),
        )
