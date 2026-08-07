# STM32 Handle Communication Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the retired Linux HID joystick with a compact Modbus-RTU bridge that reads the STM32 joystick, buttons, and five-speed gear, publishes the existing Finav control topics, and writes actual wheelchair speed back to register `0x0002`.

**Architecture:** Keep `base_control.py`, the PCAN/WHILL driver, and `base_control_router.py` unchanged. A new `handle_control.py` node exclusively owns the serial port, uses a small `0x03`/`0x06` Modbus client, converts registers `0x0003` through `0x0006` into the existing `/joy`, `/js_state`, and `/js_cmd_vel` contract, and writes `abs(/odom_encoder.linear.x) * 1000` to `0x0002`.

**Tech Stack:** ROS 2 Humble, Python 3.10, rclpy, pyserial 3.5, pytest, ROS standard messages.

---

### Task 1: Pure handle protocol and control math

**Files:**
- Create: `scripts/control/handle_protocol.py`
- Create: `tests/test_handle_protocol.py`

- [ ] **Step 1: Write failing protocol tests**

Cover:

```python
assert crc16_modbus(b"\x01\x03\x00\x03\x00\x04") == 0xC80B
assert normalize_axis(2048, center=2048, dead_zone=50) == 0.0
assert normalize_axis(4095, center=2048, dead_zone=50) == 1.0
assert normalize_axis(0, center=2048, dead_zone=50) == -1.0
assert gear_scale(1) == 0.2
assert gear_scale(5) == 1.0
assert gear_scale(0) is None
assert speed_mps_to_register(-0.123) == 123
```

- [ ] **Step 2: Verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q tests/test_handle_protocol.py
```

Expected: import failure because `handle_protocol.py` does not exist.

- [ ] **Step 3: Implement minimal pure functions**

Implement register constants, Modbus CRC16, axis normalization, uniform five-gear scaling, speed encoding, and register parsing. Keep the module free of ROS and serial imports.

- [ ] **Step 4: Verify GREEN**

Run the focused test and expect all protocol tests to pass.

### Task 2: Minimal Modbus RTU serial client

**Files:**
- Create: `scripts/control/handle_modbus.py`
- Create: `tests/test_handle_modbus.py`

- [ ] **Step 1: Write failing frame and fake-serial tests**

Test `read_holding_registers(0x0003, 4)` request/response framing, `write_single_register(0x0002, 600)` echo validation, CRC rejection, exception responses, and short reads.

- [ ] **Step 2: Verify RED**

Expected: import failure because `handle_modbus.py` does not exist.

- [ ] **Step 3: Implement `ModbusRtuClient`**

Use an injected serial factory for tests and `serial.Serial` by default. Support only:

```python
read_holding_registers(address: int, count: int) -> list[int]
write_single_register(address: int, value: int) -> None
close() -> None
```

Serialize every exchange, reset stale input, flush writes, read the exact response length, and validate slave address, function, byte count, echo, and CRC.

- [ ] **Step 4: Verify GREEN**

Run focused protocol and Modbus tests.

### Task 3: ROS handle node

**Files:**
- Create: `scripts/control/handle_control.py`
- Create: `config/handle.yaml`
- Create: `launch/sub/handle.launch.py`
- Create: `tests/test_handle_control.py`

- [ ] **Step 1: Write failing pure state-conversion tests**

Test that gear 1 and gear 5 scale both linear and angular commands uniformly, invalid gears produce zero, the navigation button only emits on a rising edge, and communication failure produces an offline zero command.

- [ ] **Step 2: Verify RED**

Expected: missing handle-control helper API.

- [ ] **Step 3: Implement `HandleControl`**

Parameters:

```yaml
port: /dev/ttyUSB0
baudrate: 115200
slave_id: 1
poll_rate: 50.0
speed_write_rate: 10.0
timeout: 0.1
reconnect_interval: 1.0
axis_center: 2048
axis_min: 0
axis_max: 4095
dead_zone: 50
linear_direction: -1.0
angular_direction: -1.0
gear_scales: [0.2, 0.4, 0.6, 0.8, 1.0]
max_linear_speed: 0.6
max_angular_speed: 0.5
```

Publish `/joy`, `/js_state`, `/js_cmd_vel`, `/handle/buttons`, and a rising-edge `/handle/navigation_button`. Subscribe to `/odom_encoder`; write the encoded absolute actual speed to `0x0002` at 10 Hz.

- [ ] **Step 4: Verify GREEN**

Run focused tests with ROS Humble sourced.

### Task 4: Remove HID implementation and wire startup

**Files:**
- Delete: `scripts/control/joy_control.py`
- Delete: `scripts/tool/joystick_test.py`
- Delete: `config/joy.yaml`
- Delete: `launch/sub/joy.launch.py`
- Delete: `tests/test_joy_control.py`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`
- Modify: `start_finav.sh`
- Modify: `base_drive.sh`
- Modify: `server/process_manager.py`
- Modify: `server/server_app.py`
- Modify: `tests/test_config_restart_targets.py`

- [ ] **Step 1: Update restart-target tests and verify RED**

Replace the `joy.yaml` expectation with `handle.yaml`; expect failure until server metadata is updated.

- [ ] **Step 2: Remove HID-only files and references**

Remove `/dev/input/js*`, `--joy-dev`, `JOY_DEV`, `joy.launch.py`, `joy_control.py`, the ROS `joy` dependency, and the HID test utility.

- [ ] **Step 3: Add handle startup**

Use `--handle-port`, `HANDLE_PORT`, `handle.launch.py`, and `handle_control.py`. Preserve `/js_state` and `/js_cmd_vel` consumers.

- [ ] **Step 4: Verify GREEN**

Run the restart-target test, all handle tests, then the complete Python suite.

### Task 5: Align documentation and protocol

**Files:**
- Modify: `README.md`
- Modify: `Doc/项目Wiki.md`
- Modify: `Doc/navigation_topics.md`
- Modify: `../轮椅手柄协议.md`

- [ ] **Step 1: Document the final register contract**

Use hexadecimal addresses, five gears, seven button bits, no 485/heartbeat/MCU emergency-stop scope, and define `0x0002` as an upper-computer-written unsigned speed display register with `0.001 m/s` resolution.

- [ ] **Step 2: Replace HID instructions**

Document `--handle-port`, Modbus diagnostics, serial permissions, `/js_state`, `/js_cmd_vel`, `/handle/buttons`, and `/handle/navigation_button`.

- [ ] **Step 3: Verify no old HID references remain**

Run:

```bash
rg -n 'joy_control|joy\.launch|joy\.yaml|--joy-dev|JOY_DEV|/dev/input/js'
```

Expected: no source or documentation matches.

### Task 6: Final verification

- [ ] Run all Python tests after sourcing ROS Humble.
- [ ] Run `python3 -m py_compile` on the three new Python modules.
- [ ] Run ROS launch syntax checks with `ROS_LOG_DIR=/tmp/ros_log`.
- [ ] Run a scoped `colcon build --base-paths src/finav --packages-select finav` from the workspace root if the workspace layout permits.
- [ ] Report that real STM32 communication remains pending hardware connection.
