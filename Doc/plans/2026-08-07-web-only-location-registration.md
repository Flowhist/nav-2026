# Web-Only Location Registration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the local Tk map annotation workflow while preserving Web location registration, navigation location loading, relocation, and RViz location markers.

**Architecture:** Web registration remains owned by `server/server_app.py` and `server/map_utils.py`, which write `<map>/<map>.locations.yaml`. Runtime consumers use `scripts/map_location/location_utils.py` for reading and map detection, while the GUI-only writer and map-list helper are removed with `annotate_tool.py`.

**Tech Stack:** Python 3.10, ROS 2 Humble, CMake, pytest.

---

### Task 1: Lock the Web-only boundary

**Files:**
- Create: `tests/test_web_location_registration.py`

- [x] Add a test proving `server.map_utils` can save and load location files without the local GUI.
- [x] Add a cleanup guard asserting `annotate_tool.py` and its CMake install entry are absent.
- [x] Run the cleanup guard before deletion and confirm it fails.

### Task 2: Remove the local annotation workflow

**Files:**
- Delete: `scripts/map_annotate/annotate_tool.py`
- Modify: `scripts/map_location/location_utils.py`
- Modify: `CMakeLists.txt`

- [x] Delete the Tk annotation executable.
- [x] Remove `discover_maps()` and `save_locations()`, which are referenced only by that executable.
- [x] Keep `resolve_maps_dir()`, `load_locations()`, `discover_maps_with_locations()`, and runtime map detection.
- [x] Remove the executable from the CMake install list.

### Task 3: Remove manual-use documentation

**Files:**
- Modify: `Doc/测试运行指令.md`
- Modify: `sim/仿真使用说明.md`

- [x] Remove the local `ros2 run finav annotate_tool.py` instructions.
- [x] Direct users to Web location registration where a replacement instruction is useful.

### Task 4: Verify the retained runtime

- [x] Run the focused Web registration and cleanup tests.
- [x] Run the full Python test suite.
- [x] Run `ament_flake8` on modified Python files.
- [x] Build the `finav` ROS package.
- [x] Search the repository for remaining `annotate_tool` and local manual-registration references.
