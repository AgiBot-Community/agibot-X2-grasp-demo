<p align="center">
  <a href="https://github.com/AgiBot-Community">
    <img src="assets/agibot-community.png" alt="AgiBot Community logo" width="152">
  </a>
</p>

<h1 align="center">X2 Grasp</h1>

<p align="center">
  <a href="../README.md">简体中文</a> · <strong>English</strong> · <a href="README.fr.md">Français</a>
</p>

<p align="center">ROS 2 visual grasping demo for the AgiBot X2 robot</p>

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB.svg)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](https://isocpp.org/)

X2 Grasp is a ROS 2 Humble package for visual grasping on the X2 robot. It combines
AprilTag detection, visual grounding, RGB-D localization, C++ Pinocchio inverse
kinematics (IK), arm control, and grasp orchestration in one `x2_grasp` package.
A standard ROS 2 Action exposes feedback, results, and cancellation.

> Start with `execute:=false`. Enable real motion only after checking target
> coordinates, TF, IK, and trajectories, clearing the arm workspace, and verifying
> that the emergency stop is available.

## Features

- One ROS 2 package for interfaces, perception, IK, control, and orchestration.
- `x2_grasp/action/Grasp` API with feedback, results, and cancellation.
- AprilTag, Grounding, and automatic arbitration modes.
- Typed `PerceptionStatus` messages rather than encoded status strings.
- RGB-D timestamp matching, retained depth frames, TF conversion, and retries.
- Pinocchio IK with multiple seeds, staged Cartesian planning, and execution guards.
- Native C++17/pybind11 IK with a Python comparison backend and automatic fallback.
- A separate rclcpp arm/gripper publisher with absolute deadlines for 50 Hz commands.
- A bundled simplified X2 URDF for demo IK; no extra description package is needed to try it.
- One active grasp goal at a time, with cancellation checks throughout the workflow.
- The same perception and planning path for dry-run and real execution.

> Grounding uses the remote Volcengine Ark vision API: RGB images are sent over
> HTTPS to the configured server. AprilTag detection runs locally. Automatic mode
> can also call the remote API when no fresh tag is available. The bundled URDF
> is a simplified IK model without full collision, control, or high-fidelity
> simulation support. See [deployment boundaries](REMOTE_API_AND_URDF.md) (Chinese).

## Requirements

| Component | Requirement |
| --- | --- |
| Operating system | Ubuntu 22.04 on the target robot architecture |
| ROS 2 | Humble |
| Robot SDK | AimDK and `aimdk_msgs` |
| IK | Pinocchio from the ROS apt repository |
| Camera | RGB, Depth, CameraInfo, and camera-to-`base_link` TF |

Use the system Python 3.10 environment with the matching ROS binaries. Do not
replace ROS Pinocchio with a pip package; Python ABI and CPU architecture must match.

## Quick start

### 1. Get the source

```bash
git clone \
  https://github.com/AgiBot-Community/agibot-X2-grasp-demo.git \
  ~/x2_grasp_ws
cd ~/x2_grasp_ws
```

### 2. Install dependencies

Install AimDK beforehand, then load the robot environment. Adjust paths if your
workspace or SDK is installed elsewhere.

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc
./scripts/install_dependencies.sh
```

The script installs `ros-humble-pinocchio`, `ros-humble-pybind11-vendor`,
`pybind11-dev`, `libopencv-dev`, `python3-numpy`, and `python3-opencv` through apt.
Non-root users need `sudo`.

`x2_command_publisher` is built within this package. When CMake finds `aimdk_msgs`,
it builds the C++ hardware node and colcon installs it. The unified launch starts
and uses it with `command_backend:=auto`. A development environment without
`aimdk_msgs` skips this hardware target; that does not validate deployment on a robot.

### 3. Build and verify

```bash
colcon build --packages-select x2_grasp --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 interface show x2_grasp/action/Grasp
ros2 interface show x2_grasp/action/ExecuteCommand
ros2 interface show x2_grasp/msg/PerceptionStatus
/usr/bin/python3 -c \
  'from x2_arm import native_backend_available; print(native_backend_available())'
```

The final command should print `True` for the native IK backend. `auto` falls back
to Python if the extension is unavailable; `native` requires it and fails if missing.

### 4. Start a dry-run

In terminal A, choose one mode. For AprilTag, place the configured 36h11 tag in view:

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag ik_backend:=auto execute:=false
```

For Grounding:

```bash
export ARK_API_KEY='your-api-key'
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=grounding execute:=false
```

For automatic arbitration, set the same key and use `mode:=auto`. Each goal first
checks for a fresh AprilTag target, then requests Grounding if needed. The source
does not switch during arm execution. The launch default is `mode:=grounding`.

### 5. Send a grasp goal

In terminal B, load ROS, AimDK, and the workspace again:

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc
source ~/x2_grasp_ws/install/setup.bash

ros2 action send_goal /x2_grasp/grasp x2_grasp/action/Grasp \
  "{target: cup}" --feedback
```

The default targets are `cup` (paper cup), `bread`, and `bottle` (medicine bottle).
Keep these configured identifiers in commands regardless of documentation language.
In AprilTag mode the target type selects the gripper setting and completion sound;
it does not identify the object visually. A second concurrent goal is rejected.

## Examples and cancellation

From the repository root, the [Python example](../example/send_grasp_goal.py) handles
feedback, results, timed cancellation, and cancellation on `Ctrl+C`:

```bash
python3 example/send_grasp_goal.py cup
python3 example/send_grasp_goal.py bottle --cancel-after 5
```

The installed client provides the same entry point:

```bash
ros2 run x2_grasp grasp_action_client bread
ros2 run x2_grasp grasp_action_client bottle --cancel-after 5
```

Cancellation is cooperative. It stops subsequent commands and cancels the internal
`ExecuteCommand` Action during execution. With `hold_on_stop=true`, the C++ publisher
resends the last position as a hold. Cancellation cannot undo commands already sent
and is not a hardware emergency stop. See the [example walkthrough](../example/README.md) (Chinese).

## Real robot execution

Verify the native IK extension and installed publisher, camera calibration, tag
size, depth scale, and TF. Dry-run each required target and mode. Coordinates must
be in meters in `base_link` and within the selected arm's reachable workspace.
The default `arm_side:=auto` evaluates both arms; use `left` or `right` for calibration.
Clear the workspace and check the emergency stop and AimDK control mode before running:

```bash
test -x install/x2_grasp/lib/x2_grasp/x2_command_publisher
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag ik_backend:=native command_backend:=native execute:=true
```

Start with one low-risk goal. `execute:=false` performs perception and planning
without changing the robot mode or sending motion trajectories.

## Workflow

```text
Grasp Action goal
       |
       v
AprilTag / Grounding / automatic arbitration
       |
       v
base_link target + typed perception status
       |
       v
C++ Pinocchio IK and staged Cartesian planning
       |
       v
dry-run result or AimDK arm/gripper execution
```

## Documentation

The main guide is available in [Chinese](../README.md), [English](README.en.md), and
[French](README.fr.md). The detailed references below are currently in Chinese.
Use the [documentation index](README.md) to navigate within the repository.

| Document | Contents |
| --- | --- |
| [Installation and build](INSTALLATION.md) | ROS, AimDK, dependencies, build, verification |
| [Usage](USAGE.md) | Modes, Actions, cancellation, real robot procedure |
| [Interfaces](INTERFACES.md) | Actions, messages, topics, field semantics |
| [Configuration](CONFIGURATION.md) | Perception, IK, trajectories, audio |
| [Architecture](ARCHITECTURE.md) | Responsibilities, threads, data flow |
| [Performance and C++ migration](PERFORMANCE.md) | Native IK scope, measurements, test conditions |
| [Command publishing](PUBLISHING.md) | C++ publisher, cancellation, watchdog, acceptance checks |
| [Remote API and bundled URDF](REMOTE_API_AND_URDF.md) | Data transfer and simplified model limitations |
| [Troubleshooting](TROUBLESHOOTING.md) | Installation, TF, perception, IK, execution |
| [Package guide](../src/x2_grasp/README.md) | Package internals and parameters |

## Repository layout

```text
.
|-- example/                   # Runnable examples
|-- docs/                      # Operating and design references
|-- scripts/                   # Dependency installation and utilities
`-- src/x2_grasp/
    |-- action/                # ROS 2 Actions
    |-- msg/                   # Typed messages
    |-- config/                # Unified configuration
    |-- launch/                # Unified launch file
    |-- x2_grasp/              # Perception and orchestration
    |-- x2_arm/                # IK selection, trajectories, hardware control
    |-- x2_common/             # Shared Python utilities
    `-- src/                   # C++ RGB-D, IK, and command publishing
```

## Tests and benchmarks

```bash
colcon test --packages-select x2_grasp --event-handlers console_direct+
colcon test-result --verbose
```

Python tests:

```bash
PYTHONPATH=src/x2_grasp python3 -m pytest src/x2_grasp/test
```

Pinocchio benchmarks:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 scripts/benchmark_ik.py --backend python
/usr/bin/python3 scripts/benchmark_ik.py --backend native
```

The benchmark reports mean, median, and P95 for solver construction, joint/configuration
mapping, FK, position/orientation/6D/axis IK, an eight-point planning chain, and
trajectory interpolation. Use explicit backends and a Release build for comparisons.
Full tests require ROS 2, AimDK, Pinocchio, OpenCV, and target robot messages.
Report the tests actually run when opening an issue or pull request.

## Configuration and credentials

Edit [unified_grasp.yaml](../src/x2_grasp/config/unified_grasp.yaml) to configure camera
topics, calibration, grasp geometry, target types, and audio. Grounding reads the
key from `ARK_API_KEY` first, then from a private `~/.x2_arm/api_key.yaml` file
containing `api_key: "your-api-key"`. Never commit real credentials.

To add a target, update the catalog at the top of the YAML file:

```yaml
target_names: [cup, bread, bottle, apple]
target_descriptions: [一次性纸杯, 长条袋装面包, 长条药瓶, 红色苹果]
target_aliases: [paper_cup=cup, medicine_bottle=bottle, fruit=apple]
target_grip_close_positions: [0.10, 0.10, 0.10, 0.25]
target_pcm_paths: [cup.pcm, bread.pcm, bottle.pcm, ""]
default_pcm_path: grasp_complete.pcm
```

The Chinese descriptions are example model prompts, retained consistently across
the guides; they describe a disposable paper cup, packaged bread, a medicine bottle,
and a red apple. The names, descriptions, gripper positions, and PCM paths must have
equal lengths and matching indices. Aliases are independent `alias=target` mappings.
Gripper values range from `0.0` (fully closed) to `1.0` (fully open) and need calibration.
Missing or unreadable target audio falls back to `default_pcm_path`; audio uses
16 kHz mono S16LE PCM. See the [configuration reference](CONFIGURATION.md).

## Common problems

- Missing Pinocchio: install `ros-humble-pinocchio` and source ROS; do not substitute a pip version.
- Missing Action type: rebuild and source the workspace's `install/setup.bash`.
- Rejected goal: check the configured target name and whether another goal is active.
- No target coordinates: check camera topics, TF, and tag calibration; for Grounding, also check credentials, networking, and RGB-D synchronization.
- Dry-run works but the robot does not move: check `execute`, AimDK services, and control mode using the [usage procedure](USAGE.md).

For diagnostic commands, see [troubleshooting](TROUBLESHOOTING.md).

## License

This project is licensed under the [Apache License 2.0](../LICENSE). Third-party ROS,
AimDK, model services, and robot resources remain subject to their own licenses and
terms. See [brand assets](assets/README.md) for the logo attribution.
