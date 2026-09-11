# 安装与构建

[文档导航](README.md) · [简体中文](../README.md) · [English](README.en.md) · [Français](README.fr.md)

## 前置条件

- Ubuntu 22.04 和 ROS 2 Humble
- 已安装并可 source 的 AimDK 工作区
- 可访问 ROS 2 apt 软件源
- `sudo`、`apt-get`、`colcon` 和 C++17 编译工具链
- RGB-D 相机驱动及机器人 URDF/TF

仓库已经内置一份用于 Demo IK 的简化 X2 URDF，因此不强制依赖额外 description 包。但相机
TF 和真机 frame 必须由机器人运行环境提供。内置模型不是完整官方描述，具体限制见
[远端 API 与内置 URDF](REMOTE_API_AND_URDF.md)。

不要通过 pip 安装 Pinocchio。Pinocchio、eigenpy 和 ROS Python 绑定必须与目标机器人的 ROS
发行版、Python ABI 和 CPU 架构一致。

## 获取工作区

项目预期目录结构：

```text
~/x2_grasp_ws/
|-- scripts/
|-- docs/
|-- example/
`-- src/x2_grasp/
```

如果源码位于其他路径，后续命令中的 `~/x2_grasp_ws` 需要对应替换。

## 安装系统依赖

```bash
cd ~/x2_grasp_ws
source /opt/ros/humble/setup.bash
./scripts/install_dependencies.sh
```

脚本根据 `${ROS_DISTRO:-humble}` 安装：

```text
ros-${ROS_DISTRO}-pinocchio
ros-${ROS_DISTRO}-pybind11-vendor
libopencv-dev
pybind11-dev
python3-numpy
python3-opencv
```

非 root 用户会使用 `sudo apt-get`。安装后验证：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -c \
  'import cv2, numpy, pinocchio; print(cv2.__version__, numpy.__version__, pinocchio.__version__)'
```

## 加载机器人环境

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc
```

必须确保能够找到 `aimdk_msgs`：

```bash
ros2 pkg prefix aimdk_msgs
```

## 构建

```bash
cd ~/x2_grasp_ws
colcon build --packages-select x2_grasp --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

验证包和接口：

```bash
ros2 pkg prefix x2_grasp
ros2 interface show x2_grasp/action/Grasp
ros2 interface show x2_grasp/action/ExecuteCommand
ros2 interface show x2_grasp/msg/PerceptionStatus
/usr/bin/python3 -c \
  'from x2_arm import native_backend_available; print(native_backend_available())'
```

最后一条命令应输出 `True`。`auto` 后端在扩展可用时选择 C++；输出 `False` 时仍可回退
Python，但性能不符合原生基准。显式要求原生后端可用：

```bash
ros2 run x2_grasp x2_ik_demo --ik-backend native --limits
```

`x2_command_publisher` 属于同一个 `x2_grasp` 功能包，不需要另外克隆、安装或构建其他功能包。
只要在执行 `colcon build` 前正确 source AimDK，使 `find_package(aimdk_msgs)` 成功，CMake 就会
自动编译该节点，`colcon` 会自动把它安装到 `install/x2_grasp/lib/x2_grasp/`。统一 launch 在
`command_backend:=auto` 时检测、启动并连接该节点。

构建后确认：

```bash
test -x install/x2_grasp/lib/x2_grasp/x2_command_publisher
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag ik_backend:=native command_backend:=native execute:=false
```

通用 WSL 没有 `aimdk_msgs` 时会明确跳过该真机目标，但仍构建 `ExecuteCommand` 接口、C++ 调度
核心和对应测试。这不代表真机发布节点已经完成 ABI 验证。

每个新终端都需要按 ROS 2、AimDK、当前工作区的顺序 source。

## 测试

```bash
colcon test --packages-select x2_grasp --event-handlers console_direct+
colcon test-result --verbose
```

目标环境应同时运行 Python、C++ geometry 和 ROS 接口测试。开发机没有 ROS 2 或 Pinocchio
时，只能执行不导入这些模块的单元测试。

## 更新源码

接口、CMake、C++、pybind11 或 `package.xml` 变化后必须重新执行 `colcon build`。仅修改
Python 文件且使用 `--symlink-install` 时通常不需要重新构建，但新脚本、新消息和新 Action
必须重新构建并 source。Release 性能数据不能用 Debug 构建复现。
