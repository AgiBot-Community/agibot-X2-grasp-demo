<p align="center">
  <a href="https://github.com/AgiBot-Community">
    <img
      src="docs/assets/agibot-community.png"
      alt="AgiBot Community logo"
      width="152"
    >
  </a>
</p>

<h1 align="center">X2 Grasp</h1>

<p align="center">
  ROS 2 visual grasping demo for the AgiBot X2 robot
</p>

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB.svg)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](https://isocpp.org/)

面向 X2 机器人的 ROS 2 Humble 视觉抓取功能包。项目将 AprilTag 定位、视觉 Grounding、
RGB-D 三维定位、C++ Pinocchio IK、机械臂控制和抓取编排整合为一个 `x2_grasp` 包，并通过标准
ROS 2 Action 对外提供可反馈、可取消的抓取接口。

> 真机安全提示：首次运行必须保持 `execute:=false`。确认目标坐标、TF、IK 和轨迹均正确后，
> 才能在清空机械臂工作空间并确认急停可用的前提下启用真实运动。

## 特性

- 单一 ROS 2 功能包，接口、视觉、IK、控制和编排统一构建
- `x2_grasp/action/Grasp` Action API，支持 feedback、result 和 cancel
- AprilTag、Grounding、自动仲裁三种目标获取模式
- 强类型 `PerceptionStatus`，不使用 JSON 或字符串拼接状态
- RGB-D 时间戳关联、深度帧保留、TF 转换和异常重试
- Pinocchio 多初值 IK、分段笛卡尔路径与真机执行保护
- C++17 Pinocchio/pybind11 原生 IK 后端，保留可对比的 Python 后端和自动回退
- 内置面向 Demo IK 的简化 X2 URDF，无需额外 description 包即可试运行
- 同一时间只执行一个抓取 goal，并在各执行阶段检查取消请求
- dry-run 与真机执行使用相同规划链路

> Grounding 识别使用火山方舟远端视觉 API，RGB 图像会通过 HTTPS 发送到配置的服务端。
> AprilTag 模式完全本地运行。内置 URDF 是 IK 用简化模型，不包含完整碰撞、控制和高保真
> 仿真描述。部署边界详见 [远端 API 与内置 URDF](docs/REMOTE_API_AND_URDF.md)。

## 系统要求

| 组件 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 22.04，目标机器人架构 |
| ROS 2 | Humble |
| 机器人 SDK | AimDK 及 `aimdk_msgs` |
| IK | ROS apt 提供的 Pinocchio |
| 相机 | RGB、Depth、CameraInfo 和相机到 `base_link` 的 TF |

## 快速开始

### 1. 获取源码

```bash
git clone \
  https://github.com/AgiBot-Community/agibot-X2-grasp-demo.git \
  ~/x2_grasp_ws
cd ~/x2_grasp_ws
```

### 2. 安装依赖

```bash
cd ~/x2_grasp_ws
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc

./scripts/install_dependencies.sh
```

安装脚本通过 apt 安装 `ros-humble-pinocchio`、`ros-humble-pybind11-vendor`、
`pybind11-dev`、`libopencv-dev`、`python3-numpy` 和 `python3-opencv`。

### 3. 构建

```bash
colcon build --packages-select x2_grasp --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

检查生成的接口：

```bash
ros2 interface show x2_grasp/action/Grasp
ros2 interface show x2_grasp/msg/PerceptionStatus
/usr/bin/python3 -c \
  'from x2_arm import native_backend_available; print(native_backend_available())'
```

### 4. 启动 dry-run

AprilTag 示例：

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag ik_backend:=auto execute:=false
```

Grounding 示例：

```bash
export ARK_API_KEY='your-api-key'
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=grounding execute:=false
```

### 5. 提交抓取任务

在另一个已加载 ROS、AimDK 和工作区环境的终端执行：

```bash
ros2 action send_goal /x2_grasp/grasp x2_grasp/action/Grasp \
  "{target: cup}" --feedback
```

默认目标为 `cup`、`bread`、`bottle`，可在统一配置中扩展。也可以使用项目提供的客户端：

```bash
ros2 run x2_grasp grasp_action_client bread
ros2 run x2_grasp grasp_action_client bottle --cancel-after 5
```

## 基本 Demo

根目录 [example](example/README.md) 提供可直接阅读和运行的示例：

- `send_grasp_goal.py`：完整 Python Action 客户端
- Action feedback/result 处理
- 定时取消与 `Ctrl+C` 取消
- AprilTag、Grounding、自动仲裁模式的双终端操作步骤
- 感知状态和目标坐标监控命令

```bash
python3 example/send_grasp_goal.py cup
python3 example/send_grasp_goal.py bottle --cancel-after 5
```

## 工作流程

```text
Grasp Action goal
       |
       v
AprilTag / Grounding / Auto arbitration
       |
       v
base_link target + typed perception status
       |
       v
C++ Pinocchio IK and staged Cartesian planning
       |
       v
dry-run result or AimDK arm/hand execution
```

## 文档

| 文档 | 内容 |
| --- | --- |
| [安装与构建](docs/INSTALLATION.md) | ROS、AimDK、系统依赖、构建和验证 |
| [运行与调用](docs/USAGE.md) | 三种模式、Action、取消、真机流程 |
| [接口说明](docs/INTERFACES.md) | Action、消息、Topic 和字段语义 |
| [配置参考](docs/CONFIGURATION.md) | 视觉、IK、轨迹和音频参数 |
| [架构说明](docs/ARCHITECTURE.md) | 模块职责、线程模型和数据流 |
| [性能与 C++ 迁移](docs/PERFORMANCE.md) | 原生 IK 边界、左右臂逐项性能和测试条件 |
| [发布管理](docs/PUBLISHING.md) | 发布路径分类、机械臂 C++ 发布器建议和迁移门槛 |
| [远端 API 与内置 URDF](docs/REMOTE_API_AND_URDF.md) | 数据外发边界和简化模型适用范围 |
| [故障排查](docs/TROUBLESHOOTING.md) | 安装、TF、视觉、IK 和执行问题 |
| [包内说明](src/x2_grasp/README.md) | 功能包级详细说明 |

## 项目结构

```text
.
|-- example/                   # 可运行示例
|-- docs/                      # 完整操作和设计文档
|-- scripts/                   # 系统依赖安装脚本
`-- src/x2_grasp/
    |-- action/                # Grasp Action
    |-- msg/                   # 强类型消息
    |-- config/                # 统一配置
    |-- launch/                # 统一启动文件
    |-- x2_grasp/              # 感知和抓取编排
    |-- x2_arm/                # IK 后端选择、轨迹和硬件控制
    |-- x2_common/             # 公共 Python 工具
    `-- src/                   # C++ RGB-D 定位和 Pinocchio IK
```

## 测试

```bash
colcon test --packages-select x2_grasp --event-handlers console_direct+
colcon test-result --verbose
```

纯 Python 测试：

```bash
PYTHONPATH=src/x2_grasp python3 -m pytest src/x2_grasp/test
```

Pinocchio 热路径性能基准：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 scripts/benchmark_ik.py --backend python
/usr/bin/python3 scripts/benchmark_ik.py --backend native
```

基准输出包含求解器构造、关节/配置映射、全部 FK、位置/姿态/6D/轴向 IK、8 点链式规划和轨迹插值的平均值、中位数与 P95。
抓取节点的 `ik_backend` 参数支持 `auto`、`native` 和 `python`；默认 `auto` 优先使用已构建的 C++ 后端，扩展不可用时回退 Python。
完整迁移清单、左右臂逐项数据和保留 Python 的边界说明见 [性能与 C++ 迁移](docs/PERFORMANCE.md)。

完整测试需要 ROS 2、AimDK、Pinocchio、OpenCV 和目标机器人消息环境。提交问题或 Pull Request 时，请注明实际执行过的测试范围。

## 配置与凭据

统一配置位于 `src/x2_grasp/config/unified_grasp.yaml`。Grounding 模式优先从
`ARK_API_KEY` 环境变量读取密钥，也支持权限受限的 `~/.x2_arm/api_key.yaml`。不要把真实密钥写入仓库配置文件。

抓取类型也在该文件顶部统一定义。数组按下标对应，可以直接修改类型、识别描述、别名、夹爪闭合值和完成提示音：

```yaml
target_names: [cup, bread, bottle, apple]
target_descriptions: [一次性纸杯, 长条袋装面包, 长条药瓶, 红色苹果]
target_aliases: [paper_cup=cup, medicine_bottle=bottle, fruit=apple]
target_grip_close_positions: [0.10, 0.10, 0.10, 0.25]
target_pcm_paths: [cup.pcm, bread.pcm, bottle.pcm, ""]
default_pcm_path: grasp_complete.pcm
```

完整规则见 [配置参考](docs/CONFIGURATION.md)。

## 许可证

本项目采用 [Apache License 2.0](LICENSE)。第三方 ROS、AimDK、模型服务和机器人资源仍受各自许可证与服务条款约束。
