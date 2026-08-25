# x2_grasp

X2 双臂自动选择视觉抓取的单一 ROS 2 功能包。包内包含消息接口、公共工具以及以下业务模块，
不再需要部署其他工作空间功能包：

项目级快速开始、可运行 Demo 和专题文档分别位于仓库根目录的 `README.md`、`example/` 和
`docs/`。本文聚焦功能包内部行为与参数。

- 火山方舟视觉定位 API；
- RGB 与对齐深度图的 C++ 三维定位；
- AprilTag 36h11 定位；
- Pinocchio IK、机械臂/夹爪控制和完整抓取动作。

其中 Grounding 目标识别使用火山方舟远端 API，RGB 图像会发送到配置的 HTTPS 服务端；
AprilTag 检测和 RGB-D 三维定位在本地执行。包内同时内置一份面向 Pinocchio IK 的简化 X2
URDF，不应将其当作包含碰撞、控制器和高保真网格的完整官方机器人描述。详细边界见仓库
`docs/REMOTE_API_AND_URDF.md`。

## Grasp Action

所有抓取操作都通过 ROS 2 Action 提交：

```text
/x2_grasp/grasp  x2_grasp/action/Grasp
```

默认目标为 `cup`、`bread`、`bottle`，类型、描述和别名可通过统一配置修改。同一时间只接受
一个 goal；新的并发 goal 会被拒绝。反馈中的 `stage` 会报告视觉定位、IK 规划、
接近、闭爪、抬升和后撤等阶段，result 则返回最终成功状态和错误。客户端可以请求取消；
取消会停止继续发布轨迹，并在当前不可中断的夹爪/服务调用返回后结束 goal。

```bash
ros2 action send_goal /x2_grasp/grasp x2_grasp/action/Grasp \
  "{target: cup}" --feedback
```

检查服务、接口和当前 goal：

```bash
ros2 action list -t
ros2 action info /x2_grasp/grasp
ros2 interface show x2_grasp/action/Grasp
```

包内提供了完整 Action 客户端，会显示 feedback、状态码和 result：

```bash
ros2 run x2_grasp grasp_action_client cup
ros2 run x2_grasp grasp_action_client bottle --cancel-after 5
```

第二条命令会在 goal 被接受 5 秒后调用标准 `cancel_goal_async()`。运行期间按 `Ctrl+C` 时，
客户端也会先尝试取消活动 goal 再退出。实现位于 `x2_grasp/grasp_action_client.py`，可作为业务
客户端接入范例。

AprilTag 与 RGB-D 定位状态均使用 `x2_grasp/msg/PerceptionStatus`。AprilTag 状态发布在
`/x2_apriltag/status`，包含检测阶段、稳定帧数、目标坐标、扩散范围和重投影误差；不再
通过 `std_msgs/String` 拼接状态文本。

```bash
ros2 topic echo /x2_apriltag/status x2_grasp/msg/PerceptionStatus
ros2 topic echo /x2_rgbd_localizer/status x2_grasp/msg/PerceptionStatus
```

## 模式选择

另外保留 `auto` 仲裁入口：每次收到指令后先等待一小段时间检查是否有新鲜的 ARTag
坐标；有则走 ARTag，没有才把指令转发给 API 画框链路。

```bash
ros2 launch x2_grasp unified_grasp.launch.py mode:=auto execute:=false
```

提交目标的方式与模式无关：

```bash
ros2 action send_goal /x2_grasp/grasp x2_grasp/action/Grasp \
  "{target: bottle}" --feedback
```

### AprilTag 模式

```bash
ros2 launch x2_grasp unified_grasp.launch.py mode:=apriltag execute:=false
```

节点收到三种指令中的任意一种后，等待稳定的 AprilTag 坐标并解算抓取。此模式下指令中的
物品类型不参与视觉计算。确认 dry-run 的后撤抬高和抓取轴 IK 都成功后才能改为 `execute:=true`。

### API 画框模式

API Key 优先从环境变量 `ARK_API_KEY` 读取，也可创建私有文件
`~/.x2_arm/api_key.yaml`，内容为 `api_key: "..."`，然后启动：

```bash
export ARK_API_KEY='your-api-key'
ros2 launch x2_grasp unified_grasp.launch.py mode:=grounding execute:=false
```

收到指令后，抓取节点会把类型转发到内部话题 `/x2_grasp/grounding_target`，完整链路为：

```text
configured target name
  -> 对应中文目标提示词调用火山方舟 API
  -> 选择检测结果中的最大框
  -> 原 RGB 像素框 + 同时刻保留的对齐深度帧
  -> base_link 三维坐标
  -> 三段抓取 IK、固定姿态收手与抓取动作
```

API 节点按指令抓取原始 RGB 帧并在内存中编码为 JPEG。请求入队时，定位节点会单独保留
同一时间戳的对齐深度帧，避免普通 30 帧
缓存先于 API 响应过期。强类型 `GroundingResult` 发布在
`/x2_grasp/grounding_result`。

API 网络/响应错误、空检测框、无有效深度、RGB-D 不同步以及定位超时会在
机械臂动作开始前自动重新采集并重试，默认最多 3 次。可通过
`grounding_retry_attempts`、`grounding_retry_delay`、
`grounding_attempt_timeout` 和 `localization_result_timeout` 调整；深度定位状态
发布在 `/x2_rgbd_localizer/status`。只有取得有效三维坐标后才会进入 IK 和动作流程。

真实抓取动作完成后，节点会按目标配置播放 `audio/` 下对应的 PCM。播放使用 AimDK
`/aima/hal/audio/playback` 原始音频流接口，并在播放前申请、
播放后释放音频焦点。PCM 格式固定为单声道、16 kHz、16-bit S16LE。

## 构建

工作区的 `src/` 下只保留 `x2_grasp/`。依赖脚本通过 apt 安装
`ros-${ROS_DISTRO}-pinocchio`、`python3-numpy`、`python3-opencv` 和 `libopencv-dev`，默认 ROS
发行版为 Humble。

AimDK 是机器人平台环境，需要提前安装。完整安装和构建顺序如下：

```bash
cd ~/x2_grasp_ws
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc

./scripts/install_dependencies.sh
colcon build --packages-select x2_grasp --symlink-install
source install/setup.bash
```

脚本需要 `apt-get`，非 root 用户需要 `sudo`，可安全重复执行。Pinocchio 必须使用与当前
ROS 发行版匹配的 apt 包，不能改用 pip 版本。

验证依赖和生成接口：

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source install/setup.bash

/usr/bin/python3 -c \
  'import pinocchio; print(pinocchio.__version__, pinocchio.__file__)'
ros2 pkg prefix x2_grasp
ros2 interface show x2_grasp/action/Grasp
ros2 interface show x2_grasp/msg/PerceptionStatus
```

也可以根据 `scripts/robot_env.sh.example` 创建自己的环境加载脚本，避免每个终端重复输入。

运行前必须 source 上述环境，因为 Pinocchio（由 ROS 2 包提供）、eigenpy、rclpy 和消息类型
包含与 ARM64、Python 3.10、ROS Humble ABI 绑定的二进制扩展。OpenCV、NumPy 直接使用
机器人系统版本。Python 入口设置 `PYTHONNOUSERSITE=1`，不会使用 `~/.local` 中的 pip 包，
但会保留 source 注入的 ROS/AimDK 路径。

功能包不再依赖独立部署的 `volcengine_grounding`、`x2_rgbd_localizer`、`x2_arm` 或旧版
`x2_grasp`。ROS 2、AimDK、Pinocchio 和 Linux 系统基础库均由目标系统提供，不能跨 ROS
发行版或 CPU 架构复制使用。

火山方舟密钥不会随功能包安装。部署时应设置 `ARK_API_KEY`，或使用权限受限的
`~/.x2_arm/api_key.yaml`；不要把密钥写入受版本控制的统一配置文件。

## 动作配置

统一配置文件是 `config/unified_grasp.yaml`。原动作顺序保持不变：

```text
闭爪 -> 后撤抬高 -> 张爪 -> 预抓取点 -> 水平推进 -> 合爪 -> 抬高 -> 保持 Z 高度水平后撤
```

主要可调参数：

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `gripper_reach` | 末端 frame 到夹爪抓取中心 | `0.11 m` |
| `tag_depth` | AprilTag 平面到物体中心的 +X 偏移 | `0.03 m` |
| `standoff` | 预抓取水平后退距离 | `0.12 m` |
| `backward` | 起始让位姿态相对当前末端的后撤距离 | `0.03 m` |
| `initial_upward` | 起始让位姿态相对当前末端的抬高距离 | `0.15 m` |
| `grasp_x_offset` | 解算抓取点沿 +X 的前推修正 | `0.05 m` |
| `grasp_plane_z` | 预抓取和抓取阶段的固定末端高度 | `0.262 m` |
| `post_grasp_lift` | 抓住后、收手前的垂直抬高距离 | `0.06 m` |
| `min_post_grasp_lift` | 期望高度不可解时允许的最低抬高距离 | `0.04 m` |
| `lift_step` | 抓取后连续抬高的 IK 步长 | `0.02 m` |
| `cartesian_step` | 水平接近和高位后撤的笛卡尔 IK 步长 | `0.015 m` |
| `high_retract_position_tolerance` | 高位后撤严格 IK 不收敛时可接受的位置误差 | `0.005 m` |
| `high_retract_axis_tolerance` | 高位后撤严格 IK 不收敛时可接受的抓取轴误差 | `0.05 rad` |
| `grasp_axis_orientation_eps` | 抓取轴允许的方向误差，约 5 度；绕轴滚转不约束 | `0.087 rad` |
| `ik_seed_perturbation` | 主初值失败后肩、肘、腕多初值扰动量 | `0.12 rad` |
| `duration` | 普通轨迹段时长 | `4.0 s` |
| `approach_duration` | 水平推进时长 | `3.0 s` |
| `target_names` | 可用抓取类型名称 | `[cup, bread, bottle]` |
| `target_descriptions` | Grounding 识别描述 | 与类型按下标对应 |
| `target_aliases` | `alias=target` 格式别名 | 见统一配置 |
| `target_grip_close_positions` | 各类型夹爪闭合位置 | `[0.10, 0.10, 0.10]` |
| `target_pcm_paths` | 各类型优先提示音，空值使用缺省音 | 见统一配置 |
| `default_pcm_path` | 类型音频不可用时的通用补救音 | `grasp_complete.pcm` |
| `initial_close_seconds` | 起始闭爪持续时间 | `2.0 s` |
| `open_seconds` | 张爪持续时间 | `0.5 s` |
| `grip_close_seconds` | 抓取闭爪持续时间 | `2.0 s` |

五个目标配置数组按下标对应，夹爪位置为 `0.0`（完全闭合）到 `1.0`（完全张开）。即使使用
AprilTag 模式，坐标定位不区分物品类型，最终闭合值和提示音仍按输入类型选择。完整扩展示例
见仓库根目录 `docs/CONFIGURATION.md`。

AprilTag 尺寸 `tag_size_m` 和以上几何参数必须按实物标定。真机前保持 `execute:=false`，并确认
目标坐标、自动选臂结果、对应可达域和抓取轴 IK 日志全部正确。IK 严格控制位置及夹爪抓取轴方向，允许
绕抓取轴自由滚转，并在主初值失败后自动尝试肩、肘、腕扰动初值。抓取后以 `lift_step`
连续抬高；期望高度不可解但已达到 `min_post_grasp_lift` 时会使用已成功高度。起始让位姿态
根据当前末端位置做后撤抬高解算；抓取完成后保持抬高后的 Z 不变，沿 -X 回到预抓取 X。
高位后撤允许使用单独配置的近似 IK；若中途不可解则保留已完成距离，第一段不可解时停在
抬高点，不会因此取消前面的抓取和抬高。

## 推荐操作流程

1. 修改 `config/unified_grasp.yaml` 中的相机话题、AprilTag 尺寸和抓取几何参数。
2. 加载 ROS 2、AimDK 和当前工作区的 `setup.bash`。
3. 使用 `execute:=false` 启动所需模式，发送三种目标各做一次 IK dry-run。
4. 观察 Action feedback、目标坐标和 `PerceptionStatus`，确认坐标系为 `base_link`。
5. 清空机械臂运动范围，确认急停可用，再使用 `execute:=true` 启动。
6. 真机先使用单个目标测试；不要并行发送 goal，服务会拒绝第二个活动 goal。

真机 Grounding 模式示例：

```bash
export ARK_API_KEY='your-api-key'
ros2 launch x2_grasp unified_grasp.launch.py mode:=grounding execute:=true
```

另一个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc
source ~/x2_grasp_ws/install/setup.bash

ros2 action send_goal /x2_grasp/grasp x2_grasp/action/Grasp \
  "{target: bread}" --feedback
```

## 常见问题

- `No module named pinocchio`：确认安装了 `ros-humble-pinocchio`，并在启动前 source
  `/opt/ros/humble/setup.bash`；不要用 `pip install pin` 替代。
- 找不到 `x2_grasp/action/Grasp`：重新构建后 source 当前工作区的 `install/setup.bash`。
- Action goal 被拒绝：目标名称无效，或已有一个 goal 正在运行。
- AprilTag 一直累计失败：检查 `tag_id`、`tag_size_m`、重投影误差、稳定帧数和相机 TF。
- Grounding 无结果：检查 `ARK_API_KEY`、网络、RGB-D 时间同步以及定位状态的 `error` 字段。
- dry-run 成功但真机不动作：确认使用 `execute:=true`，并检查 AimDK 服务和上半身控制模式。
