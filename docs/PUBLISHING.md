# 发布管理与 C++ 边界

## 结论

机械臂与夹爪的 50 Hz 连续命令流已经迁入独立 rclcpp 节点 `x2_command_publisher`。迁移覆盖
整段调度和安全边界，不是只包装 `publish()`：

- Python 一次提交完整的机械臂起止轨迹或夹爪持续目标；
- C++ 完成平滑插值和 AimDK 消息构造；
- 使用 `std::chrono::steady_clock` 和绝对 deadline，避免逐周期累计漂移；
- 同时只接受一个内部命令 Action，拒绝交错轨迹；
- cancel 和 watchdog 停止后续帧，并可重发最后命令作为 hold；
- Result 返回发布帧数、deadline miss、最大延迟和实际耗时。

其他事件驱动发布仍保留原职责：Grasp Action、Grounding、感知状态和音频由 Python 编排，
RGB-D 与定位高带宽链继续由现有 rclcpp 节点负责。C++ 发布减少解释器调度和 GIL 对控制节拍
的影响，但普通 Linux、ROS executor、DDS 和硬件总线仍不构成硬实时系统。

## 节点边界

```text
Python GraspExecutor
    -> clip start/goal through IK limits
    -> ExecuteCommand Action goal (one complete segment)
         |
         v
C++ x2_command_publisher
    -> validate command and reject concurrent streams
    -> smoothstep interpolation
    -> absolute monotonic deadlines at 50 Hz
    -> build AimDK arm/hand messages
    -> publish + collect timing metrics
    -> cancel/watchdog => stop and optional last-position hold
```

内部 Action 为 `/x2_grasp/execute_command`，类型为 `x2_grasp/action/ExecuteCommand`。它不是
公共抓取 API；外部业务仍应调用 `/x2_grasp/grasp`。

## 后端选择

抓取节点参数 `command_backend` 支持：

| 值 | 行为 |
| --- | --- |
| `auto` | 安装目录存在 C++ 节点时启动并连接；否则回退 Python 兼容发布器 |
| `native` | 必须存在并连接 C++ 节点，否则拒绝启动 |
| `python` | 强制使用原 Python 发布路径，用于兼容和对照 |

真机部署门禁应使用：

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag ik_backend:=native command_backend:=native execute:=false
```

`execute:=false` 不发送运动，但仍会检查原生节点是否已构建并可连接。

## 调度与停止

机械臂轨迹保持原有 14 关节 smoothstep 规则：时长决定最低帧数，最大关节步长
`max_delta_per_step` 可以进一步增加帧数。每帧 deadline 相对于该段统一起点计算，而不是在
上一帧后调用相对 `sleep(period)`。

夹爪命令按同一调度器持续刷新。内部 Action 收到 cancel 时设置 C++ 原子取消标志；调度器在
下一个 deadline 边界停止。`hold_on_stop=true` 时，取消或 watchdog 会再发布一次最后成功
命令。该 hold 行为必须与目标固件确认，不能替代控制器 stop 接口或硬件急停。

关键参数位于 `x2_command_publisher.ros__parameters`：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `publish_rate_hz` | `50.0` | 机械臂和夹爪发布频率 |
| `max_delta_per_step` | `0.03` | 单关节 smoothstep 最大步长约束 |
| `max_duration_seconds` | `30.0` | 单个内部命令段允许的最长时间 |
| `deadline_tolerance_ms` | `2.0` | 计为 deadline miss 的允许延迟 |
| `watchdog_ms` | `60.0` | 超过后中止当前命令流 |
| `hold_on_stop` | `true` | cancel/watchdog 后是否重发最后命令 |

## 条件构建

发布节点直接包含 AimDK C++ 消息头，因此只在 `find_package(aimdk_msgs)` 成功时构建。目标
机器人必须在构建前 source AimDK：

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc
ros2 pkg prefix aimdk_msgs
colcon build --packages-select x2_grasp --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
test -x install/x2_grasp/lib/x2_grasp/x2_command_publisher
```

通用 WSL 没有 `aimdk_msgs` 时，CMake 跳过 `x2_command_publisher`，但仍构建内部 Action、调度
核心和 gtest。不要复制其他 ROS 版本或 CPU 架构生成的 AimDK 头文件和动态库。

## 验收指标

内部 Action Result 已提供以下指标，真机验收还应结合 AimDK 控制器日志：

| 指标 | 含义 |
| --- | --- |
| `frames_requested/published` | 计划帧数和实际发送帧数 |
| `deadline_misses` | 超过允许延迟的帧数 |
| `max_lateness_ns` | 最差 deadline 延迟 |
| `elapsed_seconds` | 命令段实际耗时 |
| `watchdog_triggered` | 是否因严重调度延迟中止 |
| `hold_published` | 是否执行取消/watchdog 后 hold |

至少需要对比 Python/C++ 的 P50、P95、P99 和最大周期、累计时长误差、cancel 到最后命令的
延迟、CPU 占用，以及控制器丢包、watchdog 或拒绝数。只有真机指标和停止行为通过，才应把
`command_backend:=native` 写入生产启动配置。
