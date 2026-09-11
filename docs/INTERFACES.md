# ROS 2 接口

[文档导航](README.md) · [简体中文](../README.md) · [English](README.en.md) · [Français](README.fr.md)

## Grasp Action

名称默认为 `/x2_grasp/grasp`，类型为 `x2_grasp/action/Grasp`。

```text
# Goal
string target
---
# Result
bool success
string target
string error
---
# Feedback
string stage
string detail
```

`target` 使用规范化目标名称。`success=false` 时，`error` 提供可读错误。取消 goal 的最终 ROS
状态为 `CANCELED`；规划或执行失败为 `ABORTED`。

常见 feedback 阶段包括感知等待、读取关节状态、IK 规划、模式切换、轨迹执行、音频播报和
dry-run 完成。客户端不应依赖固定的阶段数量。

## PerceptionStatus

AprilTag 状态话题 `/x2_apriltag/status` 和 RGB-D 状态话题
`/x2_rgbd_localizer/status` 均使用 `x2_grasp/msg/PerceptionStatus`。

| 字段 | 含义 |
| --- | --- |
| `image_stamp` | 对应 RGB 图像时间戳 |
| `success` | 当前状态是否代表有效定位结果 |
| `source` | `apriltag` 或 `rgbd` |
| `stage` | 当前检测或定位阶段 |
| `detail` | 非错误补充信息 |
| `error` | 失败原因，成功时为空 |
| `detected_ids` | 当前检测到的 AprilTag ID |
| `sample_count` | 已累计稳定样本数 |
| `required_samples` | 需要的稳定样本数 |
| `spread_m` | 稳定窗口最大空间离散度，单位米 |
| `reprojection_error_px` | AprilTag 重投影误差，单位像素 |
| `target` | 当前目标三维坐标 |
| `frame_id` | 目标坐标系，正常为 `base_link` |

## GroundingCommand

内部命令话题默认为 `/x2_grasp/grounding_target`。消息携带创建时间、请求 ID 和规范化目标。
请求 ID 用于将 API 结果、深度定位和当前 Action goal 关联起来。

## GroundingResult

话题默认为 `/x2_grasp/grounding_result`，包含图像时间戳、请求 ID、成功状态、框数量、延迟和
错误。它表示视觉 API 请求结果，不等价于最终三维定位或抓取结果。

## 目标向量

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/x2_apriltag/target_vector` | `geometry_msgs/Vector3Stamped` | AprilTag 稳定目标 |
| `/x2_rgbd_localizer/target_vector` | `geometry_msgs/Vector3Stamped` | Grounding 三维目标 |

两个目标均必须使用米，并转换到 `base_link`。抓取编排器会拒绝空 frame、非有限数值和明显
不合理的目标。

## IK 解与硬件命令

`x2_ik_solver_node` 只发布 IK 解和诊断，不驱动硬件：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/x2_ik/left/solution` | `sensor_msgs/msg/JointState` | 左臂目标对应的完整 14 关节解 |
| `/x2_ik/right/solution` | `sensor_msgs/msg/JointState` | 右臂目标对应的完整 14 关节解 |
| `/x2_ik/status` | `diagnostic_msgs/msg/DiagnosticArray` | 求解成功、误差、迭代数和末端 frame |

Python 编排器通过内部 `/x2_grasp/execute_command` Action 将整段命令交给
`x2_command_publisher`。类型 `x2_grasp/action/ExecuteCommand` 支持机械臂起止 14 关节轨迹和
左/右/双夹爪持续命令；结果返回发布帧数、deadline miss、最大延迟、总耗时、watchdog 和
hold 状态。该 Action 是包内执行契约，不应替代公开的 Grasp Action。

C++ 节点向 `/mc/upper_body_command` 发布 AimDK `UpperBodyCommandArray`，并向
`/aima/hal/joint/hand/command` 发布 AimDK hand command。AimDK 类型是平台运行时契约，不是
本包生成的消息；部署前必须核对当前固件中的字段、QoS 和控制器超时。详见
[发布管理](PUBLISHING.md)。

## 接口检查

```bash
ros2 interface show x2_grasp/action/Grasp
ros2 interface show x2_grasp/action/ExecuteCommand
ros2 interface show x2_grasp/msg/GroundingCommand
ros2 interface show x2_grasp/msg/GroundingResult
ros2 interface show x2_grasp/msg/PerceptionStatus
```
