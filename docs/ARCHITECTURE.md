# 架构说明

## 包边界

仓库只包含一个 ROS 2 包 `x2_grasp`，但在包内按职责拆分：

| 模块 | 职责 |
| --- | --- |
| `x2_grasp/` | Action、感知协调、规划编排和音频 |
| `x2_arm/` | Pinocchio IK、轨迹、机械臂和夹爪接口 |
| `x2_common/` | 重试、并发、凭据和 PCM 公共工具 |
| `src/localizer.cpp` | RGB-D 对齐、深度取样和 TF 定位 |
| `msg/`、`action/` | ROS 2 强类型契约 |

## Grasp Action 数据流

```text
Action client
    |
    v
GraspActionBridge ---- feedback/result/cancel
    |
    v
grasp_node orchestrator
    |
    +--> PerceptionCoordinator
    |       +--> AprilTag target
    |       `--> Grounding + RGB-D target
    |
    +--> GraspPlanner --> Pinocchio IK and staged plan
    |
    `--> GraspExecutor --> AimDK arm and hand commands
```

## Action 线程模型

ActionServer 位于独立 ROS 节点和 `MultiThreadedExecutor` 中。主抓取循环继续负责硬件节点和
感知订阅。两个执行上下文通过容量为 1 的工作队列传递 goal：

- Action 回调负责验证目标、拒绝并发 goal、发布 feedback 和完成 result
- 主循环按顺序执行感知、IK 和硬件操作
- `threading.Event` 传递完成和取消状态
- 同一时刻最多保留一个活动 goal，避免多条机械臂轨迹交错

关闭时先通知活动工作项结束，再停止 executor，最后销毁 ActionServer 和节点，确保执行回调
完成期间 goal handle 仍然有效。

## 感知数据一致性

Grounding 请求携带 request ID 和图像时间戳。RGB-D localizer 在请求入队时保留对应深度帧，
避免网络 API 返回前普通相机缓存过期。最终只接受时间戳和 request ID 匹配的结果。

Grounding 模型运行在火山方舟远端服务。只有 JPEG RGB 图像、目标描述和请求元数据会发送到
远端；深度图、关节状态、URDF、IK 结果和硬件控制命令不会发送给识别 API。

AprilTag 使用原始图像像素和相机内参执行 solvePnP，然后通过 TF 转换到 `base_link`。稳定窗口
同时约束连续帧数、坐标离散度和重投影误差。

## 规划和执行边界

`GraspPlanner` 只负责数值验证与轨迹规划，不发布硬件命令。`GraspExecutor` 只消费已经生成的
计划并控制机械臂/夹爪。`grasp_node` 负责工作流顺序和错误到 Action result 的映射。

dry-run 和真机执行共享同一个感知与规划路径，差异只出现在硬件执行边界。这可避免调试路径与
真实执行路径产生不同规划结果。

## 内置运动学模型

`urdf/x2_ultra_plus_omnipicker_omnipicker.urdf` 是随包安装的简化 Demo URDF，供 Pinocchio
构建 IK 模型。它主要服务于关节拓扑、机械限位和末端 frame 计算，不等同于完整官方机器人
description，也不提供环境碰撞、自碰撞、ros2_control 或高保真仿真能力。

## 取消和失败

取消是协作式的，不是硬件急停。感知等待、网络重试、IK 阶段和轨迹点之间会检查取消事件。
同步服务调用必须返回后才能响应取消。安全急停仍由机器人硬件和 AimDK 控制系统负责。
