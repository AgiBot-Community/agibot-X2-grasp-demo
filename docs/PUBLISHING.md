# 发布管理与 C++ 边界

## 结论

不是所有 ROS 发布都更适合 C++。本仓库应按数据频率和职责划分：

| 发布路径 | 当前实现 | 建议 |
| --- | --- | --- |
| RGB-D 对齐图、相机点、目标向量和定位状态 | rclcpp | 保持 C++ |
| 机械臂 `UpperBodyCommandArray` 50 Hz 命令流 | rclpy + wall-clock sleep | 生产版建议整体迁入 C++ |
| 夹爪 50 Hz 命令流 | rclpy worker | 与机械臂调度器一起评估 C++ |
| Grasp Action feedback/result/cancel | rclpy | 保持 Python 编排 |
| AprilTag、Grounding 和工作流状态 | rclpy | 事件驱动，保持 Python |
| 音频块与焦点服务 | Python worker + rclpy | 保持 Python，除非实测 underrun |
| 通用 IK 解和诊断 | rclpy + C++ IK 内核 | 发布频率低，保持现状 |

当前 Python 机械臂发布器适合 Demo、dry-run 后的真机验证和 50 Hz 基本控制，但不应描述为
实时控制器。C++能减少解释器调度和 GIL 影响，却不能自动提供硬实时保证；Linux 调度策略、
ROS 2 executor、DDS、控制器缓存和硬件总线仍会影响周期。

## 当前机械臂命令链

```text
Python GraspExecutor
    -> clip start/goal through native IK limits
    -> generate smoothstep waypoints
    -> for each waypoint
         build AimDK UpperBodyCommandArray
         publish
         spin_once(0)
         wall-clock sleep to next 20 ms period
```

取消在每个轨迹点之前检查。已经发布的命令不会撤回，取消也不是急停。当前实现没有独立的
deadline miss 统计、控制器 watchdog、取消后的显式 hold 命令或发布线程优先级。

## 推荐的 C++ 发布器边界

迁移时应建立独立 rclcpp `TrajectoryCommandPublisher`，一次接收已经验证的完整 14 关节轨迹，
而不是让 Python 每个 waypoint 调一次 C++。建议职责如下：

1. 验证 14 关节顺序、有限值、时间单调性和轨迹长度上限。
2. 消费已经应用安全限位的 waypoint，不在发布线程中运行 IK。
3. 使用单调时钟和绝对 deadline 调度，避免累计 `sleep(period)` 漂移。
4. 构造并发布 AimDK `UpperBodyCommandArray`，管理 sequence、stamp、source 和 frame。
5. 使用容量为 1 的有界命令队列，新轨迹不能与旧轨迹交错。
6. 支持 cancel、shutdown 和超时后的显式 hold/stop 策略。
7. 记录周期、deadline miss、最大抖动、取消到停止延迟和发布失败。
8. 将执行结果以 Action、service 或强类型状态返回 Python 编排层。

夹爪若要求持续 50 Hz 刷新，可复用同一调度组件；如果 AimDK 控制器接受单次带持续时间的
夹爪目标，则优先使用控制器契约，不应在上层重复流式发布。

## 为什么不只迁移 `publish()`

单次 rclpy publish 的计算量不是当前主要瓶颈。仅通过 pybind11 包装发布调用会增加 Python/C++
消息转换，同时仍保留 Python 循环、`time.sleep`、取消检查和序列管理，无法改善核心时序问题。
有价值的迁移单位是完整的周期调度与安全状态机。

## 构建前置条件

C++ 发布器需要 AimDK 提供可供 ament/CMake 查找的 `aimdk_msgs` C++ typesupport 和准确的
控制器契约。当前通用 WSL ROS 2 Humble 环境执行：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix aimdk_msgs
```

结果为 `Package not found`。因此本仓库当前只能在 Python 运行时延迟导入 AimDK 消息，不能在
该 WSL 环境编译或验证 rclcpp AimDK 发布器。不要为绕过这个缺口复制未知 ABI 的生成头文件。

开始迁移前必须在目标机器人环境确认：

- `ros2 pkg prefix aimdk_msgs` 成功；
- `UpperBodyCommandArray` 和 hand command 存在 C++ typesupport；
- 控制器要求的 Topic、QoS、频率、时间戳和序列语义；
- cancel 时应发送 hold、stop，还是停止刷新；
- 控制器 watchdog 超时及允许的最大 deadline miss。

## 验收指标

C++ 发布器完成的证据不能只看“能够编译”。至少应在相同轨迹和系统负载下对比 Python/C++：

| 指标 | 采集方式 |
| --- | --- |
| 实际发布周期中位数、P95、P99、最大值 | 发布前后单调时钟打点 |
| 20 ms deadline miss 数量和比例 | 每个 waypoint 对绝对 deadline |
| 累计轨迹完成时间误差 | 首末 waypoint 时间差 |
| cancel 到最后命令/hold 的延迟 | cancel stamp 与发布 stamp |
| CPU 占用和线程调度延迟 | 同负载下进程和线程统计 |
| 控制器丢包、watchdog 或拒绝数 | AimDK 控制器日志/状态 |

只有这些指标在目标硬件上优于 Python，并且取消、限位和故障行为一致，才应切换生产默认值。
