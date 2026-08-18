# X2 Grasp Examples

本目录包含最小可运行 Demo。所有示例默认使用 dry-run，除非明确把 launch 参数改成
`execute:=true`。

## 准备环境

每个终端都执行：

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc
source ~/x2_grasp_ws/install/setup.bash
```

## Demo 1：AprilTag dry-run

终端 A 启动节点：

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag execute:=false
```

将配置中 `tag_id` 对应的 AprilTag 放入相机视野。终端 B 观察强类型状态：

```bash
ros2 topic echo /x2_apriltag/status x2_grasp/msg/PerceptionStatus
```

终端 C 提交抓取目标：

```bash
python3 example/send_grasp_goal.py cup
```

Action 应依次报告等待 AprilTag、读取关节状态、IK 规划和 dry-run 完成。AprilTag 模式下，
物品名称用于选择夹爪闭合参数，不参与标签坐标计算。

## Demo 2：Grounding dry-run

终端 A：

```bash
export ARK_API_KEY='your-api-key'
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=grounding execute:=false
```

终端 B：

```bash
python3 example/send_grasp_goal.py bottle
```

监控 Grounding 和 RGB-D 定位：

```bash
ros2 topic echo /x2_grasp/grounding_result x2_grasp/msg/GroundingResult
ros2 topic echo /x2_rgbd_localizer/status x2_grasp/msg/PerceptionStatus
```

## Demo 3：自动仲裁

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=auto execute:=false
```

```bash
python3 example/send_grasp_goal.py bread
```

节点先检查新鲜的 AprilTag 目标；没有可用标签时，自动进入 Grounding 和 RGB-D 定位链路。

## Demo 4：取消 Action

在目标接受 5 秒后自动请求取消：

```bash
python3 example/send_grasp_goal.py cup --cancel-after 5
```

也可以在客户端运行时按 `Ctrl+C`。示例会先调用 `cancel_goal_async()`，等待最多 2 秒后退出。
取消只会阻止后续轨迹点；已经发出的硬件命令和正在进行的不可中断服务调用不会回滚。

## Demo 5：真机执行

只有所有 dry-run 均通过后才能启用：

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag execute:=true
```

```bash
python3 example/send_grasp_goal.py cup
```

真机前必须确认：

- 急停可用，机械臂工作空间无人且无障碍物
- `/x2_apriltag/target_vector` 或 RGB-D 目标位于 `base_link`
- 目标坐标单位为米，数值位于右臂可达范围
- dry-run 的预抓取、推进、抬升和后撤 IK 全部成功
- 夹爪闭合参数已经按实物标定

更多说明见 [运行与调用](../docs/USAGE.md) 和
[配置参考](../docs/CONFIGURATION.md)。
