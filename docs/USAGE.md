# 运行与调用

## 启动参数

统一入口：

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=<apriltag|grounding|auto> \
  ik_backend:=<auto|native|python> \
  command_backend:=<auto|native|python> execute:=<false|true>
```

`execute:=false` 会完成感知、目标校验和 IK 规划，但不切换机器人模式，也不发送运动轨迹。
`execute:=true` 会控制真实机械臂和夹爪。

`ik_backend:=auto` 是默认值，扩展存在时使用 C++ Pinocchio 后端，否则回退 Python。
`ik_backend:=native` 适合部署检查，会在扩展缺失时直接失败；`python` 只建议用于一致性对照和
故障隔离。启动后可先检查：

```bash
/usr/bin/python3 -c \
  'from x2_arm import native_backend_available; print(native_backend_available())'
```

`command_backend:=auto` 在安装目录存在真机构建的 `x2_command_publisher` 时启动并使用完整
C++ 发布节点，否则回退 Python。真机验收使用 `command_backend:=native`，节点缺失或内部
Action 不可用时会拒绝执行，不会静默回退。

## AprilTag 模式

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag execute:=false
```

AprilTag 节点检测 36h11 标签，经 solvePnP 和 TF 转换得到 `base_link` 坐标。只有稳定帧数、
空间离散度和重投影误差同时满足配置条件时，才发布目标。

```bash
ros2 topic echo /x2_apriltag/status x2_grasp/msg/PerceptionStatus
ros2 topic echo /x2_apriltag/target_vector geometry_msgs/msg/Vector3Stamped
```

## Grounding 模式

```bash
export ARK_API_KEY='your-api-key'
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=grounding execute:=false
```

流程为 RGB 图像采集、视觉 API 目标框、同时间戳深度帧、三维定位和 TF 转换。API Key 也可
放在权限受限的 `~/.x2_arm/api_key.yaml`：

Grounding 使用火山方舟远端 API。RGB 图像会编码为 JPEG 并通过 HTTPS 发送到 `base_url`
配置的服务端，目标检测模型不在机器人本地运行。远端只返回二维框；深度定位、TF、IK 和
运动控制仍在本地完成。涉及隐私或禁止图像外发的部署应使用 `mode:=apriltag`。`auto` 模式在
找不到 AprilTag 时仍会调用远端 API。

```yaml
api_key: "your-api-key"
```

## 自动仲裁模式

```bash
export ARK_API_KEY='your-api-key'
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=auto execute:=false
```

每个 goal 先短暂检查新鲜 AprilTag 坐标。检测不到时，才启动 Grounding 请求。这个选择只在
goal 开始时进行，不会在机械臂执行中切换感知来源。

## Action 调用

```bash
ros2 action send_goal /x2_grasp/grasp x2_grasp/action/Grasp \
  "{target: cup}" --feedback
```

默认标准目标为：

| 目标 | 说明 |
| --- | --- |
| `cup` | 纸杯 |
| `bread` | 面包 |
| `bottle` | 药瓶或瓶装物体 |

目标名称、Grounding 描述、别名、夹爪闭合值和音频文件均由统一配置的 `target_*` 数组
定义。客户端应使用配置中的标准名称。已有 goal 执行时，第二个 goal 会被拒绝。

## Python Demo

```bash
python3 example/send_grasp_goal.py cup
python3 example/send_grasp_goal.py bottle --cancel-after 5
```

安装后的等价客户端：

```bash
ros2 run x2_grasp grasp_action_client bread
```

## 取消语义

客户端通过 `cancel_goal_async()` 请求取消。服务端会在等待感知、重试、IK 和模式切换阶段
检查请求；执行阶段会取消内部 `ExecuteCommand` Action。C++ 发布节点停止后续帧，并在
`hold_on_stop=true` 时重发最后位置作为 hold。取消不能撤销此前的命令，也不等同硬件急停。

## 真机执行清单

1. 在目标机器人上完成依赖安装、构建和完整测试。
2. 确认 `native_backend_available()` 输出 `True`，并检查 `x2_command_publisher` 已安装。
3. 用 `execute:=false` 分别验证所有需要使用的目标和感知模式。
4. 检查目标 frame 为 `base_link`、单位为米、坐标在所选机械臂可达域内；默认 `arm_side:=auto`。
5. 检查 AprilTag 尺寸、相机内参、深度尺度和 TF。
6. 清空机械臂工作空间，确认急停、夹爪和 AimDK 控制模式正常。
7. 使用 `ik_backend:=native command_backend:=native execute:=true` 启动，先执行单个低风险目标。

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag ik_backend:=native command_backend:=native execute:=true
```

默认 `arm_side:=auto`。标定或排查单侧机械臂时，可在启动命令中临时指定
`arm_side:=left` 或 `arm_side:=right`。

## 运行观察

```bash
ros2 action info /x2_grasp/grasp
ros2 node list
ros2 action info /x2_grasp/execute_command
ros2 topic hz /x2_apriltag/status
ros2 topic echo /x2_rgbd_localizer/status x2_grasp/msg/PerceptionStatus
```
