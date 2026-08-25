# 故障排查

## 找不到 Pinocchio

```bash
source /opt/ros/humble/setup.bash
dpkg -s ros-humble-pinocchio
/usr/bin/python3 -c 'import pinocchio; print(pinocchio.__file__)'
```

不要使用 `pip install pin`。确认启动入口使用 `/usr/bin/python3`，且没有被 Conda 或用户目录
Python 包覆盖。

## 原生 IK 扩展不可用

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -c \
  'from x2_arm import native_backend_available; print(native_backend_available())'
```

输出 `False` 时检查 `ros-humble-pybind11-vendor`、`pybind11-dev`，并使用 Release 重新构建。
不要从其他 Python 版本、ROS 发行版或 CPU 架构复制 `_x2_ik_native*.so`。可临时使用
`ik_backend:=python` 隔离扩展问题，但这不是原生性能问题的修复。

```bash
colcon build --packages-select x2_grasp --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag ik_backend:=native execute:=false
```

若 CMake 已识别 Python 3.10，但仍报告 `Could NOT find PythonInterp (missing:
PYTHON_EXECUTABLE)`，应清理 CMake 缓存并显式使用 ROS Humble 的系统 Python：

```bash
colcon build --packages-select x2_grasp --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
```

不要把 Conda 或其他 Python 环境的解释器传给 ROS 接口生成和 pybind11 构建。

## 找不到 x2_grasp 接口

```bash
cd ~/x2_grasp_ws
colcon build --packages-select x2_grasp --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 interface show x2_grasp/action/Grasp
```

新建 Action 或消息后必须重新构建。

## 找不到 AimDK 消息或服务

```bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc
ros2 pkg prefix aimdk_msgs
```

AimDK 必须在当前工作区之前 source。

## Action goal 被拒绝

- 目标不是 `cup`、`bread` 或 `bottle`
- 已有一个活动 goal
- ActionServer 正在关闭

```bash
ros2 action info /x2_grasp/grasp
```

## AprilTag 无法稳定

观察：

```bash
ros2 topic echo /x2_apriltag/status x2_grasp/msg/PerceptionStatus
```

检查 `detected_ids`、`sample_count`、`spread_m`、`reprojection_error_px` 和 `error`。常见原因：

- `tag_id` 与实物不一致
- `tag_size_m` 测量错误
- 标签像素尺寸小于 `min_side_pixels`
- 相机模糊、曝光不足或标签反光
- 相机到 `base_link` 的 TF 缺失

## Grounding 无结果

```bash
test -n "${ARK_API_KEY:-}" && echo configured
ros2 topic echo /x2_grasp/grounding_result x2_grasp/msg/GroundingResult
ros2 topic echo /x2_rgbd_localizer/status x2_grasp/msg/PerceptionStatus
```

检查 HTTPS 网络、模型 ID、API Key、请求超时、RGB-D 同步和深度有效范围。不要在日志或问题
报告中粘贴真实密钥。

## RGB-D 坐标异常

- 确认深度单位和 `depth_scale`
- 确认 `alignment_mode` 与相机实际输出一致
- 确认 RGB、Depth、CameraInfo 分辨率和 frame 匹配
- 确认输出 `frame_id` 为 `base_link`
- 使用 `tf2_echo` 检查相机光学 frame 到 `base_link`

```bash
ros2 run tf2_ros tf2_echo base_link <camera_optical_frame>
```

## IK 失败

先保持 `execute:=false`。检查目标是否在右臂可达域、URDF 是否与真机一致、当前关节状态是否
有效，以及 `gripper_reach`、`standoff`、`grasp_plane_z` 等几何参数。不要通过放宽所有容差来
掩盖坐标系或标定错误。

## dry-run 成功但真机不动作

- launch 是否使用 `execute:=true`
- AimDK 上半身控制模式切换服务是否可用
- 关节和夹爪 Topic 是否与机器人固件一致
- 是否设置了 `skip_mode_switch`
- 是否有取消请求或上一 goal 尚未结束

真机应能看到 `/x2_command_publisher` 节点和 `/x2_grasp/execute_command` Action。若
`command_backend:=native` 报节点未构建，先确认构建前已经 source AimDK，且
`ros2 pkg prefix aimdk_msgs` 成功。若出现 deadline miss 或 watchdog，检查 C++ 节点日志、系统
负载、Topic QoS 和 AimDK 控制器日志；不要通过缩短 `duration` 掩盖发布问题。指标和阈值见
[发布管理](PUBLISHING.md)。

## 取消后机器人仍有短暂动作

取消是 ROS Action 的协作式取消。C++ 发布节点会停止后续帧并按配置重发最后位置 hold，但已
发送的轨迹点不会撤回，同步服务调用也不能立刻中断。紧急情况必须使用硬件急停。
