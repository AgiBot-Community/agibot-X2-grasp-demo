# 配置参考

统一配置文件为 `src/x2_grasp/config/unified_grasp.yaml`。安装后的文件位于功能包 share 目录。
修改源码配置后需要重新构建，或在开发阶段使用 `--symlink-install`。

## 抓取类型目录

抓取类型在文件顶部的 `/**.ros__parameters` 中定义，并同时应用到 Grounding 和抓取节点：

```yaml
/**:
  ros__parameters:
    target_names: [cup, bread, bottle]
    target_descriptions: [一次性纸杯, 长条袋装面包, 长条药瓶]
    target_aliases: [paper_cup=cup, medicine_bottle=bottle]
    target_grip_close_positions: [0.10, 0.10, 0.10]
    target_pcm_paths: [cup.pcm, bread.pcm, bottle.pcm]
    default_pcm_path: grasp_complete.pcm
```

五个数组按下标一一对应。`target_aliases` 使用 `别名=标准名称`，可以为空数组。名称只能使用
小写字母、数字、下划线和连字符，并且必须以字母开头。

例如增加苹果：

```yaml
target_names: [cup, bread, bottle, apple]
target_descriptions: [一次性纸杯, 长条袋装面包, 长条药瓶, 红色苹果]
target_aliases: [paper_cup=cup, medicine_bottle=bottle, fruit=apple]
target_grip_close_positions: [0.10, 0.10, 0.10, 0.25]
target_pcm_paths: [cup.pcm, bread.pcm, bottle.pcm, ""]
default_pcm_path: grasp_complete.pcm
```

`target_pcm_paths` 优先指定每种类型自己的提示音。数组项为空、文件不存在或无法读取时，会
自动播放 `default_pcm_path` 指定的通用补救提示音。需要单独提示音时，将新的 16 kHz、
单声道、S16LE PCM 放入 `src/x2_grasp/audio/` 并修改对应数组项。配置长度不一致、别名指向
不存在的类型、重复名称、空描述或夹爪值超出 `[0, 1]` 时，节点会拒绝启动。

通用提示音由可复现脚本生成：

```bash
python3 scripts/generate_completion_tone.py
```

补救音频为 1.6 秒 C 大调上行铃音和弦，不包含语音，格式固定为 16 kHz、单声道、S16LE
原始 PCM。

## RGB-D 定位

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `alignment_mode` | `aligned` | 深度对齐模式 |
| `rgb_coordinate_mode` | `rectified` | RGB 坐标解释方式 |
| `sync_slop_sec` | `0.04` | RGB 与 Depth 同步容差 |
| `held_frame_queue_size` | `8` | 等待 API 时保留的深度帧数量 |
| `detection_slop_sec` | `2.0` | 检测框与图像时间容差 |
| `min_depth_m` | `0.05` | 最小有效深度 |
| `max_depth_m` | `20.0` | 最大有效深度 |
| `target_frame` | `base_link` | 输出目标坐标系 |

如果相机已经提供对齐深度，保持 `alignment_mode: aligned`。只有经过标定并填写有效外参后，
才能使用软件对齐模式。

## AprilTag

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `tag_id` | `0` | 目标标签 ID |
| `tag_size_m` | `0.080` | 标签实际边长，单位米 |
| `stable_frames` | `8` | 稳定发布需要的连续帧数 |
| `max_spread_m` | `0.010` | 稳定窗口最大离散度 |
| `max_reproj_error_px` | `3.0` | 最大重投影误差 |
| `min_side_pixels` | `40.0` | 标签最小图像边长 |
| `publish_mode` | `continuous` | `once` 或 `continuous` |

`tag_size_m` 必须测量打印后标签黑色外边界的实际尺寸。尺寸错误会按比例影响深度估计。

## Grounding

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model` | 配置文件值 | 火山方舟模型 ID |
| `base_url` | 北京区域 API | HTTPS API 地址 |
| `api_key_env` | `ARK_API_KEY` | 密钥环境变量 |
| `request_timeout_seconds` | `30.0` | 单次网络请求超时 |
| `bbox_selection` | `largest` | 检测框选择方式 |

不要在统一配置中填写真实 `api_key`。优先使用环境变量。

## 感知重试

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `grounding_retry_attempts` | `3` | 最大尝试次数 |
| `grounding_retry_delay` | `1.0` | 重试间隔 |
| `grounding_attempt_timeout` | `35.0` | 单轮 Grounding 超时 |
| `localization_result_timeout` | `5.0` | 等待三维定位超时 |
| `timeout` | `300.0` | AprilTag 等待总超时 |

## IK 与抓取几何

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `arm_side` | `auto` | `auto` 分别验证左右臂规划并选择关节运动量较小的一侧；也可固定为 `left` 或 `right` |
| `ik_backend` | `auto` | `auto` 优先原生 C++ 并在扩展缺失时回退；`native` 禁止回退；`python` 用于对照和排查 |
| `gripper_reach` | `0.11` | 末端 frame 到抓取中心距离 |
| `tag_depth` | `0.03` | 标签平面到物体中心的 X 偏移 |
| `standoff` | `0.12` | 预抓取水平距离 |
| `grasp_x_offset` | `0.05` | 抓取点 X 修正 |
| `grasp_plane_z` | `0.262` | 预抓取和抓取高度 |
| `post_grasp_lift` | `0.06` | 抓取后抬升距离 |
| `min_post_grasp_lift` | `0.04` | 允许的最低抬升距离 |
| `cartesian_step` | `0.015` | 笛卡尔路径离散步长 |
| `grasp_axis_orientation_eps` | `0.087` | 抓取轴方向容差，弧度 |

自动选臂仍受安全可达域约束：右臂用于机器人右侧（负 Y），左臂用于机器人左侧（正 Y）。
若首选侧完整分段 IK 不可解，会尝试另一侧；两侧均不可解时 Action 返回各自失败原因。这些参数与
机器人末端 frame、夹爪结构和桌面高度直接相关，不能照搬到不同硬件。

统一 launch 可直接覆盖后端：

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag ik_backend:=native execute:=false
```

生产部署建议先用 `native` 做启动检查，确认扩展和 ABI 正常后再使用默认 `auto`。性能回归时
必须分别显式指定 `python` 和 `native`，不要用 `auto` 代替对照组。

## 轨迹和夹爪

`duration`、`approach_duration` 控制轨迹段时长。三个
`*_grip_close_position` 参数范围为 `0.0` 到 `1.0`，其中 `0.0` 表示完全闭合，`1.0` 表示完全
张开。应根据物体硬度和尺寸逐个标定。

机械臂轨迹和夹爪命令当前按 50 Hz 发布。这个频率由硬件适配层固定，不是抓取 YAML 参数；
修改前必须核对 AimDK 控制器期望频率、QoS 和超时策略。发布架构取舍见
[发布管理](PUBLISHING.md)。

## Action 和执行

`action_name` 默认 `/x2_grasp/grasp`。`execute` 默认 `false`。`skip_mode_switch` 只应用于已经由
外部系统完成控制模式管理的部署，不应为了绕过模式切换错误而启用。
