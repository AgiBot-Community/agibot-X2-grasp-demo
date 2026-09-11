# 远端识别 API 与内置 URDF

[文档导航](README.md) · [简体中文](../README.md) · [English](README.en.md) · [Français](README.fr.md)

本项目同时包含远端视觉识别和本地机器人计算。部署前应明确两者的数据边界与适用范围。

## 远端识别 API

`grounding` 模式使用火山方舟远端视觉 API，不是在机器人本地运行目标检测模型。处理流程为：

1. `x2_grounding` 从机器人 RGB Topic 获取最新图像。
2. 节点在内存中将图像编码为 JPEG。
3. 配置中的 `target_descriptions` 作为目标描述构造提示词。
4. JPEG 图像和提示词通过 HTTPS 发送到 `base_url` 指定的火山方舟 API。
5. 远端服务返回归一化二维目标框。
6. 本地 C++ RGB-D 节点结合保留的深度帧，将目标框转换为 `base_link` 三维坐标。
7. C++ Pinocchio IK、轨迹规划和机器人控制全部在本地执行；原生扩展不可用时可显式回退 Python IK。

默认配置为：

```yaml
model: doubao-seed-2-1-pro-260628
base_url: https://ark.cn-beijing.volces.com/api/v3
api_key_env: ARK_API_KEY
request_timeout_seconds: 30.0
```

使用远端 API 意味着 RGB 图像会离开机器人并发送给配置的服务端。部署方需要自行确认网络、
隐私、数据合规、服务区域、模型授权、调用额度和费用。项目不会在仓库或安装目录中保存 API
Key，推荐通过 `ARK_API_KEY` 环境变量注入。

以下模式不会主动调用远端识别 API：

- `mode:=apriltag`：AprilTag 检测、solvePnP 和 TF 转换均在本地执行。
- `mode:=auto` 且存在新鲜稳定的 AprilTag：直接使用本地标签坐标。

`mode:=auto` 在没有可用 AprilTag 时会自动回退到 Grounding，因此仍可能上传 RGB 图像。完全
禁止图像外发的部署必须使用 `mode:=apriltag`，并在网络策略中限制远端访问。

远端 API 只提供二维识别框，不直接产生机械臂动作，也不会获得机器人控制权限。所有三维定位、
安全范围检查、IK 和执行决策仍由本地节点完成。

## 内置简化 URDF

功能包内置：

```text
src/x2_grasp/urdf/x2_ultra_plus_omnipicker_omnipicker.urdf
```

该文件随功能包安装，默认由 `x2_arm.config.default_urdf_path()` 定位，并由 Pinocchio 构建运动学
模型。它是面向本 Demo 的简化 URDF，主要保留：

- X2 机械结构的 link/joint 层级
- 右臂和左臂关节名称及顺序
- 关节轴、固定变换和机械限位
- Demo 使用的末端执行器 frame
- IK 模型构建所需的惯性和基础结构信息

它不应被视为完整的官方机器人描述，不能替代以下资源：

- 生产机器人当前固件对应的官方 URDF/Xacro
- 高保真 visual/collision mesh
- 碰撞检测、自碰撞矩阵和规划场景
- 控制器、transmission、ros2_control 和硬件接口配置
- 用于动力学辨识、安全认证或高精度仿真的模型

内置 URDF 的目标是让 Demo 在没有额外 description 包时能够完成 Pinocchio IK。它不会执行
环境碰撞检测，也不会保证针对不同 X2 批次、末端工具或标定状态完全准确。

## 部署校验

真机运行前必须确认：

1. 内置 URDF 的关节名称与 AimDK 返回的关节状态一致。
2. 右臂末端 frame 与实际安装的夹爪工具一致。
3. 机械限位与当前机器人固件一致。
4. `base_link`、相机光学 frame 和末端 frame 的 TF 正确。
5. 使用 `execute:=false` 验证目标、IK、预抓取、接近、抬升和后撤路径。
6. 机器人硬件急停和现场安全措施可用。

更换夹爪、增加转接件、修改机械结构或使用不同 X2 型号时，应使用对应的官方机器人描述重新
生成或替换运动学模型，并重新标定 `gripper_reach`、抓取平面和末端 frame。Action cancel 不是
碰撞保护或硬件急停。
