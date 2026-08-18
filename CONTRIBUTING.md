# Contributing

感谢你参与 X2 Grasp。提交改动前，请先搜索现有 Issue 和 Pull Request，避免重复工作。

## 开发环境

按照 [安装文档](docs/INSTALLATION.md) 准备 ROS 2 Humble、AimDK 和系统依赖：

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc
./scripts/install_dependencies.sh
colcon build --packages-select x2_grasp --symlink-install
source install/setup.bash
```

## 修改原则

- 保持仓库只有一个 ROS 2 功能包 `x2_grasp`
- 对外通信优先使用强类型 ROS 消息和 Action
- 新增抓取类型应通过配置完成，不应写死在业务代码中
- 真机行为变化必须同时提供 dry-run 验证路径
- 不提交 API Key、机器人凭据、构建目录或 Python 缓存
- Python 和 C++ 修改应遵循现有模块边界与命名风格

## 测试

```bash
colcon test --packages-select x2_grasp --event-handlers console_direct+
colcon test-result --verbose
```

没有 ROS 2 的开发机可以运行纯 Python 测试，但 Pull Request 说明中必须注明未执行的 ROS、
Pinocchio、C++ 或真机测试。

## Pull Request

Pull Request 应包含：

- 问题背景和行为变化
- 配置或接口兼容性说明
- 已执行的测试及结果
- 涉及真机控制时的 dry-run 和安全验证记录
- 新增参数、Topic、消息或 Action 时的文档更新

提交应保持主题明确，避免把格式化、重构和行为变化混在一个提交中。

## Commit

建议使用简洁的命令式提交信息，例如：

```text
feat: add configurable grasp target catalog
fix: fall back to the default completion cue
docs: document AprilTag calibration
test: cover invalid target aliases
```

## 许可证

提交贡献即表示你有权提供该贡献，并同意按仓库的 Apache License 2.0 发布。
