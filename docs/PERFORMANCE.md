# 性能与 C++ 迁移

## 迁移边界

仓库中的数值计算按以下边界处理：

| 路径 | 实现 | 结论 |
|---|---|---|
| Pinocchio 模型、关节限位、配置映射 | C++/pybind11 | 已迁移 |
| `fk_xyz`、`fk_rpy`、`fk_axis` | C++/pybind11 | 已迁移 |
| `solve_position` | C++/pybind11 | 已迁移 |
| `solve_pose`、`solve_6d` | C++/pybind11 | 已迁移 |
| `solve_axis` | C++/pybind11 | 已迁移 |
| RGB-D 深度注册和定位 | C++/OpenCV | 原本已经是 C++ |
| AprilTag 检测、PnP、图像转换 | OpenCV C++ Python API | 数值内核原本已经是 C++ |
| `arm_pos_from_q` | NumPy 索引 | C++ 跨语言返回更慢，保留 Python |
| 轨迹插值 | NumPy/Python | 约 0.12 ms 生成 101 点，相对 2 s 发布周期可忽略 |
| 抓取阶段、取消、重试、日志和降级 | Python | 属于业务编排，数值 IK 已在 C++ |

`NativeX2ArmIKSolver` 的普通路径只构建一份 C++ Pinocchio 模型。仅当调用者传入
`q_seed` 或 `current_head_pos` 时，才惰性创建 Python 求解器用于兼容这些特殊输入。
同一原生求解器的 Pinocchio `Data` 访问由实例互斥量串行化，pybind11 释放 GIL
后仍可安全地被多个 Python 线程调用。

## 测试环境

- 日期：2026-08-25
- WSL：Ubuntu 22.04
- ROS：ROS 2 Humble
- Python：3.10.12
- Pinocchio：4.0.0（`ros-humble-pinocchio`）
- 构建类型：Release
- 微操作：5000 次，预热 20 次
- IK 和链式规划：100 次，预热 20 次
- 求解器构造：10 次，预热 2 次

运行命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/benchmark_ik.py --backend python --side right \
  --micro-iterations 5000 --solve-iterations 100 --init-iterations 10 --warmup 20
python3 scripts/benchmark_ik.py --backend native --side right \
  --micro-iterations 5000 --solve-iterations 100 --init-iterations 10 --warmup 20
```

左臂使用相同参数并将 `--side` 改为 `left`。下表均为中位数；加速比为
`Python / native`，大于 1 表示原生后端更快。

## 右臂结果

| 操作 | Python (us) | Native (us) | 加速比 |
|---|---:|---:|---:|
| 求解器构造 | 8789.234 | 7799.433 | 1.13x |
| 限位裁剪 | 5.090 | 0.593 | 8.58x |
| `q_from_arm_pos` | 3.216 | 1.728 | 1.86x |
| `arm_pos_from_q` | 0.441 | 0.479 | 0.92x |
| `fk_xyz` | 6.332 | 2.614 | 2.42x |
| `fk_rpy` | 6.418 | 2.697 | 2.38x |
| `fk_axis` | 12.684 | 2.728 | 4.65x |
| `solve_position` | 597.624 | 102.169 | 5.85x |
| `solve_pose` | 927.638 | 116.472 | 7.96x |
| `solve_6d` | 925.735 | 150.936 | 6.13x |
| `solve_axis` | 1941.721 | 115.571 | 16.80x |
| 8 点轴向 IK 链 | 10795.290 | 691.490 | 15.61x |

`arm_pos_from_q` 的原生返回曾测得约 0.93 us，慢于 NumPy 的约 0.46 us，
因此最终实现让两个后端都使用缓存索引和 NumPy。表中的微小差异属于测量噪声。

## 左臂结果

| 操作 | Python (us) | Native (us) | 加速比 |
|---|---:|---:|---:|
| 求解器构造 | 10648.655 | 7928.623 | 1.34x |
| 限位裁剪 | 5.017 | 0.577 | 8.70x |
| `q_from_arm_pos` | 3.232 | 1.662 | 1.94x |
| `arm_pos_from_q` | 0.439 | 0.456 | 0.96x |
| `fk_xyz` | 6.174 | 2.503 | 2.47x |
| `fk_rpy` | 6.273 | 2.604 | 2.41x |
| `fk_axis` | 12.566 | 2.604 | 4.83x |
| `solve_position` | 656.211 | 97.739 | 6.71x |
| `solve_pose` | 927.782 | 109.944 | 8.44x |
| `solve_6d` | 916.667 | 112.797 | 8.13x |
| `solve_axis` | 2091.279 | 109.362 | 19.12x |
| 8 点轴向 IK 链 | 10866.526 | 670.303 | 16.21x |

所有右臂和左臂 IK 基准的 Python/native 平均迭代数均为 42。性能提升没有通过
减少迭代次数、放宽误差阈值或改变目标实现。

## 保留 Python 的路径

典型 2 秒、50 Hz 的 101 点轨迹插值中位数为 Python 119.618 us、原生 IK
配置下 123.840 us。该操作只占实际 2 秒发布周期约 0.006%，且输出仍需构造成
Python/ROS 消息，因此迁移没有实际收益。

8 点链式规划已经包含 Python 的逐点循环、结果封装和跨语言调用。它在原生后端下
仍达到 15.61x 到 16.21x 加速，说明剩余 Python 编排不是主要瓶颈。保留逐点边界
还能维持每个路径点的取消检查、日志、近似解降级和失败定位，不应为了很小的调用
开销整体迁入 C++。

这里比较的是数值计算和包含 Python 编排的链式 IK，不代表 ROS 2 控制命令的周期抖动。
机械臂/夹爪发布器是否迁入 C++应使用控制器节拍、deadline miss、watchdog 和停止延迟评估，
详见 [发布管理](PUBLISHING.md)。
