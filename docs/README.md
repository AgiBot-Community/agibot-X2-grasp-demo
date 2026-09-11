# 文档导航 / Documentation / Documentation trilingue

[简体中文](../README.md) · [English](README.en.md) · [Français](README.fr.md)

## 三语入门 / Getting started / Bien démarrer

三种语言的项目说明均包含项目特性、系统要求、安装构建、运行示例、配置、测试及排错入口。
命令、ROS 接口、参数名和目标标识符保持一致，可以直接通过相对链接切换语言。

The three main guides cover features, requirements, installation, examples, configuration,
tests, and troubleshooting. Commands, ROS interfaces, parameter names, and target identifiers
remain consistent across languages. All documentation links are relative to this repository.

Les trois guides principaux présentent les fonctionnalités, les prérequis, l'installation,
les exemples, la configuration, les tests et le dépannage. Les commandes, interfaces ROS,
paramètres et identifiants de cible restent identiques. Les liens sont relatifs au dépôt.

| 语言 / Language / Langue | 项目说明 / Main guide / Guide principal |
| --- | --- |
| 简体中文 | [中文说明](../README.md) |
| English | [English guide](README.en.md) |
| Français | [Guide français](README.fr.md) |

## 专题参考 / Detailed references / Références détaillées

以下专题文档和示例说明目前为中文；英文和法文读者可先阅读对应语言的项目说明。

The detailed references and example walkthrough below are currently in Chinese.
For installation and operation in English, start with the [English guide](README.en.md).

Les références et les exemples détaillés ci-dessous sont actuellement en chinois.
Pour l'installation et l'utilisation en français, commencez par le [guide français](README.fr.md).

| 文档 / Document | 内容 / Scope / Contenu |
| --- | --- |
| [安装与构建 / Installation / Installation](INSTALLATION.md) | ROS、AimDK、依赖、构建验证 / Dependencies and build / Dépendances et compilation |
| [运行与调用 / Usage / Utilisation](USAGE.md) | 模式、Action、取消、真机 / Modes, cancellation, hardware / Modes, annulation, robot |
| [示例 / Examples / Exemples](../example/README.md) | 多终端操作 / Terminal walkthrough / Procédure par terminal |
| [接口 / Interfaces / Interfaces](INTERFACES.md) | Action、消息、Topic / Actions, messages, topics |
| [配置 / Configuration / Configuration](CONFIGURATION.md) | 感知、IK、夹爪、音频 / Perception, IK, gripper, audio / Perception, IK, pince, audio |
| [架构 / Architecture / Architecture](ARCHITECTURE.md) | 模块、线程、数据流 / Modules, threads, data flow / Modules, threads, flux de données |
| [性能 / Performance / Performances](PERFORMANCE.md) | C++ IK、基准 / Native IK and benchmarks / IK native et mesures |
| [命令发布 / Command publishing / Publication des commandes](PUBLISHING.md) | 50 Hz、取消、watchdog / 50 Hz, cancellation, watchdog / 50 Hz, annulation, watchdog |
| [API 与 URDF / API and URDF / API et URDF](REMOTE_API_AND_URDF.md) | 远端数据与模型边界 / Data transfer and model limits / Transfert de données et limites du modèle |
| [故障排查 / Troubleshooting / Dépannage](TROUBLESHOOTING.md) | 定位与执行诊断 / Perception and execution diagnostics / Diagnostic de perception et d'exécution |
| [包内说明 / Package guide / Guide du paquet](../src/x2_grasp/README.md) | 内部行为和参数 / Internals and parameters / Fonctionnement interne et paramètres |

## 源码入口 / Source links / Liens vers le code

- [统一配置 / Unified configuration / Configuration unifiée](../src/x2_grasp/config/unified_grasp.yaml)
- [启动文件 / Launch file / Fichier de lancement](../src/x2_grasp/launch/unified_grasp.launch.py)
- [Python Action 客户端 / Python Action client / Client Action Python](../example/send_grasp_goal.py)
- [许可证 / License / Licence](../LICENSE)
