# Security Policy

## Supported Version

仓库当前只维护默认分支的最新版本。机器人部署应固定到经过现场验证的 commit，不要在未完成
dry-run 和回归测试的情况下直接更新生产机器人。

## Reporting a Vulnerability

请不要在公开 Issue 中披露仍可被利用的漏洞。优先使用 GitHub 仓库的 Private vulnerability
reporting 功能：

<https://github.com/AgiBot-Community/agibot-X2-grasp-demo/security/advisories/new>

报告请包含受影响版本、复现条件、潜在影响和建议修复方式，但不要包含真实 API Key、机器人
访问凭据或现场网络信息。

## Robot Safety

Action cancel 不是硬件急停。已经发送的运动命令无法由本项目回滚，正在执行的同步硬件调用也
可能延迟响应取消。涉及运动安全的事件必须使用机器人硬件急停和 AimDK 安全机制处理。

## Credential Handling

- 使用 `ARK_API_KEY` 或权限受限的用户配置文件
- 不要把密钥写入 `unified_grasp.yaml`
- 不要在日志、Issue、录屏或测试夹具中提交真实密钥
- 泄露后应立即在服务提供方撤销并轮换凭据
