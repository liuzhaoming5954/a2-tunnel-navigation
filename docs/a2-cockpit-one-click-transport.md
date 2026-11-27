# 驾驶舱控制链路开机重连

上位机对同一台 PC2 的手动控制和实体键盘分别建立独立 SSH 连接。可经现场已有的 PC3 中继，沿用已核验的主机指纹。两条连接各有一个 systemd 用户服务，各自重连。经 PC3 时也使用各自的 TCP 连接，仍共享现场 Wi‑Fi。只读遥测走原来的连接。

| 上位机入口 | PC2 入口 | 用途 |
| --- | --- | --- |
| `127.0.0.1:8766` | `127.0.0.1:8765` | 只读遥测和 ODIN1 位姿，沿用原只读身份 |
| `127.0.0.1:18777` | `127.0.0.1:18777` | 手动遥控网关，请求继续使用原机间鉴权 |
| `127.0.0.1:18778` | `127.0.0.1:18778` | 实体方向键证据，沿用独立键盘通道逐帧鉴权 |

PC2 晚于上位机进入 Wi‑Fi、SSH 中断或回环端口尚未就绪时，各链路独立重试。运动中断流停车仍由 PC2 的键盘证据失效和速度包超时处理。网络恢复后，驾驶舱重新取得实时状态，操作员仍需点击「启用控制」。运动由新的会话和新的实体按键触发。连接服务无需操作员维持 SSH 窗口。[控制重连单元](../deploy/templates/a2-cockpit-control-link.service) · [键盘重连单元](../deploy/templates/a2-cockpit-keyboard-link.service)

## 现场一次性配置

1. 在 PC2 为 `a2-cockpit-control` 独立账号登记上位机的专用 SSH 公钥。私钥留在上位机，原只读身份及 PC3 中继身份沿用现场配置。PC2 将账号限制为公钥认证、两个回环转发目标，禁止远端转发、Unix 套接字转发和交互会话。将账号条件块放在 PC2 的 SSH 全局配置项之后，生效前用 `sshd -t` 和 `sshd -T -C` 核对语法和该账号的实际规则。[PC2 SSH 账号规则](../deploy/templates/a2-cockpit-control-sshd.conf.example) · [sshd 配置手册](https://man.openbsd.org/sshd_config)
2. 在上位机已有的 `~/.config/a2-cockpit/ssh_config` 补充 `a2-cockpit-pc2-control`。控制别名的 PC2 地址、PC3 中继和已登记主机指纹使用现场值。将控制和键盘链路单元装入上位机 systemd 用户单元目录并分别启用。原站端维护配置继续使用 `--enable-manual-control`、已有机间密钥文件和 `--manual-pc2-url http://127.0.0.1:18777`。[上位机 SSH 配置示例](../deploy/templates/a2-cockpit-ssh.conf.example)
3. 在 PC2 以已配置好机间凭据、ROS 接口和回环监听的 `a2-cockpit-manual-gateway.service` 启用开机运行。先启用两端既有独立键盘服务。网关开机只启动 HTTP 监听，发令进程的受管启动条件仍由本机守护检查。[PC2 手动网关单元](../deploy/templates/a2-cockpit-manual-gateway.service) · [上位机键盘服务](../deploy/templates/a2-keyboard-hold-station.service) · [PC2 键盘服务](../deploy/templates/a2-keyboard-hold.service)

## 静止验证

保持停车锁存，分别查看三个站端回环端口，以及 PC2 网关和键盘服务状态。驾驶舱应显示遥测在线、锁存有效、控制禁用。单独断开控制连接时，检查键盘证据继续更新。单独断开键盘连接时，检查控制状态查询继续可用，键盘证据按时失效。两次检查中的只读遥测应继续更新，重连后旧会话保持失效。开发机可运行 `python3 -m unittest scripts.test.test_cockpit_link` 核对模板端口、身份隔离和重连配置。

## 分段诊断

上位机 `a2-keyboard-hold-station.service` 与 PC2 `a2-keyboard-hold.service` 共用 `A2_KEYBOARD_DIAG` 日志前缀。用 `journalctl -u <服务名> -o cat` 查看对应机器的系统日志。日志按事件类别限流，记录连接阶段 `phase`、耗时 `phase_elapsed_ms`、最近和最大键盘采集耗时、成功帧数、异常原因及抑制计数。单机间隔以 `mono_ns` 核对。用 `wall_ns` 跨机对齐前，先估计各机时钟偏差。日志只包含状态和耗时。[诊断实现](../scripts/a2_keyboard_hold_service.py)

PC2 的 `reply_read` 超时表示 130 毫秒内未收到完整的挑战应答。对此按同一时间窗核对上位机采集与本地回复耗时、PC3 无线和有线侧 TCP 队列及重传、PC2 的收发计数和键盘证据年龄。PC2 用 200 毫秒的证据时效撤销已失效的会话。SSH 正常连接或控制状态可读时也可能出现单帧超时。采集保持停车锁存，保留汇总耗时与分段事实，现场连接、原始记录和凭据按仓库约定保管。[现场排查记录](implementation-status.md#连续换向与独立链路排查) · [仓库约定](github-setup.md)
