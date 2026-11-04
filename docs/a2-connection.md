# 连接 A2 并完成首次数据采集

适用：开发机为 Ubuntu 24.04，通过已确认的维护有线网或维护 Wi-Fi 登录 A2 的感知计算机，检查原装雷达、IMU、摄像头并采集数据。2026-11-05 编写，**尚未在 A2 上执行验证**；维护口、IP、账号、SSH 端口及 ROS 环境均待现场确认。

本次目标是完成“连接 → 登录 → 核查 → 传工具 → 录短包 → 带回数据”。连接、诊断和采集在机器人停稳的维护状态下进行；路线采集另由现场人员通过原厂方式操作。本流程不部署自主运动程序。

文中 **【本机】** 指开发电脑终端，**【A2】** 指 SSH 登录后的机器人终端。命令分段执行，确认当前步骤结果后再继续；不要把整篇一次粘贴执行。

## 1. 到现场前准备

带上开发电脑、适配维护接口的网线/转接器或已确认的维护 Wi-Fi 信息，以及采集工具包；这些工具不要求机器人能访问互联网。

先向设备资料、现场管理员或机载本地控制台核实下表，记录到本机私有的 `config/local/a2-connection-notes.md`，不把密码或私钥写进记录。

| 项目 | 现场填写 |
|---|---|
| 机器人编号、感知计算机名称/型号 | 待填写；确认目标是否为运行原装传感器驱动的 Orin |
| 维护连接方式 | 有线维护口 / 维护 Wi-Fi；确认接口位置或 SSID |
| A2 维护 IPv4 地址 | 待填写，以下命令使用 IPv4 |
| 开发机地址方式 | DHCP / 指定静态 IPv4 与前缀长度；静态地址须确认未占用 |
| SSH 用户名与端口 | 待填写；不能假定用户名、默认密码或端口 |
| 登录凭据 | 已有密码 / 已配置的私钥，仅在凭据管理处保存 |
| SSH 主机公钥指纹 | 从可信设备记录或机载控制台核对 |
| ROS / 厂商驱动环境 | 实际发行版、setup 文件及所需 ROS domain / DDS 配置，未知则登录后核查 |

**维护地址、底盘 SDK 地址和雷达地址用途不同。** 设计资料中的雷达 IP、SDK 网段不能作为 Orin 的 SSH 地址。若登录目标实际为 PC2 或其他主机，先确认感知计算机的入口，再继续；不要把“能登录某台计算机”当作传感器接入已完成。

已有离线包为 `artifacts/a2-capture-kit.zip`。包含本连接文档的新包为 `artifacts/a2-capture-kit-with-connection.zip`，本流程使用后者；包中只有源码工具、示例和说明，不包含 ROS、厂商驱动、现场标定或 ARM64 依赖。

## 2. 接入维护网络

**【本机】** 先记录原有网卡、地址和路由，便于结束后恢复：

```bash
ip -br link
ip -br addr
ip route
nmcli device status
```

有线维护：按设备说明连接指定维护口，在 Ubuntu“设置 → 网络 → 对应有线连接 → IPv4”中操作。维护网提供 DHCP 时使用自动配置；要求静态地址时，仅为开发机连接该维护口的网卡填写确认过的地址和掩码。独立直连网络通常无需网关和 DNS，按实际维护网要求填写，避免把它设成默认上网路径。记录原设置后再应用。

维护 Wi-Fi：连接已确认的 SSID，凭据通过系统交互输入；若显示“无互联网”但能访问维护地址，可以继续。不要启用网络共享或给机内网络桥接外网。

不调整 A2 原有机内网卡、路由、雷达 IP 或系统时间。若维护网与本机其他网络/VPN 网段重叠，先核查路由并调整开发机维护连接，必要时现场协助；不要通过清空全部路由或关闭机内网络排查。

## 3. 检查路由和 SSH 登录

**【本机】** 在一个终端输入现场核实的信息，变量仅在当前终端有效；新开本机终端时需重新设置。这里不输入密码：

```bash
read -r -p 'A2 维护 IPv4 地址：' A2_SSH_HOST
read -r -p 'A2 SSH 用户名：' A2_SSH_USER
read -r -p 'A2 SSH 端口：' A2_SSH_PORT
ip route get "$A2_SSH_HOST"
ping -c 3 -W 1 "$A2_SSH_HOST"
```

路由结果的 `dev` 应是预期维护网卡，`src` 应是该接口的开发机地址。ping 成功可证明 ICMP 可达；禁用 ICMP 的设备仍可能允许 SSH，ping 失败不能单独判定 SSH 不可用。

登录：

```bash
ssh -p "$A2_SSH_PORT" \
  -o ConnectTimeout=8 \
  -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  "$A2_SSH_USER@$A2_SSH_HOST"
```

首次登录显示主机指纹时，与可信记录核对后接受。可由机载控制台读取实际启用的主机公钥指纹，例如 `ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub`。主机密钥变更时先查明是否换机、重装或连错地址；不要关闭主机密钥检查或直接删除旧记录来绕过提示。

密码在 SSH 提示时输入，终端不回显是正常行为。已采用密钥认证时，在上面的 SSH 命令中添加 `-i` 和本机私钥路径，后续 scp 也使用同一身份；不把私钥传给 A2，也不把密码写进命令。

成功后当前终端变为 **【A2】**。传包或拉取数据时，另开 **【本机】** 终端并重新输入这三个连接变量，保留 A2 会话。

## 4. 确认登录的是目标计算机

**【A2】** 先执行只读检查：

```bash
hostname
whoami
uname -m
cat /etc/os-release
python3 --version
ip -br addr
ip route
df -h .
free -h
ls /opt/ros
```

Orin 预期为 `aarch64`；若实际不符，记录主机身份并确认是否登录错计算机。Jetson 上若存在 `/etc/nv_tegra_release`，另读取首行登记 L4T 信息。记录结果，不覆盖出厂系统，也不升级或重启原厂服务。

当前采集工具按 Python 3.10+ 准备。若目标版本更旧，先记录环境并核实兼容方案，不替换原厂系统 Python。

没有 `/opt/ros` 不一定表示没有自定义 ROS 安装；核实厂商环境路径或容器部署。原厂硬件能用，也不代表已提供 ROS 2 `PointCloud2` / `Imu` / `Image`，后续发现步骤用于确认这一点。

## 5. 上传采集工具并校验

**【本机】** 先进入自己的项目根目录，再核对工具包并生成外层校验文件：

```bash
cd artifacts
sha256sum a2-capture-kit-with-connection.zip > a2-capture-kit-with-connection.zip.sha256
```

为本次维护选一个新目录，例如 `a2_capture_visit_01`。先在 A2 用户主目录下创建它；若提示目录已存在，改用新编号，不覆盖旧采集：

```bash
ssh -p "$A2_SSH_PORT" "$A2_SSH_USER@$A2_SSH_HOST" 'mkdir a2_capture_visit_01'
```

上一条成功后上传。SSH 端口参数是小写 `-p`，scp 是大写 `-P`：

```bash
scp -P "$A2_SSH_PORT" \
  a2-capture-kit-with-connection.zip \
  a2-capture-kit-with-connection.zip.sha256 \
  "$A2_SSH_USER@$A2_SSH_HOST:a2_capture_visit_01/"
```

**【A2】** 在保留的 SSH 会话中校验、解压并核对包内文件：

```bash
cd ~/a2_capture_visit_01
sha256sum -c a2-capture-kit-with-connection.zip.sha256
unzip a2-capture-kit-with-connection.zip
cd a2-capture-kit
sha256sum -c SHA256SUMS
mkdir -p data/captures config/local
```

两次校验均应全部 `OK`。若没有 `unzip`，可使用已安装的 Python：`python3 -m zipfile -e a2-capture-kit-with-connection.zip .`，在 ZIP 所在目录执行。缺少其他必要依赖时在维护阶段准备，不要求机器人巡检时联网安装。

也可用受控 U 盘传输 ZIP 和对应 `.sha256`，复制到新的维护目录后执行相同校验与解压步骤。

## 6. 加载 A2 的实际 ROS / 驱动环境

**【A2】** 从 `/opt/ros` 或厂商资料确认实际发行版；以下只适用于该标准安装路径存在的情况：

```bash
read -r -p 'A2 实际 ROS 2 发行版：' A2_ROS_DISTRO
if [ -r "/opt/ros/$A2_ROS_DISTRO/setup.bash" ]; then
  source "/opt/ros/$A2_ROS_DISTRO/setup.bash" && printf '当前 ROS_DISTRO=%s\n' "${ROS_DISTRO:-未设置}"
else
  printf '未找到 setup.bash，请核实实际环境，暂不执行后续步骤。\n' >&2
fi
```

路径错误、source 返回非零或加载后的 `ROS_DISTRO` 与预期不符时，先停在此步骤排查。再按厂商说明加载实际驱动工作区的 setup 文件。自定义路径/容器安装时使用厂商给出的入口，不自动 source 找到的所有环境。新开 SSH 会话需重新核对和加载。

记录 ROS 通信设置并检查依赖：

```bash
printenv ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY
python3 -c 'import rclpy, rosbag2_py, sensor_msgs.msg, tf2_msgs.msg, rosidl_runtime_py; print("ROS Python imports OK")'
ros2 bag record --help
ros2 topic list -t
```

`printenv` 中未设置的项可能不显示；按原厂进程实际使用的 domain/RMW/DDS 配置核查，不能把开发机演示用的 `ROS_DOMAIN_ID=78/79` 或 `ROS_LOCALHOST_ONLY=1` 照搬到这里。SSH 连接不转发 ROS 2 发现；本次录包进程直接在 A2 上运行，开发机只用于 SSH 和传文件。

如果只有原厂 UDP/DDS 或视频接口而没有所需 ROS 话题，记录接口和驱动部署情况，先完成相应接入。保留原有驱动，不为“看不到话题”重复启动驱动、发布虚假输入或启动运动示例。

## 7. 核查话题并录制首个短包

**【A2】** 确认仍在 `~/a2_capture_visit_01/a2-capture-kit`，执行：

```bash
python3 scripts/a2_capture.py inventory \
  --output data/captures/a2-inventory.json
cp config/examples/mapping-capture.json config/local/mapping-capture.json
```

使用现有文本编辑器填写 `config/local/mapping-capture.json`：

| 配置 | 填写内容 |
|---|---|
| `topics.front_points` | 实际前雷达 `sensor_msgs/msg/PointCloud2` 话题 |
| `topics.imu` | 实际六轴 `sensor_msgs/msg/Imu` 话题，另记录 IMU 来源 |
| `topics.rear_points` | 可选：实际后雷达点云话题 |
| `topics.image` / `topics.camera_info` | 可选：实际 `Image` / `CameraInfo`；当前录制入口不支持压缩图像或 RTSP |
| `point_time.field` / `point_time.unit` | 按驱动核实字段及 `s/ms/us/ns`；未知时两项均留空，先保存原始数据 |
| `storage_id` | 默认 `sqlite3`，须已安装对应存储插件 |

必需角色为 `front_points`、`imu`，各话题不得重复；空示例不能录制。`/tf`、`/tf_static` 自动纳入。然后核查实际样本：

```bash
python3 scripts/a2_capture.py inventory \
  --config config/local/mapping-capture.json \
  --output data/captures/a2-sensor-check.json
```

先排查核心话题缺失、类型不符和无数据等错误。TF、frame 或每点时间未决项需要记录；允许先保留原始数据，但不能据此判断可建图。确认磁盘容量后录制 60 秒静止短包：

```bash
python3 scripts/a2_capture.py record \
  --config config/local/mapping-capture.json \
  --session data/captures/a2-static-01 --duration 60 --execute
```

等待到时正常收尾，或按 Ctrl-C 并等待完成；不要在录制期间退出 SSH 或拔线。SSH 断开不保证录包自动完整结束，重连后需先确认本次采集进程和文件状态，再决定补录，避免重复启动。

检查实际消息：

```bash
ros2 bag info data/captures/a2-static-01/bag
python3 scripts/inspect_mapping_bag.py data/captures/a2-static-01/bag \
  --config data/captures/a2-static-01/config.json \
  --output data/captures/a2-static-01/bag-check.json
```

查看检查报告、`recording-result.json` 和 `bag/metadata.yaml`。缺点内时间会报告未决 issue，需要核实；文件存在和消息有数量不证明时间同步、外参或去畸变已合格。静止短包检查后再按现场安排采集新目录的直行、转弯和短路线往返数据。原装传感器可用即可推进，新增巡检载荷不影响本次录制。

## 8. 保存记录并完整带回数据

**【A2】** 等所有本次采集正常停止后，将环境/话题核查报告复制到对应会话，再在会话中保存 `field-notes.md`，登记 IMU 来源、原始时钟/每点时间参考、路线参考和异常；另保存厂商驱动及标定配置副本，不包含凭据。

```bash
cp data/captures/a2-inventory.json data/captures/a2-static-01/
cp data/captures/a2-sensor-check.json data/captures/a2-static-01/
cd data/captures/a2-static-01
find . -type f ! -path './FILES.sha256' -print0 | sort -z | xargs -0 sha256sum > FILES.sha256
sha256sum -c FILES.sha256
```

清单应在文件停止变化、现场记录补齐后生成。此后有修改就重新生成清单；不要一边录制一边核验。

**【本机】** 先回到自己的项目根目录，再拉取完整会话。每次使用新的接收目录，下面的 `visit-01` 若已存在请换编号；不要加 `--delete`，不要只复制一个 `.db3` / `.mcap` 分片。

```bash
mkdir -p data/captures
mkdir data/captures/visit-01
scp -P "$A2_SSH_PORT" -r \
  "$A2_SSH_USER@$A2_SSH_HOST:a2_capture_visit_01/a2-capture-kit/data/captures/a2-static-01" \
  data/captures/visit-01/
cd data/captures/visit-01/a2-static-01
sha256sum -c FILES.sha256
```

全部 `OK` 后，回到本机项目根目录、加载本机 ROS 环境，用 `inspect_mapping_bag.py` 再读一次包，输出到新的报告文件，确认目标环境实际可读取。其他路线会话逐个复制、核验；文件较大时可改用已确认可用的断点续传工具或受控介质，仍按完整会话核对 SHA-256。源端副本保留到本机确认完整且完成后续备份。

## 9. 正常结束与排障

**SSH 意外断开后的收尾【A2】**：重连后先检查本次进程。工具中的 recorder 使用独立会话，父脚本若已退出，子进程可能仍在录制，此时时限与磁盘保留线监控也可能已失效，应先处理再开展其他检查。

```bash
pgrep -af 'a2_capture.py|ros2 bag record'
read -r -p '待核实的本次采集进程 PID：' A2_CAPTURE_PID
ps -p "$A2_CAPTURE_PID" -o pid,ppid,pgid,etime,args
```

人工核对命令与本次会话绝对路径。若本次 `a2_capture.py` 父进程仍运行，选择该父进程；若父进程已退出，选择已核实的本次 `ros2 bag record` 进程。确认后才发送 SIGINT 请求写完缓存并退出：

```bash
if [[ "$A2_CAPTURE_PID" =~ ^[1-9][0-9]*$ ]] && (( A2_CAPTURE_PID > 1 )); then
  kill -s INT "$A2_CAPTURE_PID"
fi
```

等待写盘、确认 recorder 退出后再检查 metadata 与实际消息，记录断线和人工收尾情况。父脚本退出时可能没有 `recording-result.json`，不能补写成正常完成；有疑问就补录。不要批量 `pkill`，不要用 SIGKILL 代替正常收尾，不能误停原厂记录进程。

结束前在 **【A2】** 确认本次 recorder/观察工具已退出、结果已保存。可用 `pgrep -af a2_capture.py` 和 `pgrep -af 'ros2 bag record'` 查看候选进程，核对 PID、命令及会话路径，不能直接按名称批量杀进程；原厂服务可能也有记录进程。然后 `exit` 退出 SSH，恢复开发机原网络设置并移除维护连接。路线移动必须已由操作员停稳；拔维护线不作为停车指令。

| 现象 | 排查顺序 |
|---|---|
| `Network is unreachable` / 路由走错接口 | 维护网卡链路、开发机地址/掩码、网段重叠与路由；核对目标维护 IP |
| SSH 超时 | 电源/维护口、维护地址、网络可达性、实际端口及维护访问策略；不是先改账号 |
| `Connection refused` | 目标端口没有监听或被明确拒绝；确认 SSH 服务/端口，不重启整机试错 |
| `Permission denied` | 用户名、密码或指定密钥、账号登录策略；不猜默认密码或关闭认证 |
| 主机指纹变化 | 核实设备身份、系统变更与可信指纹，再处理本机记录 |
| scp 报 SFTP subsystem 不可用 | 核实厂商文件传输入口；确认仅支持旧 SCP 时才评估 `scp -O`，不盲目修改 sshd |
| SSH 正常但 ROS 无话题 | 登录目标、实际 ROS/厂商环境、原厂驱动位置、domain/RMW/DDS/QoS；SSH 本身不转发话题 |
| Python 模块或存储插件缺失 | 记录实际发行版，在维护期准备匹配的 ARM64 依赖；不照搬本机 x86_64 目录 |
| SSH 录制中断开 | 重连核实本次 PID/会话、结果文件与元数据，检查实际包；有缺口就补录 |
| 回传校验不一致 | 保留源数据，确认源文件已停写、全分片齐全、传输完成，再重新复制 |

本次连接完成的结果应是：确认目标主机与环境，保存实际话题/字段核查结果，取得可读取的首个短包，完整回传并核验文件。此结果不等于 M0 全部结束或建图通过。

项目工作区中的补充说明见[录包与建图验证](mapping-capture.md)、[本机开发与实机验证](development.md)。离线工具包保留本文全文，可直接按上述步骤操作；关联项目文档在完整工作区查看。

命令依据：[OpenSSH ssh 手册](https://man.openbsd.org/ssh)、[OpenSSH scp 手册](https://man.openbsd.org/scp)、[NetworkManager nmcli 手册](https://networkmanager.dev/docs/api/latest/nmcli.html)。主机连接和文件传输选项已据官方手册核对，A2 的厂商网络参数仍以实机资料为准。
