#!/usr/bin/env bash
# 独立 PC2 遥控网关订阅既有遥测，只能通过受管接收器的套接字递交命令。
set -eo pipefail
: "${A2_ROS_SETUP:?缺少正式采集服务的 ROS 环境}"
: "${A2_INTERFACES_SETUP:?缺少正式采集服务的接口环境}"
: "${CREDENTIALS_DIRECTORY:?请由 systemd 装载遥控通道密钥}"
source "$A2_ROS_SETUP"
source "$A2_INTERFACES_SETUP"
set -u
# 现场用固定发布树中的入口接入。整个启动过程不加载底盘 SDK，不创建放行标志。
entry=$(readlink -f -- "${BASH_SOURCE[0]}")
root=$(dirname -- "$(dirname -- "$entry")")
reset_args=()
if [[ -n "${A2_STOP_RESET_SOCKET:-}" ]]; then
  # 仅独立遥控网关可以接入受管的本机复位套接字。
  reset_args=(--stop-reset-socket "$A2_STOP_RESET_SOCKET")
fi
exec /usr/bin/python3 "$root/scripts/run_a2_console.py" --role pc2 \
  --host 127.0.0.1 --port 18777 --enable-manual-control \
  --manual-key-file "$CREDENTIALS_DIRECTORY/manual.key" --manual-socket /run/a2-manual/receiver.sock \
  "${reset_args[@]}"
