#!/usr/bin/env bash
# PC2 的开机服务入口。systemd 提供配置与可写的 SDK 工作目录，启动器不接收控制参数。
set -eo pipefail

: "${A2_PC2_RELEASE:?请配置绝对路径，指向已校验的 PC2 只读包根目录}"
: "${A2_ROS_SETUP:?请配置 ROS 发行版的 setup.bash}"
: "${A2_INTERFACES_SETUP:?请配置与发布包配套的 a2_interfaces 工作区 setup 文件}"
: "${A2_SDK_PYTHONPATH:?请配置原厂 Python SDK 所在目录}"
: "${A2_SDK_NIC:?请配置机载原厂 SDK 网卡}"
for entry in "$A2_PC2_RELEASE" "$A2_ROS_SETUP" "$A2_INTERFACES_SETUP" "$A2_SDK_PYTHONPATH"; do
  if [[ $entry != /* ]]; then
    echo "只读服务依赖绝对路径，收到：$entry" >&2
    exit 1
  fi
done

# 原厂 DDS 按网卡选择本机 IPv4 地址。网络初始化可能晚于服务启动，
# 此时交给 systemd 按 RestartSec 重试，避免 SDK 绑定到错误网卡或空地址。
if ! ip -o -4 address show dev "$A2_SDK_NIC" up scope global | grep -q ' inet '; then
  echo "等待原厂 SDK 网卡 $A2_SDK_NIC 启用并取得 IPv4 地址" >&2
  exit 1
fi

# ROS/colcon 环境脚本允许部分变量尚未定义，应在启用 nounset 之前加载。
source "$A2_ROS_SETUP"
source "$A2_INTERFACES_SETUP"
set -u
export PYTHONPATH="$A2_SDK_PYTHONPATH${PYTHONPATH:+:$PYTHONPATH}"
exec /bin/bash "$A2_PC2_RELEASE/scripts/start_a2_console_readonly.sh"
