#!/usr/bin/env bash
set -euo pipefail

: "${A2_SDK_NIC:?Set A2_SDK_NIC to the Unitree SDK network interface}"
: "${A2_SDK_DOMAIN_ID:?Set A2_SDK_DOMAIN_ID to the Unitree DDS domain}"
: "${A2_ROBOT_ID:?Set A2_ROBOT_ID to the robot identifier}"

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
children=()
cleanup() {
  if (( ${#children[@]} )); then
    kill -TERM "${children[@]}" 2>/dev/null || true
    wait "${children[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# SDK 读取底盘状态，ROS 域按现场配置交给只读接口。地图与驾驶舱由上位机提供。
python3 "$root/scripts/publish_a2_platform_state.py" \
  --network-interface "$A2_SDK_NIC" --unitree-domain-id "$A2_SDK_DOMAIN_ID" \
  --robot-id "$A2_ROBOT_ID" &
children+=("$!")
# ODIN1 在自己的驱动域采集定位。此处仅把同机状态文件交给网关，两个 ROS 域可独立运行。
odin_args=()
if [[ -n "${A2_ODIN1_POSE_FILE:-}" ]]; then
  odin_args+=(--odin1-pose-file "$A2_ODIN1_POSE_FILE")
fi
watchdog_args=()
if [[ -n "${WATCHDOG_USEC:-}" && -n "${NOTIFY_SOCKET:-}" ]]; then
  # 正式只读单元配置看门狗。手工启动仍可用于联调。
  watchdog_args=(--watch-platform-stream)
fi
python3 "$root/scripts/run_a2_console.py" --role pc2 --host 127.0.0.1 \
  --port "${A2_CONSOLE_PORT:-8765}" "${odin_args[@]}" "${watchdog_args[@]}" &
children+=("$!")

# 任一必需的子进程退出时，结束本次维护会话并清理其他子进程。
wait -n "${children[@]}"
