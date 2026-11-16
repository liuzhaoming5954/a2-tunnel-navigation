#!/usr/bin/env bash
set -euo pipefail

: "${A2_MAP_YAML:?Set A2_MAP_YAML to the trinary map YAML path}"
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

python3 "$root/scripts/publish_a2_map.py" --map "$A2_MAP_YAML" &
children+=("$!")
python3 "$root/scripts/publish_a2_platform_state.py" \
  --network-interface "$A2_SDK_NIC" --unitree-domain-id "$A2_SDK_DOMAIN_ID" \
  --robot-id "$A2_ROBOT_ID" &
children+=("$!")
python3 "$root/scripts/run_a2_console.py" --ros --host 127.0.0.1 \
  --port "${A2_CONSOLE_PORT:-8765}" &
children+=("$!")

# A required child exiting ends this maintenance session and cleans up its peers.
wait -n "${children[@]}"
