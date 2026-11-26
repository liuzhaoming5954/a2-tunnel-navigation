#!/bin/bash
# 由 a2-autonomy.service 经固定版本的 /etc/a2/autonomy-start 链接调用。
set -eu

# systemd 读取停车守护已经使用的 SDK 环境；这里只取解释器、网卡和域。
: "${A2_STOP_PYTHON:?缺少现场 SDK 解释器}"
: "${A2_STOP_INTERFACE:?缺少实测 SDK 网卡}"
[[ $A2_STOP_PYTHON == /* && -x $A2_STOP_PYTHON ]] || {
  echo "SDK 解释器须为可执行的绝对路径" >&2
  exit 1
}
entry=$(readlink -f -- "${BASH_SOURCE[0]}")
# 正式键盘通道仅由 PC2 的受管单元启用。已部署的其他遥控入口继续沿用自己的配置。
keyboard=()
case ${A2_REQUIRE_KEYBOARD_HOLD:-0} in
  0) ;;
  1) keyboard=(--keyboard-hold) ;;
  *) echo 'A2_REQUIRE_KEYBOARD_HOLD 只能是 0 或 1' >&2; exit 1 ;;
esac
# 持续按住只允许复用正式键盘证据。配置缺失时保留原受管入口的模式。
continuous=()
case ${A2_ALLOW_CONTINUOUS_HOLD:-0} in
  0) ;;
  1)
    [[ ${A2_REQUIRE_KEYBOARD_HOLD:-0} == 1 ]] || {
      echo '启用持续按住须同时启用独立方向键证据' >&2
      exit 1
    }
    continuous=(--continuous-hold)
    ;;
  *) echo 'A2_ALLOW_CONTINUOUS_HOLD 只能是 0 或 1' >&2; exit 1 ;;
esac
trace=()
case ${A2_TRACE_PARKING:-0} in
  0) ;;
  1) trace=(--trace-parking) ;;
  *) echo 'A2_TRACE_PARKING 只能是 0 或 1' >&2; exit 1 ;;
esac
cd /run/a2-manual
exec "$A2_STOP_PYTHON" -sB "$(dirname -- "$entry")/run_a2_manual_receiver.py" \
  --backend unitree --operator-supervised --interface "$A2_STOP_INTERFACE" --domain "${A2_STOP_DOMAIN:-0}" \
  --gateway-user unitree --socket /run/a2-manual/receiver.sock "${keyboard[@]}" "${continuous[@]}" "${trace[@]}"
