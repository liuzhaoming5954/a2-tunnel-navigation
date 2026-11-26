#!/usr/bin/env bash
# 创建不可复用的运行快照，再打开日志、人工控制两个终端。
# 此入口不替操作员提交保存或停止；进入设备流程后仍由 s / c / q 分别控制。
set -euo pipefail
umask 077 # 地图、标定和设备日志默认只允许本用户访问。
base=$(cd -- "$(dirname -- "$0")" && pwd)
cd "$base"
if (( $# > 1 )); then
    printf '用法：%s [全新的运行目录]\n' "$0" >&2
    exit 2
fi
command -v gnome-terminal >/dev/null
sha256sum -c current-program.sha256
# SDK 已静态链接，构建清单覆盖二进制和 SDK 输入摘要。
if pgrep -x host_sdk_sample >/dev/null || pgrep -f '^(/[^ ]*/|\./)?odin1-local-mapping( |$)' >/dev/null; then
    printf '%s\n' '已有 ODIN1 程序在运行，请先结束当前轮次。' >&2
    exit 2
fi
if (( $# == 1 )); then
    run_dir=$(realpath -m -- "$1")
    # mkdir 不加 -p：已有目录（即使空目录）也不能作为新一轮重新使用。
    mkdir -- "$run_dir"
else
    run_dir=$(mktemp -d "$base/mapping.sdk0145.XXXXXX")
fi
while read -r checksum relative; do
    cp -- "$base/$relative" "$run_dir/$relative"
done < current-program.sha256
cp -- current-program.sha256 "$run_dir/current-program.sha256"
printf '本轮快照：%s\n' "$run_dir"
# 使用快照自己的脚本和二进制，之后源码变更不会改变本轮行为。
gnome-terminal --window --title='ODIN1 v0.14.5 建图日志' -- \
    bash "$run_dir/run-visible.sh" "$run_dir"
gnome-terminal --window --title='ODIN1 v0.14.5 人工控制' -- \
    bash "$run_dir/control.sh" "$run_dir"
