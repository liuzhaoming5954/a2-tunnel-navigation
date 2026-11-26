#!/usr/bin/env bash
set -uo pipefail
run_dir=$(realpath -e -- "${1:?需要本轮运行目录}") || exit 2
base=$(cd -- "$(dirname -- "$0")" && pwd)
# 所有脚本与二进制来自同一份快照，避免从上一级误取旧版文件。
if [[ "$base" != "$run_dir" ]]; then
    printf '%s\n' '请通过 start-manual.sh 建立快照，再运行快照中的入口。' >&2
    exit 2
fi
# 拒绝分支必须早于日志重定向、退出码写入与 USB 操作，保护已存在的证据。
for previous in probe.log probe.pid exit-code.txt .run-started; do
    if [[ -e "$run_dir/$previous" || -L "$run_dir/$previous" ]]; then
        printf '拒绝复用已有运行目录：%s 中存在 %s。\n' "$run_dir" "$previous" >&2
        exit 2
    fi
done
cd "$run_dir" || exit 2
sha256sum -c current-program.sha256 || exit 2
# 原子领取本轮目录。两个日志窗口同时启动同一快照时只有一个能够继续。
mkdir -- .run-started || exit 2
finish() {
    printf '%s\n' "$1" > "$run_dir/exit-code.txt"
    printf '\n测试退出码 %s，日志 %s/probe.log\n' "$1" "$run_dir"
    # SDK 进程已结束后释放设备锁；保留日志窗口不应阻止下一轮测试。
    flock -u 9 || true
    read -r -p '按 Enter 关闭日志窗口。' || true
}
# 整个会话持续持锁，防止两个新候选在进程检查与启动之间同时进入设备。
exec 9> "${XDG_RUNTIME_DIR:-/tmp}/odin1-local-mapping-${UID}.lock"
if ! flock -n 9; then
    printf '%s\n' '另一轮候选建图正在运行。'
    finish 2
    exit 2
fi
if pgrep -x host_sdk_sample >/dev/null || pgrep -f '^(/[^ ]*/|\./)?odin1-local-mapping( |$)' >/dev/null; then
    printf '%s\n' '已有 ODIN1 程序在运行。' | tee "$run_dir/preflight.txt"
    finish 2
    exit 2
fi
printf 'ODIN1 v0.14.5 设备内部建图 · 人工保存和停止\n运行目录 %s\n' "$run_dir"
if ! bash "$base/enable-usb.sh" 2>&1 | tee "$run_dir/preflight.txt"; then
    finish 3
    exit 3
fi
printf '采集完成并停稳后输入 s；看到 MAP_SAVED 后输入 c；最后输入 q 退出。\n'
(
    printf '%s\n' "$BASHPID" > "$run_dir/probe.pid"
    exec "$run_dir/odin1-local-mapping" "$run_dir" --manual-stop --allow-stopped --encrypted-logs
) 2>&1 | tee -i "$run_dir/probe.log" | (
    trap '' INT
    grep --line-buffered -v '\[DEBUG\]'
)
result=${PIPESTATUS[0]}
finish "$result"
exit "$result"
