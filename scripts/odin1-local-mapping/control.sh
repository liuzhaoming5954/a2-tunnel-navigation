#!/usr/bin/env bash
set -u
run_dir=${1:?}
printf 'ODIN1 人工操作\n本轮目录 %s\n\n' "$run_dir"
printf '采集完并停稳后输入 s，再按 Enter，保存并取回地图。\n'
printf '看到 MAP_SAVED 后输入 c，再按 Enter，取消建图并切到 RAW。\n'
printf '输入 q，再按 Enter，等待已发起的保存完成后退出。输入 l 查看最近状态。\n\n'
while read -r -p 's 保存 | c 取消建图 | q 退出 | l 状态 > ' action; do
    if [[ -f "$run_dir/exit-code.txt" ]]; then
        printf '建图程序已退出，退出码 %s\n' "$(cat "$run_dir/exit-code.txt")"
        break
    fi
    pid=$(cat "$run_dir/probe.pid" 2>/dev/null) || continue
    if [[ ! "$pid" =~ ^[0-9]+$ ]] || [[ $(readlink "/proc/$pid/exe") != "$run_dir/odin1-local-mapping" ]]; then
        printf '%s\n' '本轮进程已结束，请查看日志窗口。'
        break
    fi
    printf "%s action=%s\n" "$(date --iso-8601=seconds)" "$action" >> "$run_dir/control.log"
    case "$action" in
        s|S)
            if grep -q 'PROBE MAPPING_CANCELLED' "$run_dir/probe.log"; then
                printf '%s\n' '本轮建图已经取消。'
                continue
            fi
            # 保存占用标记由主程序创建和移除；处理中重复按 s 不新增下一次保存。
            # c、q 仍各自提交请求，由主程序在当前保存完成后处理。
            if [[ -d "$run_dir/save-map.busy" ]]; then
                printf '%s\n' '保存正在处理中，请等待本次结果；本次 s 未新增保存请求。'
                continue
            fi
            if (set -o noclobber; : > "$run_dir/save-map.request") 2>/dev/null; then
                printf '%s\n' '已发送保存请求，请查看日志窗口中的 save_map 读回值和 MAP_SAVED。'
            else
                printf '%s\n' '保存请求已在等待处理。'
            fi
            ;;
        c|C)
            if grep -q 'PROBE MAPPING_CANCELLED' "$run_dir/probe.log"; then
                printf '%s\n' '本轮建图已经取消。'
                continue
            fi
            touch "$run_dir/cancel-map.request"
            printf '%s\n' '已提交取消建图请求，已发起的保存、取图完成后执行，请查看 MAPPING_CANCELLED。'
            ;;
        q|Q)
            touch "$run_dir/exit.request"
            printf '%s\n' '已提交退出请求，程序会等已发起的保存、取图完成后退出。保存超时可按 s 重试。'
            ;;
        l|L|'')
            grep 'PROBE ' "$run_dir/probe.log" | tail -8
            ;;
        *) printf '%s\n' '请输入 s、c、q 或 l，再按 Enter。' ;;
    esac
done
read -r -p '按 Enter 关闭操作窗口。' || true
