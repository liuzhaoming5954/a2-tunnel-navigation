#!/usr/bin/env bash
# 只编译 x86_64 开发机上的板载建图工具。每次构建都生成独立目录，保留上轮证据。
set -euo pipefail
umask 077
source_dir=$(cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(cd -- "$source_dir/../.." && pwd)
if (( $# > 2 )); then
    printf '用法：bash %s [厂商 SDK 仓库] [全新的构建目录]\n' "$0" >&2
    exit 2
fi
sdk_dir=$(cd -- "${1:-$repo_dir/third_party/src/odin_ros_driver-0.14.5}" && pwd)
sdk_commit=9487890acd648f1c7647576d1ad2b04dd7fd3677
if [[ $(git -C "$sdk_dir" rev-parse HEAD) != "$sdk_commit" ]]; then
    printf '需要厂商 v0.14.5 提交 %s。\n' "$sdk_commit" >&2
    exit 1
fi
# 同时核验工作树中的真实输入，避免未提交的头文件或库与指定提交不一致。
(cd -- "$sdk_dir" && sha256sum -c "$source_dir/sdk-files.sha256") >&2
if (( $# == 2 )); then
    output=$(realpath -m -- "$2")
    mkdir -- "$output"
else
    runs="$repo_dir/artifacts/odin1-local-mapping"
    mkdir -p -- "$runs"
    output=$(mktemp -d "$runs/build.XXXXXX")
fi
# 固定实际编译的源码和运行脚本。清单最后生成，只有完整的构建可启动。
inputs=(odin1-local-mapping.cpp odin1-local-mapping-support.cpp odin1-local-mapping-support.h
        start-manual.sh run-visible.sh control.sh enable-usb.sh sdk-files.sha256 build.sh)
for name in "${inputs[@]}"; do
    cp -- "$source_dir/$name" "$output/$name"
done
cd -- "$output"
g++ -std=c++17 -O2 -g -Wall -Wextra -Wpedantic -Werror -pthread \
    -I "$sdk_dir/include" -isystem /usr/include/libusb-1.0 \
    odin1-local-mapping.cpp odin1-local-mapping-support.cpp \
    "$sdk_dir/lib/liblydHostApi_amd.a" -lusb-1.0 -o odin1-local-mapping
sha256sum "${inputs[@]}" odin1-local-mapping > current-program.sha256
# 标准输出单独返回构建目录，供后续测试和人工启动复用。
printf '%s\n' "$output"
