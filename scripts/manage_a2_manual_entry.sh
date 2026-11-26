#!/bin/bash
# 在 PC2 安装或移除本版本的受管遥控入口。全程保留停车锁存，只通知 systemd 重读配置。
set -euo pipefail

fail() { echo "$*" >&2; exit 1; }
[[ $# == 1 && ( $1 == install || $1 == remove ) ]] || fail "用法：$0 install|remove"
(( EUID == 0 )) || fail "应在 PC2 使用管理员账号安装"
release=$(dirname -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")")
[[ $release == /opt/a2/* ]] || fail "请先将校验通过的固定版本发布包安装至 /opt/a2 下"

# 维护事务共用现有配置目录锁。两个普通账号都不能改写受管入口、版本目录或服务增补。
exec 9< /etc/a2
flock -x 9
entry=/etc/a2/autonomy-start
dropdir=/etc/systemd/system/a2-autonomy.service.d
drop=$dropdir/40-cockpit-manual-receiver.conf
launcher=$release/scripts/start_a2_manual_receiver.sh
template=$release/deploy/templates/a2-manual-receiver.conf
latched=/var/lib/a2-remote-stop/latched
ready=/run/a2-remote-stop/ready
[[ -f $latched && ! -L $latched && ! -e $ready && ! -L $ready ]] || fail "须保持停车锁存且无放行标志"
systemctl is-active --quiet a2-remote-stop.service || fail "停车守护未运行"
[[ $(systemctl show -p ActiveState --value a2-autonomy.service) == inactive ]] || fail "受管运动服务需要处于未运行状态"

check_target() {
  # 允许重复安装同一固定版本。已有不同版本、试验脚本或失效的其他链接时均原样保留。
  [[ ! -e $1 && ! -L $1 ]] || [[ -L $1 && $(readlink -- "$1") == "$2" ]] ||
    fail "已有其他现场配置，保留 $1"
}
check_target "$entry" "$launcher"
check_target "$drop" "$template"
if [[ $1 == remove ]]; then
  for target in "$entry" "$drop"; do
    if [[ -L $target ]]; then unlink -- "$target"; fi
  done
  systemctl daemon-reload
  exit 0
fi

trusted_path() {
  # 检查目标和每级父目录，避免通过一个可替换的中间目录注入启动文件。
  local path=$1
  while [[ $path != / ]]; do
    [[ -e $path && ! -L $path && $(stat -c %u "$path") == 0 &&
       -z $(find "$path" -maxdepth 0 -perm /022 -print) ]] || fail "路径须由 root 管理且仅属主可写：$path"
    path=$(dirname -- "$path")
  done
}
autonomy_uid=$(id -u a2-autonomy) || fail "缺少受管运动服务的专用普通账号 a2-autonomy"
[[ $autonomy_uid != 0 && $autonomy_uid != "$(id -u unitree)" ]] || fail "两个服务须使用独立的普通账号"
[[ " $(id -Gn unitree) " == *" unitree "* ]] || fail "网关账号缺少套接字访问组 unitree"
[[ -x $launcher && -f $template ]] || fail "发布包须包含可执行启动入口和服务模板"
trusted_path "$release"
trusted_path /etc/a2
trusted_path /etc/a2/remote-stop.env
trusted_path /etc/systemd/system
[[ -z $(find "$release" \( ! -user root -o -perm /022 -o \( ! -type f ! -type d \) \) -print -quit) ]] ||
  fail "发布树须由 root 管理且仅属主可写，包内应全是普通文件或目录"
[[ -d $dropdir ]] || mkdir -m 0755 -- "$dropdir"
trusted_path "$dropdir"

# 最后公布可执行入口。若第二个链接或 daemon-reload 失败，仅回滚本次新建的链接。
created=()
recover() {
  local status=$?
  if (( status != 0 )); then
    for target in "${created[@]}"; do unlink -- "$target"; done
    systemctl daemon-reload || :
  fi
}
trap recover EXIT
if [[ ! -L $drop ]]; then ln -sT -- "$template" "$drop"; created+=("$drop"); fi
if [[ ! -L $entry ]]; then ln -sT -- "$launcher" "$entry"; created+=("$entry"); fi
systemctl daemon-reload
echo "已安装固定版本受管遥控入口，停车锁存有效，运动服务未启动。"
