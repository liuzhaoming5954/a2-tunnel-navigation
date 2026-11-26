#!/usr/bin/env python3
"""PC2 系统服务专用的停车锁存复位入口。"""

import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import pwd
import re
import socket
import stat
import struct
import subprocess
import sys
import time

from a2_remote_stop import AutonomyUnit, GuardFiles, RUN_DIR, STATE_DIR, reset_latch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/a2_safety"))
from manual_authority import HANDSET_SOURCE, _distinct_fields, read_owner
from manual_grant import identity


RESET_DIR = Path("/run/a2-stop-reset")
GUARD_UNIT = "a2-remote-stop.service"
GATEWAY_USER = "unitree"
GATEWAY_CGROUP = "/system.slice/a2-cockpit-manual-gateway.service"
NEUTRAL = "handset_neutral_release_unconfirmed"
ID_FORMAT = re.compile(r"[0-9a-f]{32}")
BOOT_FORMAT = re.compile(r"[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}")
POLL_S = 0.04
START_S = 3.0
STOP_S = 5.5
FEEDBACK_S = 1.8


class Refused(Exception):
    """可安全返回给驾驶舱的状态代码。"""


def reply(reason, *, ok=False, available=None):
    result = {"ok": ok, "reason": reason}
    if available is not None:
        result["available"] = available
    return result


def foreign_or_missing(path, kind):
    """验证固定运行路径的所有者与类型。缺失与现存的非可信路径分别处理。"""
    try:
        meta = path.lstat()
    except FileNotFoundError:
        return True
    if meta.st_uid != os.geteuid() or meta.st_mode & 0o022 or not kind(meta.st_mode):
        raise PermissionError(f"运行路径不可信：{path}")
    return False


class Transaction:
    """独占复位和补偿，持久意图用于覆盖跨开机的异常退出。"""

    def __init__(self, files, runtime=RESET_DIR):
        self.runtime = runtime
        self.inflight = runtime / "inflight.json"
        self.pending = files.state_dir / "reset-pending.json"
        self.files = files

    @contextmanager
    def locked(self, *, wait=False):
        if foreign_or_missing(self.runtime, stat.S_ISDIR):
            raise FileNotFoundError("缺少系统管理的复位运行目录")
        fd = os.open(self.runtime / "transaction.lock",
                     os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
        try:
            meta = os.fstat(fd)
            if not stat.S_ISREG(meta.st_mode) or meta.st_uid != os.geteuid() or meta.st_mode & 0o022:
                raise PermissionError("复位事务锁不可信")
            fcntl.flock(fd, fcntl.LOCK_EX | (0 if wait else fcntl.LOCK_NB))
            yield
        finally:
            os.close(fd)

    def has_intent(self):
        return any(not foreign_or_missing(path, stat.S_ISREG)
                   for path in (self.inflight, self.pending))

    @staticmethod
    def _read(path):
        if foreign_or_missing(path, stat.S_ISREG):
            return None
        try:
            with path.open("rb") as stream:
                data = json.loads(stream.read(256), object_pairs_hook=_distinct_fields)
                if stream.read(1) or not isinstance(data, dict) or set(data) != {"owner", "boot_id"}:
                    raise ValueError("复位意图内容无效")
        except FileNotFoundError:
            # 其他连接的 ExecStopPost 可能正好读到已提交事务被删除的时刻。
            return None
        if (not isinstance(data["owner"], str) or not ID_FORMAT.fullmatch(data["owner"])
                or not isinstance(data["boot_id"], str) or not BOOT_FORMAT.fullmatch(data["boot_id"])):
            raise ValueError("复位意图标识无效")
        return data

    def write(self, owner, boot_id):
        if (not ID_FORMAT.fullmatch(owner) or not BOOT_FORMAT.fullmatch(boot_id)
                or foreign_or_missing(self.files.state_dir, stat.S_ISDIR)):
            raise ValueError("系统服务缺少复位事务身份或状态目录")
        payload = {"owner": owner, "boot_id": boot_id}
        # 持久意图先于停止作业落盘。单独使用 /run 将在 PC2 掉电时丢失这份证据。
        for path in (self.pending, self.inflight):
            tmp = path.with_name(f".{path.name}.{owner}")
            fd = os.open(tmp, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as stream:
                    json.dump(payload, stream, separators=(",", ":"))
                    stream.flush()
                    os.fsync(stream.fileno())
                tmp.replace(path)
                if path == self.pending:
                    self.files.sync_dir()
            finally:
                tmp.unlink(missing_ok=True)

    def belongs_to(self, owner):
        # Accept=yes 的 status 连接也运行 ExecStopPost，只处理当前 INVOCATION_ID 的事务。
        markers = [self._read(path) for path in (self.inflight, self.pending)]
        owners = {item["owner"] for item in markers if item is not None}
        return owners == {owner}

    def clear(self):
        # 成功时只有在守护重新布防后才提交。两个标志全部移除前，自主单元均不可启动。
        self.pending.unlink(missing_ok=True)
        self.files.sync_dir()
        self.inflight.unlink(missing_ok=True)

    def recover_boot(self, boot_id):
        """守护启动前补回旧开机的锁存。同一开机的受管重启仍由复位事务控制。"""
        try:
            marker = self._read(self.pending)
        except (OSError, ValueError, RecursionError):
            self.files.persist(self.files.latch, "跨重启复位意图无法核实")
            return
        if marker is not None and marker["boot_id"] != boot_id:
            self.files.persist(self.files.latch, "PC2 在复位事务完成前重启")
            self.clear()


class GuardService:
    """只允许固定停车守护的查询及生命周期操作。"""

    @staticmethod
    def command(*args, timeout=0.4):
        return subprocess.run(["/usr/bin/systemctl", "--system", *args, GUARD_UNIT],
                              check=True, timeout=timeout, capture_output=True, text=True)

    def state(self, timeout=0.35):
        fields = "LoadState,ActiveState,SubState,MainPID,InvocationID,StatusText"
        result = self.command("show", "--all", f"--property={fields}", timeout=timeout)
        props = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
        if props.get("LoadState") != "loaded":
            raise RuntimeError("无法核实停车守护单元")
        return props

    def stopped(self):
        props = self.state()
        return props["ActiveState"] in ("inactive", "failed") and props["MainPID"] == "0"

    def current_id(self):
        return self.state()["InvocationID"]

    def invocation(self, *, latched, timeout=0.35):
        props = self.state(timeout)
        invocation = props["InvocationID"]
        status = props.get("StatusText", "")
        expected = (status.startswith("停车已锁存：") if latched
                    else status == "遥控停车已布防，等待指定按键")
        if (props["ActiveState"] == "active" and props["SubState"] == "running" and expected
                and int(props["MainPID"]) > 1 and ID_FORMAT.fullmatch(invocation) and int(invocation, 16)):
            return invocation
        return None

    def stop(self):
        self.command("stop", "--no-block")

    def start(self):
        self.command("start", "--no-block")

    def restart(self):
        self.command("restart", "--no-block")


def neutral_sample(report, clock):
    """回中只说明手柄此刻未操作。该状态始终保留原厂控制权未知。"""
    if report.get("source") != HANDSET_SOURCE:
        return False
    if report.get("value") == "owned" and report.get("reason") == "handset_active":
        raise Refused("stop_handset_active")
    stamp = report.get("observed_at")
    return (report.get("value") == "unknown" and report.get("reason") == NEUTRAL
            and type(stamp) in (int, float) and 0 <= clock() - stamp < 0.20
            and type(report.get("frame_count")) is int and report["frame_count"] > 0
            and type(report.get("source_tick")) is int)


class Feedback:
    """沿用守护的两种停稳判据，同时连续监听手柄以覆盖守护重启窗口。"""

    def __init__(self, interface, domain, *, clock=time.monotonic, read=read_owner):
        from a2_remote_stop_sdk import StopHardware
        from observe_a2_handset_owner import HandsetOwner

        class ReadOnlyHardware(StopHardware):
            def _receive_remote(hardware, message):
                self.handset.receive(message)
                super()._receive_remote(message)

        self.handset = HandsetOwner(clock=clock)
        self.hardware = ReadOnlyHardware(interface, domain, None, None, feedback_only=True)
        self.clock = clock
        self.read = read
        self.identity = identity()
        self.pin = None

    def start(self):
        self.hardware.start()

    def close(self):
        self.hardware.close()

    def report(self):
        snapshot = self.handset.report()
        current = (snapshot["producer_instance"], snapshot["generation"])
        present = neutral_sample(snapshot, self.clock)
        if self.pin is not None and (not present or self.pin != current):
            # 短按后回中也会改变代号，不能用新的回中掩盖重启期间的按键事件或中断。
            raise Refused("stop_handset_changed")
        return snapshot if present else None

    def wait_safe(self, since):
        deadline = self.clock() + FEEDBACK_S
        baseline = None
        latest = None
        independent_ok = False
        while self.clock() < deadline:
            latest = self.report()
            independent = self.read(*self.identity)
            independent_ok = neutral_sample(independent, self.clock)
            if latest is not None and independent_ok:
                if baseline is None:
                    baseline = latest["frame_count"]
                elif latest["frame_count"] > baseline and self.hardware.still_since(since):
                    self.pin = (latest["producer_instance"], latest["generation"])
                    # 防止最后一次慢查询使本地手柄样本跨过新鲜期限。
                    self.report()
                    return
            else:
                baseline = None
            time.sleep(POLL_S)
        reason = "stop_not_stationary" if latest is not None and independent_ok else "stop_handset_unavailable"
        raise Refused(reason)

    def require_still(self, since):
        self.report()
        if not self.hardware.still_since(since):
            raise Refused("stop_not_stationary")


def wait_for(condition, limit, check=lambda: None):
    deadline = time.monotonic() + limit
    while time.monotonic() < deadline:
        check()
        if condition():
            return True
        time.sleep(POLL_S)
    return False


class StopReset:
    """复位前核对真实反馈，完成或失败均恢复停车守护。"""

    def __init__(self, transaction, guard, sender, feedback_factory):
        self.tx = transaction
        self.files = transaction.files
        self.guard = guard
        self.sender = sender
        self.feedback_factory = feedback_factory

    def check_latched(self, timeout=0.35):
        if self.tx.has_intent():
            raise Refused("stop_reset_busy")
        if foreign_or_missing(self.files.latch, stat.S_ISREG):
            raise Refused("stop_not_latched")
        invocation = self.guard.invocation(latched=True, timeout=timeout)
        if not invocation or not foreign_or_missing(self.files.ready, stat.S_ISREG):
            raise Refused("stop_guard_not_running")
        return invocation

    def status(self):
        if foreign_or_missing(self.tx.runtime, stat.S_ISDIR):
            return reply("stop_reset_unavailable", available=False)
        try:
            self.check_latched(timeout=0.16)
        except Refused as ex:
            return reply(str(ex), ok=True, available=False)
        return reply("stop_reset_ready", ok=True, available=True)

    def _confirmed(self, *, latched, different_from=None):
        invocation = self.guard.invocation(latched=latched)
        latch_exists = not foreign_or_missing(self.files.latch, stat.S_ISREG)
        ready_exists = not foreign_or_missing(self.files.ready, stat.S_ISREG)
        return (invocation and invocation != different_from and latch_exists == latched
                and ready_exists != latched)

    def _restore(self):
        # 先重新持久锁存并撤销 ready，再安排守护重新读取锁存，覆盖 worker 异常退出。
        self.files.persist(self.files.latch, "上位机复位未完成，恢复停车锁存")
        self.files.ready.unlink(missing_ok=True)
        # 后台开始恢复，ExecStopPost 可以接管本次作业，不会等待已退出的网页。
        prior = self.guard.current_id()
        self.guard.restart()
        if not wait_for(lambda: self._confirmed(latched=True, different_from=prior), START_S):
            return False
        self.tx.clear()
        return True

    def recover(self, owner):
        if not ID_FORMAT.fullmatch(owner):
            return False
        if not self.tx.belongs_to(owner):
            return True
        with self.tx.locked(wait=True):
            return not self.tx.belongs_to(owner) or self._restore()

    def reset(self, owner, boot_id):
        try:
            with self.tx.locked():
                return self._reset_locked(owner, boot_id)
        except BlockingIOError:
            return reply("stop_reset_busy")
        except Refused as ex:
            return reply(str(ex))

    def _reset_locked(self, owner, boot_id):
        self.check_latched()
        if not self.sender.inactive():
            raise Refused("stop_sender_active")
        feedback = self.feedback_factory()
        try:
            feedback.start()
            feedback.wait_safe(time.monotonic())
            invocation = self.check_latched()
            if not self.sender.inactive():
                raise Refused("stop_sender_active")
            self.tx.write(owner, boot_id)
            # 在持久锁存和两个额外启动条件建立后才可停止守护。
            self.guard.stop()
            if not wait_for(self.guard.stopped, STOP_S, feedback.report):
                raise RuntimeError("停车守护退出超时")
            if not self.sender.inactive():
                raise Refused("stop_sender_active")
            after_stop = time.monotonic()
            feedback.wait_safe(after_stop)
            reset_latch(self.files, self.sender)
            feedback.require_still(after_stop)
            self.guard.start()
            if not wait_for(lambda: self._confirmed(latched=False, different_from=invocation),
                            START_S, feedback.report):
                raise RuntimeError("停车守护重新布防超时")
            if not self.sender.inactive():
                raise Refused("stop_sender_active")
            feedback.require_still(after_stop)
            self.tx.clear()
            return reply("stop_reset_done", ok=True)
        except Exception as ex:
            if not self.tx.belongs_to(owner):
                raise
            print(f"停车复位未完成：{ex}", file=sys.stderr)
            reason = str(ex) if isinstance(ex, Refused) else "stop_reset_rolled_back"
            try:
                if self._restore():
                    return reply(reason)
            except (OSError, RuntimeError, ValueError, KeyError, subprocess.SubprocessError) as recovery:
                print(f"停车复位恢复尚未确认：{recovery}", file=sys.stderr)
            return reply("stop_reset_recovery_pending")
        finally:
            try:
                feedback.close()
            except Exception as ex:
                print(f"关闭只读停稳订阅失败：{ex}", file=sys.stderr)


def allowed_peer(conn, user=GATEWAY_USER, group=GATEWAY_CGROUP):
    creds = struct.Struct("3i")
    pid, uid, _ = creds.unpack(conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, creds.size))
    if pid <= 1 or uid == 0 or uid != pwd.getpwnam(user).pw_uid:
        return False
    for row in Path(f"/proc/{pid}/cgroup").read_text().splitlines():
        level, controllers, path = row.split(":", 2)
        if (level == "0" and not controllers) or "name=systemd" in controllers.split(","):
            return path == group
    return False


def read_request(conn):
    deadline = time.monotonic() + 0.12
    data = b""
    while len(data) < 96:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        conn.settimeout(remaining)
        chunk = conn.recv(96 - len(data))
        if not chunk:
            break
        data += chunk
        if b"\n" in data:
            if not data.endswith(b"\n") or data.count(b"\n") != 1:
                break
            request = json.loads(data, object_pairs_hook=_distinct_fields)
            if (isinstance(request, dict) and set(request) == {"op", "schema_version"}
                    and type(request["schema_version"]) is int and request["schema_version"] == 1
                    and request["op"] in ("status", "reset")):
                return request["op"]
            break
    raise ValueError("复位套接字只接受单条有长度上限的版本 1 请求")


def serve(conn, operation, owner, boot_id, *, authorize=allowed_peer):
    # 直接在 systemd 接收的 socket 回复，SDK 打印与 Python 日志留在 journal。
    try:
        if not authorize(conn):
            return
    except (OSError, KeyError, ValueError):
        return
    try:
        action = read_request(conn)
    except (OSError, ValueError, RecursionError):
        result = reply("stop_reset_bad_request")
    else:
        try:
            result = operation.status() if action == "status" else operation.reset(owner, boot_id)
        except Exception as ex:
            print(f"停车复位服务不可用：{ex}", file=sys.stderr)
            result = reply("stop_reset_unavailable")
    try:
        conn.settimeout(0.10)
        conn.sendall(json.dumps(result, separators=(",", ":")).encode() + b"\n")
    except OSError:
        # 网页失联不能撤回已经接收的请求，也不能让恢复依赖连接是否还在。
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    service = commands.add_parser("serve")
    service.add_argument("--interface", required=True)
    service.add_argument("--domain", type=int, default=0, choices=range(233), metavar="0..232")
    commands.add_parser("recover")
    commands.add_parser("recover-boot")
    args = parser.parse_args(argv)
    if os.geteuid() != 0:
        parser.error("仅 PC2 本地 root 系统服务可以运行")
    owner = os.environ.get("INVOCATION_ID", "")
    boot_id = identity()[0]
    tx = Transaction(GuardFiles(STATE_DIR, RUN_DIR))
    if args.action == "recover-boot":
        tx.recover_boot(boot_id)
        return 0
    operation = StopReset(tx, GuardService(), AutonomyUnit(),
                          lambda: Feedback(args.interface, args.domain))
    if args.action == "recover":
        return 0 if operation.recover(owner) else 1
    with socket.socket(fileno=os.dup(sys.stdin.fileno())) as conn:
        serve(conn, operation, owner, boot_id)
    return 0


if __name__ == "__main__":
    sys.exit(main())
