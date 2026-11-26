#!/usr/bin/env python3
"""PC2 独立停车。守护处理遥控按键和受管发令进程的异常退出。"""

import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
from queue import Full, Queue
import signal
import socket
import subprocess
from threading import Event, Lock, Thread
import time


UNIT = "a2-autonomy.service"
STATE_DIR = Path("/var/lib/a2-remote-stop")
RUN_DIR = Path("/run/a2-remote-stop")
REMOTE_MAX_AGE = 0.35
FIRST_FRAME_TIMEOUT = 3.0
SENDER_EXIT_REASON = "受管发令进程意外退出或退出状态无法核实"


def single_button(mask):
    """现场选择一个 16 位按键位。多键同时按下仍可匹配这个位。"""
    if type(mask) is not int or not 0 < mask <= 0xffff or mask & (mask - 1):
        raise ValueError("停车键必须是单个按键位，例如从实物读到的单比特十六进制值")
    return mask


class GuardState:
    """每一帧直接检查停车位。第一次触发即锁存，不需要等待松手或轮询下一帧。"""

    def __init__(self, mask, clock=time.monotonic, *, initializing=False):
        self.mask = single_button(mask)
        self.clock = clock
        self.started = None if initializing else clock()
        self.last_frame = None
        self.trigger_at = None
        self.reason = ""
        self.triggered = Event()
        self.lock = Lock()

    def _trip_locked(self, reason, now):
        if not self.triggered.is_set():
            self.trigger_at, self.reason = now, reason
            self.triggered.set()

    def trip(self, reason):
        with self.lock:
            self._trip_locked(reason, self.clock())

    def listening(self):
        # 首帧等待从订阅可用时计算。SDK 版本查询可能耗时，不应消耗遥控输入窗口。
        with self.lock:
            if self.last_frame is None:
                self.started = self.clock()

    def keys(self, value, *, fresh=True):
        with self.lock:
            now = self.clock()
            if type(value) is not int or not 0 <= value <= 0xffff:
                self._trip_locked("遥控按键帧无效", now)
                return
            if value & self.mask:
                self._trip_locked(f"遥控停车键 0x{self.mask:04x}", now)
            # 即便看门线程此时未获调度，迟到的恢复帧也不能掩盖中断。
            self._check_age(now)
            # 重复的底盘帧即使携带停车键也立即执行，但不把重复帧计作新遥控观测。
            if fresh:
                self.last_frame = now

    def _check_age(self, now):
        if self.last_frame is None:
            if self.started is not None and now - self.started >= FIRST_FRAME_TIMEOUT:
                self._trip_locked("启动后未收到遥控帧", now)
        elif now - self.last_frame >= REMOTE_MAX_AGE:
            self._trip_locked("遥控数据中断", now)

    def check_remote(self):
        with self.lock:
            self._check_age(self.clock())

    def ready_input(self):
        with self.lock:
            return (not self.triggered.is_set() and self.last_frame is not None
                    and self.clock() - self.last_frame < REMOTE_MAX_AGE)


class GuardFiles:
    """持久目录保留触发及异常退出标志，运行目录的 ready 只供自主服务启动时检查。"""

    def __init__(self, state_dir, run_dir):
        self.state_dir, self.run_dir = Path(state_dir), Path(run_dir)
        self.latch = self.state_dir / "latched"
        self.unclean = self.state_dir / "guard-active"
        self.ready = self.run_dir / "ready"
        self.sender_fault = self.run_dir / "sender-fault"

    @contextmanager
    def acquire(self):
        self.state_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.run_dir.mkdir(mode=0o755, parents=True, exist_ok=True)
        # 永不删除锁文件。删除一个仍被持有的 inode 会允许第二个进程锁住同名新文件。
        with (self.state_dir / "owner.lock").open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("停车守护正在运行，拒绝重复启动或复位") from exc
            yield

    def sync_dir(self):
        fd = os.open(self.state_dir, os.O_DIRECTORY | os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def persist(self, path, reason):
        # 判据是标志文件的存在。先落盘运行标志才允许自主启动，异常断电亦需人工复位。
        try:
            with path.open("x", encoding="utf-8") as out:
                out.write(reason + "\n")
                out.flush()
                os.fsync(out.fileno())
        except FileExistsError:
            pass
        self.sync_dir()


class AutonomyUnit:
    """只操作项目专用控制组。既不匹配进程名，也不执行现场传来的 shell 命令。"""

    @staticmethod
    def command(*args, check=True):
        result = subprocess.run(["systemctl", "--system", *args, UNIT], capture_output=True,
                                text=True, timeout=1.0, check=False)
        if check and result.returncode:
            raise RuntimeError(result.stderr.strip() or f"systemctl 返回 {result.returncode}")
        return result

    def properties(self, *names):
        args = [f"--property={name}" for name in names]
        result = self.command("show", "--all", *args)
        return dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)

    def unexpected_exit(self, service_result):
        # systemd 先记录主进程退出，再把意外退出的单元移出 active。主动停止则先离开
        # active，再收割主进程。只凭退出码 0 或 Result=success 会漏报异常退出。
        props = self.properties("ExecMainStartTimestampMonotonic", "ExecMainExitTimestampMonotonic",
                                "ActiveExitTimestampMonotonic")
        started = int(props["ExecMainStartTimestampMonotonic"])
        if not started:
            # 启动条件不满足或 ExecStartPre 失败时没有发令进程。
            return False
        exited = int(props["ExecMainExitTimestampMonotonic"])
        stopped = int(props["ActiveExitTimestampMonotonic"])
        return service_result != "success" or not started <= stopped < exited

    def request_stop(self):
        # 先登记停止作业，阻止已有启动作业继续；再中止仍在退出的发送者，避免其在宽限期续发速度。
        self.command("stop", "--no-block")
        if not self.inactive():
            self.command("kill", "--kill-who=all", "--signal=SIGKILL", check=False)

    def inactive(self):
        # 退出后 ControlGroup 可能为空，显式请求空属性，避免和管理器查询不完整混淆。
        props = self.properties("LoadState", "ActiveState", "ControlGroup")
        if props.get("LoadState") != "loaded":
            raise RuntimeError(f"无法核实已安装的 {UNIT}")
        if props.get("ActiveState") not in ("inactive", "failed") or "ControlGroup" not in props:
            return False
        # systemd 有时在单元转为 failed 后仍保留控制组；检查内核中的子孙，不能把 failed 当作退出证明。
        group = props["ControlGroup"]
        if not group:
            return True
        if not group.startswith("/") or Path(group).name != UNIT or ".." in Path(group).parts:
            raise RuntimeError("自主服务返回的控制组路径异常")
        root = Path("/sys/fs/cgroup")
        if not (root / "cgroup.controllers").exists():
            root /= "systemd"
        if not root.is_dir():
            raise RuntimeError("无法读取 systemd 控制组")
        folder = root / group.lstrip("/")
        if not folder.exists():
            return True
        def on_error(exc):
            if not isinstance(exc, FileNotFoundError):
                raise exc
        for name, _, _ in os.walk(folder, onerror=on_error):
            try:
                if (Path(name) / "cgroup.procs").read_text().strip():
                    return False
            except FileNotFoundError:
                if Path(name).exists():
                    return False
        return True


def notify_sender_exit(files, manager, service_result):
    """由 systemd 在发令进程之外调用；只通知守护，不在退出钩子内调用底盘 SDK。"""
    if not files.ready.is_file():
        return
    try:
        if not manager.unexpected_exit(service_result):
            return
    except (KeyError, OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        print(f"受管发令退出状态查询失败：{exc}", flush=True)
    # ExecStopPost 结束前登记故障。单元的启动条件也检查该标志，防止守护落盘锁存前重启。
    # 运行目录由停车守护的系统单元管理，发令账号不能修改。故障标志不由退出进程清理。
    files.sender_fault.touch(mode=0o600)


def reset_latch(files, manager):
    """仅在守护退出、受管发送者全部退出后复位。删除标志不会启动任何程序。"""
    with files.acquire():
        if not manager.inactive():
            raise RuntimeError("自主服务仍在运行，不能复位")
        files.ready.unlink(missing_ok=True)
        files.sender_fault.unlink(missing_ok=True)
        files.unclean.unlink(missing_ok=True)
        files.latch.unlink(missing_ok=True)
        files.sync_dir()


def stop_autonomy(state, manager, unit_down, shutdown, report, retry_s=.25, admission_closed=None):
    """独立线程终止自主服务；管理器超时或故障不阻塞底盘停车线程。"""
    state.triggered.wait()
    while not shutdown.is_set():
        try:
            # 先停一次，不等磁盘。最后一次停单元必须发生在撤销启动许可之后，
            # 以覆盖按键与撤销 ready 文件之间已经由 systemd 放行的启动作业。
            fenced = admission_closed is None or admission_closed.is_set()
            manager.request_stop()
            if fenced and manager.inactive():
                report("autonomy_exited", unit=UNIT)
                unit_down.set()
                return
        except Exception as exc:
            report("autonomy_stop_error", error=str(exc))
        shutdown.wait(retry_s)


def brake_until_still(state, hardware, unit_down, shutdown, report, retry_s=.25):
    """首次立即请求停车。发送者退出后再请求一次，随后用两路新鲜反馈观测停稳。"""
    state.triggered.wait()
    first = True
    final_ack_at = None
    while not shutdown.is_set():
        # 在 RPC 前取值，使退出前已在途的 RPC 即便稍后成功也不能算作最终停车。
        after_exit = unit_down.is_set() and not first
        stamp = time.monotonic()
        accepted = False
        try:
            code = hardware.stop_move()
            report("stop_reply", code=code, started=stamp, after_exit=after_exit)
            accepted = type(code) is int and code == 0
            if after_exit and accepted and final_ack_at is None:
                final_ack_at = time.monotonic()
        except Exception as exc:
            report("stop_rpc_error", error=str(exc), started=stamp)
        first = False
        if accepted and final_ack_at is not None:
            try:
                if hardware.still_since(final_ack_at):
                    report("stationary_feedback", since=final_ack_at)
                    return True
            except Exception as exc:
                report("feedback_error", error=str(exc))
        # 发送者刚刚退出时立即补发，无需等到下一个重试周期。
        if after_exit:
            shutdown.wait(retry_s)
        else:
            unit_down.wait(retry_s)
    return False


def notify(message):
    address = os.environ.get("NOTIFY_SOCKET")
    if address:
        if address.startswith("@"):
            address = "\0" + address[1:]
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.settimeout(.2)
            sock.sendto(message.encode(), address)


def guard(args, files, manager, hardware_class, exiting, report):
    state = GuardState(args.button_mask, initializing=True)
    unit_down, shutdown, admission_closed, confirmed = Event(), Event(), Event(), Event()
    # SDK 初始化也可能阻塞。先撤去旧的启动许可，保留上次崩溃/重启的锁存证据。
    with files.acquire():
        files.ready.unlink(missing_ok=True)
        if files.latch.exists() or files.unclean.exists():
            state.trip("恢复已有停车锁存或守护异常退出记录")
        files.persist(files.unclean, "守护已启动，重新布防前请现场核对")
        hardware = hardware_class(args.interface, args.domain, state.keys, state.trip)
        terminate = Thread(target=stop_autonomy, args=(state, manager, unit_down, shutdown, report),
                           kwargs={"admission_closed": admission_closed}, daemon=True)
        def run_brake():
            if brake_until_still(state, hardware, unit_down, shutdown, report):
                confirmed.set()

        brake = Thread(target=run_brake, daemon=True)
        terminate.start()
        started = announced = persisted = False
        try:
            hardware.start()
            state.listening()
            started = True
            brake.start()
            while not exiting.is_set():
                state.check_remote()
                if files.sender_fault.is_file():
                    state.trip(SENDER_EXIT_REASON)
                if state.triggered.is_set():
                    if not persisted:
                        files.ready.unlink(missing_ok=True)
                        admission_closed.set()
                        files.persist(files.latch, state.reason)
                        report("latched", reason=state.reason, triggered_at=state.trigger_at)
                        notify("STATUS=停车已锁存：" + state.reason)
                        persisted = True
                elif not announced and state.ready_input():
                    # 自主单元可能已有排队的 start 作业。先检查单元定义，
                    # 发送者的启动顺序由 After 和 ready 条件共同约束。
                    if "LoadState=loaded" not in manager.command("show", "-p", "LoadState").stdout.splitlines():
                        raise RuntimeError(f"请先安装 {UNIT}")
                    if state.ready_input():
                        files.ready.touch(exist_ok=False)
                        report("armed", button=f"0x{state.mask:04x}", remote="rt/lf/lowstate")
                        notify("STATUS=遥控停车已布防，等待指定按键")
                if not announced and (files.ready.exists() or persisted):
                    notify("READY=1")
                    announced = True
                if announced:
                    notify("WATCHDOG=1")
                exiting.wait(.05)
        finally:
            # 即使终端关闭、初始化失败或 systemd 正常停止，也撤销许可、停车并保留锁存。
            state.trip("停车守护正在退出")
            try:
                files.ready.unlink(missing_ok=True)
                admission_closed.set()
                files.persist(files.latch, state.reason)
            finally:
                # 正常监听期间停车可持续重试。进程被要求退出时留有限时间用于最终停车与观测。
                deadline = time.monotonic() + 3.
                terminate.join(max(0., deadline - time.monotonic()))
                if started:
                    brake.join(max(0., deadline - time.monotonic()))
                shutdown.set()
                if not confirmed.is_set():
                    report("stop_unconfirmed_at_exit", unit_exited=unit_down.is_set())
                if started and brake.is_alive():
                    brake.join(.5)
                if not brake.is_alive():
                    hardware.close()
    return 0 if confirmed.is_set() else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    observe = actions.add_parser("observe", help="只读打印实物按键变化")
    armed = actions.add_parser("guard", help="监听指定按键并保护受管自主程序")
    actions.add_parser("reset", help="现场确认已停稳并停止守护后，清除锁存")
    actions.add_parser("notify-sender-exit", help="由 systemd 退出钩子通知发令进程故障")
    for part in (observe, armed):
        part.add_argument("--interface", required=True, help="PC2 底盘 DDS 网卡")
        part.add_argument("--domain", type=int, default=0)
    armed.add_argument("--button-mask", type=lambda value: single_button(int(value, 0)), required=True)
    args = parser.parse_args(argv)
    if hasattr(args, "domain") and not 0 <= args.domain <= 232:
        parser.error("DDS 域必须在 0 到 232 之间")
    files, manager = GuardFiles(STATE_DIR, RUN_DIR), AutonomyUnit()
    if args.action == "notify-sender-exit":
        notify_sender_exit(files, manager, os.environ.get("SERVICE_RESULT"))
        return 0
    if args.action == "reset":
        reset_latch(files, manager)
        print("停车锁存已复位。守护与自主程序须分别启动。", flush=True)
        return 0

    from a2_remote_stop_sdk import StopHardware
    exiting = Event()
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, lambda *_: exiting.set())
    if args.action == "observe":
        last = None
        def display(keys):
            nonlocal last
            if keys != last:
                print(f"按键位 0x{keys:04x}", flush=True)
                last = keys
        hw = StopHardware(args.interface, args.domain, display,
                          lambda error: print(f"遥控帧无效：{error}", flush=True), observe_only=True)
        try:
            hw.start()
            print("正在只读识键。单独按下并松开指定按钮，Ctrl+C 退出。", flush=True)
            exiting.wait()
        finally:
            hw.close()
        return 0

    # 日志写入与两个执行线程隔离，终端显示慢时仍能发送停车与终止请求。
    messages = Queue(maxsize=256)
    def report(event, **fields):
        try:
            messages.put_nowait({"event": event, "monotonic_s": time.monotonic(), **fields})
        except Full:
            pass

    def write_logs():
        while True:
            row = messages.get()
            if row is None:
                break
            print(json.dumps(row, ensure_ascii=False), flush=True)
    logger = Thread(target=write_logs, daemon=True)
    logger.start()
    try:
        return guard(args, files, manager, StopHardware, exiting, report)
    finally:
        try:
            messages.put_nowait(None)
        except Full:
            pass
        logger.join(.2)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as exc:
        raise SystemExit(f"停车程序错误：{exc}")
