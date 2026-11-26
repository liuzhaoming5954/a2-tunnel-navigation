"""通过 systemd 的只读属性及守护放行标志，观察 PC2 独立停车守护。"""

from pathlib import Path
import re
import stat
import subprocess
from threading import Event, Lock, Thread
import time


SOURCE = "pc2_systemd"
MAX_AGE_MS = 2500
_INTERVAL_S = 1.0
_TIMEOUT_S = 0.45
_UNIT = "a2-remote-stop.service"
_READY = Path("/run/a2-remote-stop/ready")
_ARMED_TEXT = "遥控停车已布防，等待指定按键"
_LATCHED_PREFIX = "停车已锁存："


def _ready_marker():
    """普通账号可查看 root 管理的运行标志；拒绝把权限错误、符号链接当作放行。"""
    try:
        parent = _READY.parent.lstat()
        if not stat.S_ISDIR(parent.st_mode) or parent.st_uid != 0 or parent.st_mode & 0o022:
            return None
        marker = _READY.lstat()
    except FileNotFoundError:
        return False
    except OSError:
        return None
    if not stat.S_ISREG(marker.st_mode) or marker.st_uid != 0 or marker.st_mode & 0o022:
        return None
    return True


def _read_guard():
    # 此固定调用没有控制操作，不读取仅 root 可访问的持久锁存目录，也不需要 sudo。
    props = "LoadState,ActiveState,SubState,MainPID,InvocationID,StatusText"
    try:
        result = subprocess.run(
            ["/usr/bin/systemctl", "--system", "show", _UNIT, "--property=" + props, "--no-pager"],
            capture_output=True, check=True, timeout=_TIMEOUT_S, encoding="utf-8",
        )
        fields = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
        if fields.get("LoadState") != "loaded":
            return "unknown", None
        active, sub, pid = fields["ActiveState"], fields["SubState"], int(fields["MainPID"])
        if active in ("inactive", "failed") and pid == 0:
            return "stopped", None
        invocation = fields["InvocationID"]
        if (active != "active" or sub != "running" or pid <= 0
                or not re.fullmatch(r"[a-fA-F0-9]{32}", invocation) or int(invocation, 16) == 0):
            return "unknown", None
        status = fields.get("StatusText", "")
        ready = _ready_marker()
        # 守护先撤 ready、持久化锁存，再发布中文锁存状态。两份证据相矛盾时保留未知。
        if status.startswith(_LATCHED_PREFIX) and ready is False:
            return "running", True
        if status == _ARMED_TEXT and ready is True:
            return "running", False
        return "running", None
    except (OSError, ValueError, KeyError, subprocess.SubprocessError):
        return "unknown", None


def blocked_reason(sample):
    """真实底盘只接受本机守护明确放行的新鲜证据；停车请求不使用本判定。"""
    if not isinstance(sample, dict) or sample.get("source") != SOURCE:
        return "stop_guard_not_ready"
    age = sample.get("age_ms")
    if (type(age) is not int or not 0 <= age < MAX_AGE_MS or sample.get("max_age_ms") != MAX_AGE_MS):
        return "stop_guard_not_ready"
    if sample.get("latched") is True:
        return "stop_latched"
    if sample.get("guard") == "running" and sample.get("latched") is False:
        return None
    return "stop_guard_not_ready"


def reset_blocked_reason(report):
    """复位独立于发令门禁，只接受运行中守护的新鲜锁存证据。"""
    blocked = blocked_reason(report)
    if blocked == "stop_latched" and report.get("guard") == "running":
        return None
    if (isinstance(report, dict) and report.get("source") == SOURCE
            and type(report.get("age_ms")) is int and report["age_ms"] >= MAX_AGE_MS):
        return "stop_guard_stale"
    if not isinstance(report, dict) or report.get("guard") != "running":
        return "stop_guard_not_ready"
    return blocked or "stop_not_latched"


class RemoteStopStatus:
    """定频独立取样，让 systemd 阻塞或多个网页并发轮询时不拖慢本体遥测。"""

    def __init__(self):
        self._sample = (time.monotonic(), "unknown", None)
        self._lock = Lock()
        self._done = Event()
        self._worker = Thread(target=self._run, name="remote-stop-telemetry", daemon=True)

    def start(self):
        self._worker.start()

    def close(self):
        self._done.set()
        self._worker.join(timeout=_TIMEOUT_S + 1.)

    def _run(self):
        while not self._done.is_set():
            started = time.monotonic()
            guard, latched = _read_guard()
            with self._lock:
                self._sample = started, guard, latched
            self._done.wait(_INTERVAL_S)

    def snapshot(self):
        with self._lock:
            started, guard, latched = self._sample
        age = max(0, int((time.monotonic() - started) * 1000))
        if age >= MAX_AGE_MS:
            guard, latched = "unknown", None
        elif latched is False and _ready_marker() is not True:
            # 缓存仍显示布防时逐次检查放行标志，刚刚锁存不能借旧的放行快照申请会话。
            latched = None
        return {"source": SOURCE, "guard": guard, "latched": latched, "age_ms": age, "max_age_ms": MAX_AGE_MS}
