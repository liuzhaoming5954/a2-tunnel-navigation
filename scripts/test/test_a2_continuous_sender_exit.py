"""用本机模拟底盘验证长按运动中的发令进程强制退出。"""

from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
from tempfile import TemporaryDirectory
from threading import Event, Thread
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import a2_remote_stop as stop


def request(endpoint, operation, **fields):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(0.5)
        client.connect(str(endpoint))
        message = dict(schema_version=1, op=operation, **fields)
        client.sendall(json.dumps(message).encode() + b"\n")
        with client.makefile("rb") as stream:
            return json.loads(stream.readline(4096))


def publish_hold(path, pressed):
    # 独立采样线程使用本机单调时钟。刷新同一按压周期时不更换标识。
    row = {"pressed": pressed, "epoch": "hold-forward" if pressed else "neutral",
           "direction": "forward" if pressed else None, "observed_mono_ns": time.monotonic_ns()}
    pending = path.with_suffix(".pending")
    pending.write_text(json.dumps(row), encoding="ascii")
    # 开发机可能默认给组写入权限，接收器会拒绝这种输入文件。
    pending.chmod(0o600)
    pending.replace(path)


class Chassis:
    """接收器退出后，模拟底盘还保留最后一次运动状态。"""

    def __init__(self):
        self.moving = Event()
        self.moves = []
        self.sender_stops = []
        self.guard_stops = []

    def receive(self, stream):
        for line in stream:
            row = json.loads(line)
            if row["event"] == "move":
                self.moves.append(row)
                self.moving.set()
            elif row["event"] == "stop":
                self.sender_stops.append(row)
                self.moving.clear()

    def guard_stop_move(self):
        self.guard_stops.append(time.monotonic())
        self.moving.clear()
        return 0

    def still_since(self, since):
        return not self.moving.is_set() and time.monotonic() - since >= 0.50


@contextmanager
def receiver(endpoint, hold, latch, chassis, work):
    argv = [sys.executable, "-I", "-S", "-B", str(ROOT / "scripts/run_a2_manual_receiver.py"),
            "--backend", "mock", "--socket", str(endpoint), "--mock-hold-file", str(hold),
            "--mock-keyboard-hold", "--mock-stop-file", str(latch), "--continuous-hold"]
    with (work / "receiver-errors.log").open("w+") as errors, ThreadPoolExecutor(max_workers=1) as output:
        process = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=errors, text=True)
        reading = output.submit(chassis.receive, process.stdout)
        try:
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    break
                try:
                    ready = request(endpoint, "status")["ok"] is True
                except (FileNotFoundError, ConnectionRefusedError):
                    ready = False
                if ready:
                    yield process
                    return
                time.sleep(0.01)
            errors.seek(0)
            raise RuntimeError(f"模拟发令进程未就绪 {errors.read().strip()}")
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            reading.result(timeout=2)
            process.stdout.close()


def systemd_conditions(files):
    # 只映射生产单元的三个驻留文件条件，真实 systemd 与生产目录均不接触。
    unit = (ROOT / "deploy/templates/a2-autonomy.service").read_text()
    mapping = {
        "/var/lib/a2-remote-stop/latched": files.latch,
        "/run/a2-remote-stop/sender-fault": files.sender_fault,
        "/run/a2-remote-stop/ready": files.ready,
    }
    conditions = []
    for line in unit.splitlines():
        if line.startswith("ConditionPathExists="):
            value = line.removeprefix("ConditionPathExists=")
            name = value.removeprefix("!")
            if name in mapping:
                conditions.append((mapping[name], not value.startswith("!")))
    if len(conditions) != len(mapping):
        raise RuntimeError("发令单元驻留文件条件缺失")
    return conditions


class ContinuousSenderExitTest(unittest.TestCase):
    def test_killed_sender_after_thirty_seconds_stops_and_cannot_restart_old_session(self):
        with TemporaryDirectory(prefix="a2-sender-") as temp, ThreadPoolExecutor(max_workers=2) as tasks:
            work = Path(temp)
            files = stop.GuardFiles(work / "state", work / "run")
            endpoint = work / "manual.sock"
            hold = work / "keyboard.json"
            publish_hold(hold, False)
            pressed, exiting, armed, confirmed = Event(), Event(), Event(), Event()
            chassis = Chassis()
            conditions = systemd_conditions(files)
            current = {"process": None, "started_us": 0, "exited_us": 0}
            guard_events = []

            def stream_keyboard():
                while not exiting.is_set():
                    publish_hold(hold, pressed.is_set())
                    exiting.wait(0.04)

            def hardware_class(_interface, _domain, keys, _trip):
                finished = Event()

                def stream_handset():
                    while not finished.is_set():
                        keys(0)
                        finished.wait(0.02)

                handset = Thread(target=stream_handset, daemon=True)

                def close():
                    finished.set()
                    handset.join(1)

                return SimpleNamespace(start=handset.start, close=close, stop_move=chassis.guard_stop_move,
                                       still_since=chassis.still_since)

            def manager_command(action, *_args, check=True):
                if action == "show":
                    process = current["process"]
                    active = process is not None and process.poll() is None
                    state = "active" if active else "failed" if current["exited_us"] else "inactive"
                    exited = current["exited_us"]
                    properties = (f"LoadState=loaded\nActiveState={state}\nControlGroup=\n"
                                  f"ExecMainStartTimestampMonotonic={current['started_us']}\n"
                                  f"ExecMainExitTimestampMonotonic={exited}\n"
                                  f"ActiveExitTimestampMonotonic={exited + 1}\n")
                    return subprocess.CompletedProcess(action, 0, properties, "")
                if action == "start":
                    allowed = all(path.exists() == present for path, present in conditions)
                    return subprocess.CompletedProcess(action, 0 if allowed else 1, "", "")
                if action in ("stop", "kill"):
                    return subprocess.CompletedProcess(action, 0, "", "")
                raise AssertionError(f"出现未经模拟的系统管理器操作 {action}")

            def report(event, **fields):
                guard_events.append((event, fields))
                if event == "armed":
                    armed.set()
                if event == "stationary_feedback":
                    confirmed.set()

            manager = stop.AutonomyUnit()
            args = SimpleNamespace(button_mask=0x0100, interface="mock0", domain=0)
            with patch.object(manager, "command", side_effect=manager_command), patch.object(stop, "notify"):
                keyboard = tasks.submit(stream_keyboard)
                guard = tasks.submit(stop.guard, args, files, manager, hardware_class, exiting, report)
                try:
                    self.assertTrue(armed.wait(3))
                    self.assertEqual(manager.command("start").returncode, 0)
                    with receiver(endpoint, hold, files.latch, chassis, work) as process:
                        current.update(process=process, started_us=time.monotonic_ns() // 1_000)
                        session = request(endpoint, "acquire")
                        self.assertIs(session.get("continuous_hold"), True, session)
                        self.assertNotIn("expires_in_ms", session)
                        pressed.set()
                        # 动作起步前确保独立发布线程已写入按下样本。
                        deadline = time.monotonic() + 0.3
                        while not json.loads(hold.read_text())["pressed"] and time.monotonic() < deadline:
                            time.sleep(0.005)
                        self.assertTrue(json.loads(hold.read_text())["pressed"])
                        packet = dict(session_id=session["session_id"], sequence=1,
                                      linear_mps=0.20, angular_radps=0.0)
                        first = time.monotonic()
                        while time.monotonic() - first < 31.0:
                            result = request(endpoint, "velocity", **packet)
                            self.assertIs(result["ok"], True, result)
                            packet["sequence"] += 1
                            time.sleep(0.05)
                        self.assertTrue(chassis.moving.is_set())
                        self.assertGreater(chassis.moves[-1]["monotonic_s"] - chassis.moves[0]["monotonic_s"], 30)
                        self.assertTrue(all(row["linear_mps"] == 0.20 and row["angular_radps"] == 0.0
                                            for row in chassis.moves))
                        self.assertFalse(chassis.sender_stops)
                        self.assertFalse(chassis.guard_stops)
                        # 只强杀本测试自己创建的模拟发令进程。模拟底盘还在动，由独立守护发停。
                        killed_at = time.monotonic()
                        process.kill()
                        self.assertEqual(process.wait(timeout=2), -signal.SIGKILL)
                        current["exited_us"] = time.monotonic_ns() // 1_000
                        self.assertTrue(chassis.moving.is_set())
                        self.assertFalse(chassis.guard_stops)
                        stop.notify_sender_exit(files, manager, "signal")
                        self.assertTrue(confirmed.wait(3))
                        self.assertFalse(chassis.moving.is_set())
                        self.assertLess(chassis.guard_stops[0] - killed_at, 1.0)
                        self.assertFalse(chassis.sender_stops)
                        self.assertTrue(any(label == "stop_reply" and value["after_exit"] and value["code"] == 0
                                            for label, value in guard_events))
                    self.assertEqual(files.latch.read_text().strip(), stop.SENDER_EXIT_REASON)
                    self.assertTrue(files.sender_fault.exists())
                    self.assertFalse(files.ready.exists())
                    self.assertNotEqual(manager.command("start", check=False).returncode, 0)
                    old_count = len(chassis.moves)
                    # 绕过模拟启动门禁另起测试进程，直接验证重连时旧会话帧被锁存拒绝。
                    with receiver(endpoint, hold, files.latch, chassis, work):
                        self.assertEqual(request(endpoint, "velocity", **packet),
                                         {"ok": False, "reason": "stop_latched"})
                        self.assertEqual(request(endpoint, "acquire"), {"ok": False, "reason": "stop_latched"})
                    # 用独立空锁存路径模拟未来的干净启动，原有守护锁存仍保留。
                    with receiver(endpoint, hold, work / "fresh-mock-latch", chassis, work):
                        self.assertEqual(request(endpoint, "velocity", **packet),
                                         {"ok": False, "reason": "invalid_session"})
                        self.assertIs(request(endpoint, "acquire")["ok"], False)
                    self.assertTrue(files.latch.is_file())
                    self.assertEqual(len(chassis.moves), old_count)
                    self.assertFalse(chassis.moving.is_set())
                finally:
                    exiting.set()
                keyboard.result(timeout=2)
                self.assertEqual(guard.result(timeout=4), 0)


if __name__ == "__main__":
    unittest.main()
