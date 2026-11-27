"""通过真实回环网关和模拟接收器，验证跨过 30 秒后的停车及旧会话淘汰。"""

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, contextmanager
from http.client import HTTPConnection
import json
from pathlib import Path
import secrets
import subprocess
import sys
from tempfile import TemporaryDirectory
from threading import Event, Lock, Thread
import time
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/a2_gateway"))

from a2_gateway.motion import LocalMotionClient
from a2_gateway.server import GatewayServer
from a2_gateway.station import StationSource


class KeyboardPublisher:
    """键盘采样线程独立于 HTTP 发令，静默故障保留最后一帧供 PC2 判定过期。"""

    def __init__(self, path):
        self.path = path
        self.lock = Lock()
        self.sample = (False, "neutral-before-session", None)
        self.finished = Event()
        self.wake = Event()
        self.published = Event()
        self.thread = Thread(target=self.run, daemon=True)
        self.thread.start()

    def publish(self, pressed, epoch, direction="forward"):
        with self.lock:
            self.published.clear()
            self.sample = None if pressed is None else (pressed, epoch, direction if pressed else None)
            self.wake.set()
        if not self.published.wait(1):
            raise RuntimeError("独立模拟键盘未更新")

    def run(self):
        while not self.finished.is_set():
            with self.lock:
                self.wake.clear()
                if self.sample is not None:
                    pressed, epoch, direction = self.sample
                    staging = self.path.with_suffix(".next")
                    staging.write_text(json.dumps({"pressed": pressed, "epoch": epoch,
                                                   "direction": direction,
                                                   "observed_mono_ns": time.monotonic_ns()}))
                    staging.chmod(0o600)
                    staging.replace(self.path)
                self.published.set()
            self.wake.wait(.035)

    def close(self):
        self.finished.set()
        self.wake.set()
        self.thread.join(1)


class MockTelemetry:
    def __init__(self, receiver):
        self.receiver = receiver
        self.lock = Lock()
        self.events = []
        self.thread = Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        for line in self.receiver.stdout:
            with self.lock:
                self.events.append(json.loads(line))

    def snapshot(self):
        return {"robot": {"id": "A2-MOCK-CONTINUOUS", "online": self.receiver.poll() is None,
                          "telemetry_stamp_ms": time.time_ns() // 1_000_000, "telemetry_age_ms": 0}}

    def close(self):
        # 两级 HTTP 网关都引用同一组遥测，拆除控制监听时保持接收器与观测线程存活。
        pass

    def observed(self):
        with self.lock:
            return list(self.events)


@contextmanager
def serve(telemetry, *, role, manual, key, port=0):
    with GatewayServer(("127.0.0.1", port), telemetry, mode="manual_control", role=role,
                       manual_control=manual, manual_key=key) as server:
        thread = Thread(target=server.serve_forever, kwargs={"poll_interval": .02}, daemon=True)
        thread.start()
        try:
            yield server.server_port
        finally:
            server.shutdown()
            thread.join(1)


def request(port, path, payload=None):
    connection = HTTPConnection("127.0.0.1", port, timeout=1.5)
    try:
        connection.request("GET" if payload is None else "POST", path,
                           body=None if payload is None else json.dumps(payload),
                           headers={"Origin": f"http://127.0.0.1:{port}",
                                    "Content-Type": "application/json"})
        reply = connection.getresponse()
        return reply.status, json.loads(reply.read())
    finally:
        connection.close()


def stop_receiver(receiver):
    if receiver.poll() is None:
        receiver.terminate()
        try:
            receiver.wait(2)
        except subprocess.TimeoutExpired:
            receiver.kill()
            receiver.wait(2)


class ContinuousHoldReconnectTest(unittest.TestCase):
    def test_reusable_session_moves_each_direction_then_closes(self):
        with TemporaryDirectory(prefix="a2-reusable-mock-") as name, ExitStack() as stack:
            sock, key_file = Path(name) / "mock.sock", Path(name) / "keyboard.json"
            keyboard = KeyboardPublisher(key_file)
            stack.callback(keyboard.close)
            receiver = stack.enter_context(subprocess.Popen(
                [sys.executable, "-ISBu", str(ROOT / "scripts/run_a2_manual_receiver.py"),
                 "--backend", "mock", "--socket", str(sock), "--mock-keyboard-hold",
                 "--mock-hold-file", str(key_file), "--continuous-hold"],
                stdout=subprocess.PIPE, text=True,
            ))
            stack.callback(stop_receiver, receiver)
            deadline = time.monotonic() + 4
            while not sock.is_socket() and receiver.poll() is None and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(sock.is_socket(), "模拟接收器未就绪")
            telemetry = MockTelemetry(receiver)
            key = secrets.token_urlsafe(32)
            local = LocalMotionClient(sock, require_source=True)
            pc2 = stack.enter_context(serve(telemetry, role="pc2", manual=local, key=key))
            station = StationSource(f"http://127.0.0.1:{pc2}", manual_key=key)
            browser = stack.enter_context(serve(station, role="station", manual=station, key=key))
            self.assertIs(request(browser, "/api/state")[1]["control"]["reusable_hold"], True)
            status, session = request(browser, "/api/control/session", {"reusable_hold": True})
            self.assertEqual(status, 200, session)
            self.assertIs(session["continuous_hold"] and session["reusable_hold"], True)
            session_id = session["session_id"]
            sequence = 0
            last_action = None

            def command(path, **fields):
                nonlocal sequence
                sequence += 1
                return request(browser, "/api/control/" + path,
                               {"session_id": session_id, "sequence": sequence, **fields})

            for direction, linear, angular in (
                    ("forward", .2, 0.), ("backward", -.2, 0.), ("left", 0., .15), ("right", 0., -.15)):
                keyboard.publish(True, f"press-{direction}", direction)
                status, action = command("action/start", direction=direction)
                self.assertEqual(status, 200, action)
                self.assertEqual(action, {"ok": True, "action_id": sequence})
                if last_action is not None:
                    # 即使旧速度包使用更大的序号，上一段的动作编号也不能用于新一段。
                    self.assertEqual(command("velocity", action_id=last_action, linear_mps=linear,
                                             angular_radps=angular)[1]["reason"], "invalid_action")
                last_action = action["action_id"]
                self.assertEqual(command("velocity", action_id=last_action, linear_mps=linear,
                                         angular_radps=angular)[0], 200)
                if direction in ("forward", "left"):
                    keyboard.publish(False, f"released-{direction}")
                self.assertEqual(command("action/release", action_id=last_action)[0], 200)
                if direction in ("backward", "right"):
                    keyboard.publish(False, f"released-{direction}")
                deadline = time.monotonic() + .6
                while time.monotonic() < deadline:
                    self.assertEqual(command("velocity", linear_mps=0., angular_radps=0.)[0], 200)
                    control = request(browser, "/api/state")[1]["control"]
                    self.assertIs(control["armed"], True, control)
                    if control["action_state"] == "ready":
                        break
                    time.sleep(.025)
                else:
                    self.fail(f"{direction} 松键后未回到已启用的待命状态")

            status, reply = request(browser, "/api/control/stop", {"session_id": session_id})
            self.assertEqual((status, reply), (200, {"ok": True}))
            self.assertEqual(command("velocity", linear_mps=0., angular_radps=0.)[0], 409)
            self.assertEqual(local.velocity({"session_id": session_id, "sequence": sequence + 1,
                                             "linear_mps": 0., "angular_radps": 0.})[1]["reason"],
                             "invalid_session")
            self.assertFalse(local.state()["armed"])
            self.assertEqual([(event["event"], event["linear_mps"], event["angular_radps"])
                              for event in telemetry.observed() if event["reason"] != "operator_stop"],
                             [("move", .2, 0.), ("stop", 0., 0.), ("move", -.2, 0.), ("stop", 0., 0.),
                              ("move", 0., .15), ("stop", 0., 0.), ("move", 0., -.15), ("stop", 0., 0.)])

    def exercise(self, fault, expected_reason):
        with TemporaryDirectory(prefix="a2-continuous-mock-") as name, ExitStack() as stack, ExitStack() as control:
            work = Path(name)
            sock, key_file = work / "mock.sock", work / "keyboard.json"
            keyboard = KeyboardPublisher(key_file)
            stack.callback(keyboard.close)
            receiver = stack.enter_context(subprocess.Popen(
                [sys.executable, "-ISBu", str(ROOT / "scripts/run_a2_manual_receiver.py"),
                 "--backend", "mock", "--socket", str(sock), "--mock-keyboard-hold",
                 "--mock-hold-file", str(key_file), "--continuous-hold"],
                stdout=subprocess.PIPE, text=True,
            ))
            stack.callback(stop_receiver, receiver)
            deadline = time.monotonic() + 4
            while not sock.is_socket() and receiver.poll() is None and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(sock.is_socket(), f"{fault} 模拟接收器未就绪")
            telemetry = MockTelemetry(receiver)
            local = LocalMotionClient(sock, require_source=True)
            key = secrets.token_urlsafe(32)
            pc2 = control.enter_context(serve(telemetry, role="pc2", manual=local, key=key))
            station = StationSource(f"http://127.0.0.1:{pc2}", manual_key=key)
            browser = stack.enter_context(serve(station, role="station", manual=station, key=key))
            response, session = request(browser, "/api/control/session", {})
            self.assertEqual(response, 200, (fault, session))
            self.assertEqual(set(session), {"ok", "session_id", "continuous_hold"})
            self.assertIs(session["continuous_hold"], True)
            keyboard.publish(True, "same-forward-press")
            sequence = 0

            def velocity():
                nonlocal sequence
                sequence += 1
                return request(browser, "/api/control/velocity", {
                    "session_id": session["session_id"], "sequence": sequence,
                    "linear_mps": .20, "angular_radps": 0.,
                })

            started = time.monotonic()
            while True:
                status, reply = velocity()
                self.assertEqual((status, reply), (200, {"ok": True}), (fault, sequence, reply))
                if time.monotonic() - started >= 31:
                    break
                time.sleep(.085)
            outputs_before_fault = telemetry.observed()
            self.assertTrue(outputs_before_fault, fault)
            self.assertTrue(all(row["event"] == "move" and row["linear_mps"] == .20
                                for row in outputs_before_fault), (fault, outputs_before_fault[-3:]))
            motion = outputs_before_fault[-1]["monotonic_s"] - outputs_before_fault[0]["monotonic_s"]
            self.assertGreater(motion, 30, fault)
            fault_at = time.monotonic()

            if fault == "release":
                keyboard.publish(False, "physical-release")
            elif fault == "keyboard":
                keyboard.publish(None, None)
            else:
                # 关闭站端至模拟 PC2 的真实 TCP 监听。实体键采样和 PC2 接收器继续运行。
                control.close()
                self.assertGreaterEqual(velocity()[0], 400)
            deadline = fault_at + 1.5
            while time.monotonic() < deadline:
                if fault != "control":
                    # 模拟网页漏收松键或不了解独立采样断流，仍持续发新序号的同方向速度包。
                    velocity()
                stops = [row for row in telemetry.observed() if row["event"] == "stop"]
                if stops:
                    break
                time.sleep(.055)
            else:
                self.fail(f"{fault} 模拟底盘未在故障后停车")
            stop = stops[0]
            self.assertEqual(stop["reason"], expected_reason)
            self.assertEqual((stop["linear_mps"], stop["angular_radps"]), (0., 0.))
            self.assertIsNone(receiver.poll())
            if fault == "control":
                control.enter_context(serve(telemetry, role="pc2", manual=local, key=key, port=pc2))
            # 恢复实体输入和 HTTP 后，旧会话的速度重发与重新启用都必须被拒绝。
            keyboard.publish(False, "neutral-after-fault")
            keyboard.publish(True, "new-press-after-fault")
            for _ in range(3):
                self.assertGreaterEqual(velocity()[0], 400, fault)
            self.assertEqual(local.acquire({})[0], 409, fault)
            self.assertFalse(local.state()["armed"])
            self.assertTrue(all(row["event"] != "move" for row in telemetry.observed()
                                if row["monotonic_s"] >= stop["monotonic_s"]), fault)
            print(f"{fault}: 同次按住行进 {motion:.2f} s，故障后 "
                  f"{stop['monotonic_s'] - fault_at:.3f} s 停车（{expected_reason}），重连旧会话无运动")

    def test_motion_crosses_30_seconds_then_each_loss_revokes_and_stops(self):
        cases = (("release", "operator_hold_released"), ("keyboard", "operator_hold_stale"),
                 ("control", "deadman_expired"))
        with ThreadPoolExecutor(max_workers=len(cases)) as pool:
            futures = [(case, pool.submit(self.exercise, *case)) for case in cases]
            for (fault, _), future in futures:
                with self.subTest(fault=fault):
                    future.result()
