"""用合成遥控帧、停车回执和临时文件验证 PC2 独立停车程序。"""

from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
from threading import Event, Thread
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import a2_remote_stop as stop


def ignore_report(event, **fields):
    """测试直接观察底盘和任务管理器，不依赖中文日志的具体措辞。"""


class RemoteInputTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.button = 0x0100
        self.guard = stop.GuardState(self.button, clock=lambda: self.now)

    def test_accepts_only_one_configured_bit_in_the_16_bit_remote_word(self):
        for mask in (0, -1, 0x30, 0x10000):
            with self.subTest(mask=mask), self.assertRaises(ValueError):
                stop.GuardState(mask)
        for mask in (0x0001, 0x8000):
            with self.subTest(mask=mask):
                guard = stop.GuardState(mask)
                guard.keys(mask)
                self.assertTrue(guard.triggered.is_set())

    def test_first_frame_with_held_button_trips_even_with_other_keys(self):
        self.guard.keys(self.button | 0x8000)
        self.assertTrue(self.guard.triggered.is_set())
        self.assertEqual(self.guard.trigger_at, self.now)
        self.assertFalse(self.guard.ready_input())

    def test_frozen_handset_frame_can_stop_but_cannot_keep_guard_ready(self):
        # 底盘序号冻结时仍然响应指定键，其他键的重复数据不能延长输入有效期。
        self.guard.keys(self.button, fresh=False)
        self.assertTrue(self.guard.triggered.is_set())
        other = stop.GuardState(self.button, clock=lambda: self.now)
        other.keys(0)
        for _ in range(4):
            self.now += 0.1
            other.keys(0x8000, fresh=False)
        self.assertTrue(other.triggered.is_set())
        self.assertFalse(other.ready_input())

    def test_release_and_repeated_press_keep_original_stop_time_and_reason(self):
        self.assertFalse(self.guard.ready_input())
        self.guard.keys(0)
        self.assertTrue(self.guard.ready_input())
        self.guard.keys(0x4000)
        self.assertFalse(self.guard.triggered.is_set())
        self.now += 0.1
        self.guard.keys(self.button)
        first_time, first_reason = self.guard.trigger_at, self.guard.reason
        self.assertTrue(first_reason)
        self.now += 0.1
        self.guard.keys(0)
        self.guard.keys(self.button)
        self.guard.trip("第二个停车原因")
        self.assertTrue(self.guard.triggered.is_set())
        self.assertFalse(self.guard.ready_input())
        self.assertEqual((self.guard.trigger_at, self.guard.reason), (first_time, first_reason))

    def test_startup_grace_then_missing_first_frame_latches(self):
        self.now += 2.9
        self.guard.check_remote()
        self.assertFalse(self.guard.ready_input())
        self.assertFalse(self.guard.triggered.is_set())
        self.now += 0.2
        self.guard.check_remote()
        self.assertTrue(self.guard.triggered.is_set())
        self.guard.keys(0)
        self.assertFalse(self.guard.ready_input())

    def test_new_remote_frames_refresh_deadline_but_do_not_clear_link_loss(self):
        self.guard.keys(0)
        for _ in range(4):
            self.now += 0.3
            self.guard.check_remote()
            self.assertFalse(self.guard.triggered.is_set())
            self.guard.keys(0)
        self.now += 0.36
        self.guard.check_remote()
        self.assertTrue(self.guard.triggered.is_set())
        self.guard.keys(0)
        self.assertFalse(self.guard.ready_input())

    def test_recovered_frame_cannot_hide_gap_if_watchdog_has_not_run_yet(self):
        self.guard.keys(0)
        self.now += 0.36
        # 故意不调用 check_remote()，让新的中立帧与监视线程竞争。
        self.guard.keys(0)
        self.assertTrue(self.guard.triggered.is_set())
        self.assertFalse(self.guard.ready_input())

    def test_first_frame_window_begins_when_sdk_is_listening(self):
        self.now += 5
        self.guard.listening()
        self.guard.check_remote()
        self.assertFalse(self.guard.triggered.is_set())
        self.now += 2.0
        self.guard.keys(0)
        self.assertTrue(self.guard.ready_input())


class StopActionTest(unittest.TestCase):
    def setUp(self):
        self.guard = stop.GuardState(0x0080)
        self.guard.keys(0x0080)
        self.unit_down = Event()
        self.shutdown = Event()

    def test_brakes_while_termination_is_blocked_and_after_writer_has_exited(self):
        # 一条线程模拟任务管理器阻塞回执。另一条应当已经在调用底盘停车。
        request_entered = Event()
        finish_request = Event()
        first_brake = Event()
        after_exit = []
        manager = Mock()

        def request_stop():
            request_entered.set()
            if not finish_request.wait(2):
                raise TimeoutError("模拟任务管理器未收到结束信号")

        manager.request_stop.side_effect = request_stop
        manager.inactive.side_effect = finish_request.is_set
        hardware = Mock()

        def stop_move():
            after_exit.append(self.unit_down.is_set())
            first_brake.set()
            return 0

        hardware.stop_move.side_effect = stop_move
        hardware.still_since.return_value = True

        with ThreadPoolExecutor(max_workers=2) as pool:
            terminate = pool.submit(stop.stop_autonomy, self.guard, manager, self.unit_down,
                                    self.shutdown, ignore_report, retry_s=0.005)
            brake = pool.submit(stop.brake_until_still, self.guard, hardware, self.unit_down,
                                self.shutdown, ignore_report, retry_s=0.005)
            try:
                self.assertTrue(request_entered.wait(1))
                self.assertTrue(first_brake.wait(1))
                self.assertFalse(self.unit_down.is_set())
                self.assertFalse(brake.done())
                finish_request.set()
                terminate.result(timeout=1)
                self.assertTrue(self.unit_down.is_set())
                self.assertTrue(brake.result(timeout=1))
            finally:
                finish_request.set()
                self.shutdown.set()
        self.assertFalse(after_exit[0])
        self.assertIn(True, after_exit)

    def test_lost_or_rejected_rpc_and_lack_of_still_feedback_require_retry(self):
        self.unit_down.set()
        emitted = []
        replies = iter((TimeoutError("模拟回执丢失"), 17, 0, 0))

        def stop_move():
            code = next(replies, 0)
            emitted.append(code)
            if isinstance(code, Exception):
                raise code
            return code

        hardware = Mock()
        hardware.stop_move.side_effect = stop_move
        # 只有收到第四次停车命令后才提供持续停稳证据。
        hardware.still_since.side_effect = lambda stamp: len(emitted) >= 4
        with ThreadPoolExecutor(max_workers=1) as pool:
            job = pool.submit(stop.brake_until_still, self.guard, hardware, self.unit_down,
                              self.shutdown, ignore_report, retry_s=0.005)
            try:
                self.assertTrue(job.result(timeout=1))
            finally:
                self.shutdown.set()
        self.assertGreaterEqual(len(emitted), 4)
        self.assertTrue(self.guard.triggered.is_set())

    def test_persistent_rejection_cannot_be_accepted_even_with_still_feedback(self):
        self.unit_down.set()
        retry_observed = Event()
        hardware = Mock()

        def rejected():
            if hardware.stop_move.call_count >= 2:
                retry_observed.set()
            return 19

        hardware.stop_move.side_effect = rejected
        hardware.still_since.return_value = True
        with ThreadPoolExecutor(max_workers=1) as pool:
            job = pool.submit(stop.brake_until_still, self.guard, hardware, self.unit_down,
                              self.shutdown, ignore_report, retry_s=0.005)
            try:
                self.assertTrue(retry_observed.wait(1))
                self.assertFalse(job.done())
            finally:
                self.shutdown.set()
            self.assertFalse(job.result(timeout=1))

    def test_active_writer_is_not_reported_inactive_after_failed_termination(self):
        manager = Mock()
        requests = []

        def request_stop():
            requests.append(True)
            if len(requests) == 1:
                raise OSError("模拟任务管理器暂时不可用")

        manager.request_stop.side_effect = request_stop
        # 接到第二次请求后仍可能在退出。检查两次活动状态，然后才返回已退出。
        manager.inactive.side_effect = lambda: len(requests) >= 2 and manager.inactive.call_count >= 3
        with ThreadPoolExecutor(max_workers=1) as pool:
            job = pool.submit(stop.stop_autonomy, self.guard, manager, self.unit_down,
                              self.shutdown, ignore_report, retry_s=0.005)
            try:
                job.result(timeout=1)
            finally:
                self.shutdown.set()
        self.assertGreaterEqual(len(requests), 2)
        self.assertGreaterEqual(manager.inactive.call_count, 3)
        self.assertTrue(self.unit_down.is_set())

    def test_restarted_writer_is_stopped_again_after_admission_closes(self):
        # 第一次 stop 已让控制组变空，此时另一条 start 作业仍可以使用旧的 ready。
        # 控制第一次重试的时序，重现“已停 → 再启动 → 撤销许可 → 再停”的竞态。
        closed, writer, between_attempts, resume = Event(), Event(), Event(), Event()
        writer.set()
        manager = Mock()
        stops_after_close = []

        def request_stop():
            stops_after_close.append(closed.is_set())
            writer.clear()

        def wait_for_retry(_):
            between_attempts.set()
            resume.wait(2)

        manager.request_stop.side_effect = request_stop
        manager.inactive.side_effect = lambda: not writer.is_set()
        with patch.object(self.shutdown, "wait", side_effect=wait_for_retry), ThreadPoolExecutor(max_workers=1) as pool:
            job = pool.submit(stop.stop_autonomy, self.guard, manager, self.unit_down,
                              self.shutdown, ignore_report, admission_closed=closed)
            try:
                self.assertTrue(between_attempts.wait(1))
                self.assertFalse(self.unit_down.is_set())
                self.assertFalse(writer.is_set())
                writer.set()
                closed.set()
                resume.set()
                job.result(timeout=1)
            finally:
                resume.set()
                self.shutdown.set()
        self.assertEqual(stops_after_close, [False, True])
        self.assertFalse(writer.is_set())
        self.assertTrue(self.unit_down.is_set())


class OperatorResetTest(unittest.TestCase):
    def test_second_guard_and_reset_are_excluded_while_owner_is_running(self):
        with TemporaryDirectory() as tmp:
            files = stop.GuardFiles(Path(tmp) / "state", Path(tmp) / "run")
            with files.acquire():
                files.latch.touch()
                with self.assertRaises(RuntimeError), files.acquire():
                    self.fail("正在运行的守护应持有唯一的文件锁")
                manager = Mock()
                manager.inactive.return_value = True
                with self.assertRaises(RuntimeError):
                    stop.reset_latch(files, manager)
                self.assertTrue(files.latch.exists())

    def test_latch_survives_restart_and_only_resets_with_inactive_unit(self):
        with TemporaryDirectory() as tmp:
            state_dir = Path(tmp) / "state"
            run_dir = Path(tmp) / "run"
            state_dir.mkdir()
            run_dir.mkdir()
            original = stop.GuardFiles(state_dir, run_dir)
            original.latch.touch()
            original.unclean.touch()
            original.sender_fault.touch()
            # 重启后的文件对象指向上一次触发留下的锁存，两次构造之间没有共享内存。
            reopened = stop.GuardFiles(state_dir, run_dir)
            self.assertTrue(reopened.latch.exists())
            self.assertTrue(reopened.unclean.exists())
            with (patch.object(stop, "STATE_DIR", state_dir), patch.object(stop, "RUN_DIR", run_dir),
                  patch.object(stop, "AutonomyUnit") as unit_class, patch("builtins.print")):
                manager = unit_class.return_value
                manager.inactive.return_value = False
                with self.assertRaises(RuntimeError):
                    stop.main(["reset"])
                self.assertTrue(reopened.latch.exists())
                self.assertTrue(reopened.sender_fault.exists())
                self.assertTrue(reopened.unclean.exists())
                manager.inactive.return_value = True
                self.assertEqual(stop.main(["reset"]), 0)
                self.assertEqual([name for name, args, kwargs in manager.mock_calls], ["inactive", "inactive"])
            self.assertFalse(reopened.latch.exists())
            self.assertFalse(reopened.unclean.exists())
            self.assertFalse(reopened.ready.exists())
            self.assertFalse(reopened.sender_fault.exists())


class SenderExitNotificationTest(unittest.TestCase):
    def test_distinguishes_unexpected_exit_from_planned_stop_and_no_sender(self):
        cases = (
            ("强杀退出", "signal", 100, 250, 251, True),
            ("退出码为零但未发起停机", "success", 100, 250, 251, True),
            ("有主动停机顺序且进程正常退出", "success", 100, 250, 240, False),
            ("主动停机但清理失败", "timeout", 100, 250, 240, True),
            ("无法确认退出顺序", "success", 100, 250, 250, True),
            ("没有创建发令主进程", "exit-code", 0, 0, 0, False),
        )
        for label, result, started, exited, stopped, should_latch in cases:
            with self.subTest(label=label), TemporaryDirectory() as tmp:
                files = stop.GuardFiles(Path(tmp) / "state", Path(tmp) / "run")
                files.run_dir.mkdir()
                files.ready.touch()
                manager = stop.AutonomyUnit()
                output = (f"ExecMainStartTimestampMonotonic={started}\n"
                          f"ExecMainExitTimestampMonotonic={exited}\n"
                          f"ActiveExitTimestampMonotonic={stopped}\n")
                with patch.object(manager, "command", return_value=SimpleNamespace(stdout=output)):
                    stop.notify_sender_exit(files, manager, result)
                self.assertEqual(files.sender_fault.exists(), should_latch)

    def test_failure_to_query_manager_marks_fault_only_while_armed(self):
        with TemporaryDirectory() as tmp:
            files = stop.GuardFiles(Path(tmp) / "state", Path(tmp) / "run")
            files.run_dir.mkdir()
            manager = stop.AutonomyUnit()
            with patch.object(manager, "command", side_effect=subprocess.TimeoutExpired("systemctl", 1)) as query:
                stop.notify_sender_exit(files, manager, "signal")
                query.assert_not_called()
                self.assertFalse(files.sender_fault.exists())
                files.ready.touch()
                with patch("builtins.print"):
                    stop.notify_sender_exit(files, manager, "signal")
                self.assertTrue(files.sender_fault.exists())


class GuardEntryTest(unittest.TestCase):
    def test_killed_mock_sender_is_parked_by_guard_and_stays_latched(self):
        # 真正强杀本机的模拟发令子进程。测试替代 systemd 的退出派发与底盘 SDK，
        # 模拟底盘会保持上一条速度，直到独立守护实际调用停车接口。
        with TemporaryDirectory() as tmp:
            files = stop.GuardFiles(Path(tmp) / "state", Path(tmp) / "run")
            args = SimpleNamespace(button_mask=0x0100, interface="mock0", domain=12)
            exiting, armed, moving, stationary = Event(), Event(), Event(), Event()
            timestamps = {}
            events = []
            hardware = Mock()

            def hardware_class(interface, domain, keys, trip):
                def stream_neutral_frames():
                    while not exiting.is_set():
                        keys(0)
                        exiting.wait(0.02)

                hardware.start.side_effect = lambda: Thread(target=stream_neutral_frames, daemon=True).start()

                def stop_move():
                    moving.clear()
                    return 0

                hardware.stop_move.side_effect = stop_move
                hardware.still_since.side_effect = lambda since: not moving.is_set() and time.monotonic() - since >= 0.5
                return hardware

            def systemctl(argv, **kwargs):
                status = ("failed" if "exited" in timestamps else "inactive")
                if argv[2] == "show":
                    output = (f"LoadState=loaded\nActiveState={status}\nControlGroup=\n"
                              f"ExecMainStartTimestampMonotonic={timestamps.get('started', 0)}\n"
                              f"ExecMainExitTimestampMonotonic={timestamps.get('exited', 0)}\n"
                              f"ActiveExitTimestampMonotonic={timestamps.get('exited', 0) + 1}\n")
                    return subprocess.CompletedProcess(argv, 0, output, "")
                return subprocess.CompletedProcess(argv, 0, "", "")

            def report(event, **fields):
                events.append((event, fields))
                if event == "armed":
                    armed.set()
                if event == "stationary_feedback":
                    stationary.set()

            with (patch.object(stop.subprocess, "run", side_effect=systemctl),
                  patch.object(stop, "notify"), ThreadPoolExecutor(max_workers=1) as pool):
                job = pool.submit(stop.guard, args, files, stop.AutonomyUnit(), hardware_class, exiting, report)
                try:
                    self.assertTrue(armed.wait(2))
                    self.assertEqual(hardware.stop_move.call_count, 0)
                    timestamps["started"] = time.monotonic_ns() // 1000
                    # 只在子进程发出模拟运动输入后才强杀，并使模拟底盘保持最后的输入。
                    program = "import signal; print('moving', flush=True); signal.pause()"
                    with subprocess.Popen([sys.executable, "-c", program], stdout=subprocess.PIPE, text=True) as sender:
                        try:
                            readable, _, _ = select.select([sender.stdout], [], [], 2)
                            self.assertTrue(readable)
                            self.assertEqual(sender.stdout.readline().strip(), "moving")
                            moving.set()
                            sender.kill()
                            self.assertEqual(sender.wait(timeout=2), -signal.SIGKILL)
                        finally:
                            if sender.poll() is None:
                                sender.kill()
                    timestamps["exited"] = time.monotonic_ns() // 1000
                    self.assertTrue(moving.is_set())
                    self.assertEqual(hardware.stop_move.call_count, 0)
                    with (patch.object(stop, "STATE_DIR", files.state_dir),
                          patch.object(stop, "RUN_DIR", files.run_dir), patch.dict(os.environ, SERVICE_RESULT="signal")):
                        self.assertEqual(stop.main(["notify-sender-exit"]), 0)
                    self.assertTrue(stationary.wait(3))
                    self.assertFalse(moving.is_set())
                    self.assertFalse(files.ready.exists())
                    self.assertTrue(files.sender_fault.exists())
                    self.assertEqual(files.latch.read_text().strip(), stop.SENDER_EXIT_REASON)
                    self.assertGreaterEqual(hardware.stop_move.call_count, 2)
                    self.assertTrue(any(event == "stop_reply" and row["after_exit"] for event, row in events))
                finally:
                    exiting.set()
                self.assertEqual(job.result(timeout=4), 0)

    def test_button_in_running_guard_brakes_exits_unit_and_stays_latched_on_restart(self):
        # 用真正的 guard、文件锁、AutonomyUnit 和两个执行线程，只替换 SDK 与 systemctl。
        # 第一轮从解锁状态开始。第二轮复用同一持久目录，但更换 SDK 实例与所有线程。
        with TemporaryDirectory() as tmp:
            files = stop.GuardFiles(Path(tmp) / "persist", Path(tmp) / "runtime")
            args = SimpleNamespace(button_mask=0x0100, interface="mock0", domain=12)
            commands = []
            active = Event()

            def systemctl(argv, **kwargs):
                commands.append(argv)
                action = argv[2]
                if action == "show":
                    status = "active" if active.is_set() else "inactive"
                    properties = f"LoadState=loaded\nActiveState={status}\nControlGroup=\n"
                    return subprocess.CompletedProcess(argv, 0, properties, "")
                if action == "kill":
                    active.clear()
                return subprocess.CompletedProcess(argv, 0, "", "")

            with (patch.object(stop.subprocess, "run", side_effect=systemctl),
                  patch.object(stop, "notify"), ThreadPoolExecutor(max_workers=1) as pool):
                for restart in (False, True):
                    exiting = Event()
                    armed = Event()
                    latched = Event()
                    stationary = Event()
                    events = {"armed": armed, "latched": latched, "stationary_feedback": stationary}
                    reasons = []
                    hardware = Mock()
                    hardware.stop_move.return_value = 0
                    hardware.still_since.return_value = True
                    remote = {}

                    def hardware_class(interface, domain, keys, trip):
                        remote["keys"] = keys
                        # 提供一帧松开按键的有效遥控数据，模拟启动时底盘静止。
                        hardware.start.side_effect = lambda: keys(0)
                        return hardware

                    def report(event, **fields):
                        if event == "latched":
                            reasons.append(fields["reason"])
                        if event in events:
                            events[event].set()

                    job = pool.submit(stop.guard, args, files, stop.AutonomyUnit(),
                                      hardware_class, exiting, report)
                    try:
                        if not restart:
                            self.assertTrue(armed.wait(2))
                            self.assertTrue(files.ready.is_file())
                            self.assertFalse(files.latch.exists())
                            self.assertEqual(hardware.stop_move.call_count, 0)
                            # 模拟一个已经通过 systemd 准入并写底盘的自主进程。
                            active.set()
                            remote["keys"](args.button_mask | 1)
                            remote["keys"](0)
                        self.assertTrue(latched.wait(2))
                        self.assertTrue(stationary.wait(2))
                        self.assertFalse(active.is_set())
                        self.assertFalse(files.ready.exists())
                        self.assertTrue(files.latch.is_file())
                        self.assertTrue(files.unclean.is_file())
                        self.assertGreaterEqual(hardware.stop_move.call_count, 2)
                        if restart:
                            self.assertFalse(armed.is_set())
                            self.assertIn("恢复", reasons[0])
                            self.assertIn("0x0100", files.latch.read_text())
                    finally:
                        exiting.set()
                    self.assertEqual(job.result(timeout=4), 0)
                    hardware.close.assert_called_once()
            self.assertIn("kill", [cmd[2] for cmd in commands])
            self.assertTrue(all(cmd[-1] == stop.UNIT for cmd in commands))
            self.assertTrue(all(cmd[2] in {"stop", "show", "kill"} for cmd in commands))


if __name__ == "__main__":
    unittest.main()
