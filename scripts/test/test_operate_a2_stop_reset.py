"""用本机临时文件和合成反馈验证 PC2 锁存复位，不加载机器人 SDK。"""

import json
import os
from pathlib import Path
import struct
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import operate_a2_stop_reset as reset


OWNER = "a" * 32
OTHER = "b" * 32
BOOT = "01234567-0000-0000-0000-0123456789ab"


class FakeGuard:
    """模拟 systemd 在另一次运行实例中重建 ready 或读取持久锁存。"""

    def __init__(self, files):
        self.files = files
        self.phase = "latched"
        self.counter = 1
        self.events = []

    def current_id(self):
        return f"{self.counter:032x}"

    def invocation(self, *, latched, timeout=0.35):
        if self.phase == ("latched" if latched else "ready"):
            return self.current_id()
        return None

    def stop(self):
        self.events.append("stop")
        self.phase = "stopped"

    def stopped(self):
        return self.phase == "stopped"

    def start(self):
        self.events.append("start")
        self.counter += 1
        self.phase = "ready"
        self.files.ready.touch()

    def restart(self):
        self.events.append("restart")
        self.counter += 1
        self.phase = "latched"


class ResetTest(unittest.TestCase):
    def setUp(self):
        self.addCleanup(os.umask, os.umask(0o077))
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        parent = Path(directory.name)
        files = reset.GuardFiles(parent / "state", parent / "guard-runtime")
        with files.acquire():
            files.persist(files.latch, "现场停车")
        runtime = parent / "reset-runtime"
        runtime.mkdir(mode=0o700)
        self.files = files
        self.transaction = reset.Transaction(files, runtime)
        self.guard = FakeGuard(files)
        self.sender = Mock()
        self.sender.inactive.return_value = True
        self.feedback = Mock()
        self.worker = reset.StopReset(self.transaction, self.guard, self.sender, lambda: self.feedback)

    def test_only_stationary_handset_samples_allow_clearing_then_rearming(self):
        self.assertEqual(self.worker.status(), reset.reply("stop_reset_ready", ok=True, available=True))
        real_stop = self.guard.stop
        real_start = self.guard.start

        def stop_after_persisting_both_intents():
            self.assertTrue(self.transaction.inflight.is_file())
            self.assertTrue(self.transaction.pending.is_file())
            self.assertTrue(self.files.latch.is_file())
            real_stop()

        def start_only_after_removing_the_latch_with_motion_still_fenced():
            self.assertFalse(self.files.latch.exists())
            self.assertTrue(self.transaction.has_intent())
            real_start()

        self.guard.stop = stop_after_persisting_both_intents
        self.guard.start = start_only_after_removing_the_latch_with_motion_still_fenced
        self.assertEqual(self.worker.reset(OWNER, BOOT), reset.reply("stop_reset_done", ok=True))
        self.assertEqual(self.guard.events, ["stop", "start"])
        self.assertEqual(self.feedback.wait_safe.call_count, 2)
        self.assertEqual(self.feedback.require_still.call_count, 2)
        self.feedback.close.assert_called_once()
        self.assertFalse(self.files.latch.exists())
        self.assertTrue(self.files.ready.exists())
        self.assertFalse(self.transaction.has_intent())
        self.assertEqual(self.worker.reset(OWNER, BOOT)["reason"], "stop_not_latched")

    def test_no_guard_stop_on_bad_feedback_or_running_sender(self):
        self.feedback.wait_safe.side_effect = reset.Refused("stop_not_stationary")
        self.assertEqual(self.worker.reset(OWNER, BOOT)["reason"], "stop_not_stationary")
        self.sender.inactive.return_value = False
        self.assertEqual(self.worker.reset(OWNER, BOOT)["reason"], "stop_sender_active")
        self.assertEqual(self.guard.events, [])
        self.assertTrue(self.files.latch.exists())
        self.assertFalse(self.transaction.has_intent())

    def test_handset_activity_after_latch_was_deleted_restores_latch(self):
        self.feedback.require_still.side_effect = reset.Refused("stop_handset_changed")
        self.assertEqual(self.worker.reset(OWNER, BOOT)["reason"], "stop_handset_changed")
        self.assertEqual(self.guard.events, ["stop", "restart"])
        self.assertTrue(self.files.latch.exists())
        self.assertFalse(self.files.ready.exists())
        self.assertFalse(self.transaction.has_intent())

    def test_recovery_after_worker_death_is_constrained_to_the_matching_invocation(self):
        with self.transaction.locked():
            self.assertEqual(self.worker.reset(OWNER, BOOT)["reason"], "stop_reset_busy")
            self.transaction.write(OWNER, BOOT)
            self.guard.stop()
            reset.reset_latch(self.files, self.sender)
        self.assertTrue(self.worker.recover(OTHER))
        self.assertFalse(self.files.latch.exists())
        self.assertEqual(self.guard.events, ["stop"])
        self.assertTrue(self.worker.recover(OWNER))
        self.assertTrue(self.files.latch.exists())
        self.assertEqual(self.guard.events, ["stop", "restart"])
        self.assertFalse(self.transaction.has_intent())

    def test_guard_restart_failure_keeps_the_fence_and_the_latch(self):
        self.feedback.require_still.side_effect = RuntimeError("合成采样失败")
        with patch.object(self.guard, "restart", side_effect=OSError("合成启动失败")):
            self.assertEqual(self.worker.reset(OWNER, BOOT)["reason"], "stop_reset_recovery_pending")
        self.assertTrue(self.files.latch.exists())
        self.assertTrue(self.transaction.has_intent())
        self.assertTrue(self.worker.recover(OWNER))
        self.assertFalse(self.transaction.has_intent())

    def test_guard_boot_checks_only_previous_boot_and_recovers_without_runtime_marker(self):
        self.transaction.write(OWNER, BOOT)
        reset.reset_latch(self.files, self.sender)
        self.transaction.recover_boot(BOOT)
        self.assertFalse(self.files.latch.exists())
        self.assertTrue(self.transaction.has_intent())
        self.transaction.inflight.unlink()
        self.transaction.recover_boot("01234567-0000-0000-0000-000000000000")
        self.assertTrue(self.files.latch.exists())
        self.assertFalse(self.transaction.has_intent())


class HandsetTest(unittest.TestCase):
    def test_pinned_generation_rejects_even_a_handset_that_is_neutral_again(self):
        feedback = reset.Feedback.__new__(reset.Feedback)
        feedback.clock = lambda: 10.0
        feedback.pin = ("source", 4)
        data = {"source": reset.HANDSET_SOURCE, "value": "unknown", "reason": reset.NEUTRAL,
                "observed_at": 9.9, "frame_count": 300, "source_tick": 300,
                "producer_instance": "source", "generation": 4}
        feedback.handset = Mock()
        feedback.handset.report.return_value = data
        self.assertEqual(feedback.report(), data)
        data["generation"] = 5
        with self.assertRaisesRegex(reset.Refused, "stop_handset_changed"):
            feedback.report()

    def test_freshness_active_input_and_unknown_source_remain_distinct(self):
        data = {"source": reset.HANDSET_SOURCE, "value": "unknown", "reason": reset.NEUTRAL,
                "observed_at": 10.0, "frame_count": 301, "source_tick": 301}
        self.assertTrue(reset.neutral_sample(data, lambda: 10.1))
        self.assertFalse(reset.neutral_sample(data, lambda: 10.21))
        self.assertFalse(reset.neutral_sample({**data, "source": "client"}, lambda: 10.1))
        with self.assertRaisesRegex(reset.Refused, "stop_handset_active"):
            reset.neutral_sample({**data, "value": "owned", "reason": "handset_active"}, lambda: 10.1)


class ProtocolTest(unittest.TestCase):
    def request(self, request, *, authorized=True):
        operation = Mock()
        operation.status.return_value = reset.reply("stop_reset_ready", ok=True, available=True)
        conn = Mock()
        conn.recv.side_effect = [request[:96], b""]
        reset.serve(conn, operation, OWNER, BOOT, authorize=lambda _: authorized)
        if not conn.sendall.called:
            return None, operation
        return json.loads(conn.sendall.call_args.args[0]), operation

    def test_fixed_schema_and_only_one_request(self):
        response, operation = self.request(b'{"schema_version":1,"op":"status"}\n')
        self.assertTrue(response["available"])
        operation.status.assert_called_once()
        operation.reset.assert_not_called()
        bad = (b'{"schema_version":true,"op":"reset"}\n',
               b'{"schema_version":1,"op":"status","op":"reset"}\n',
               b'{"schema_version":1,"op":"reset","host":"external"}\n',
               b'{"schema_version":1,"op":"reset"}\n{"schema_version":1,"op":"reset"}\n',
               b'x' * 96, b'{"schema_version":1,"op":"reset"}')
        for request in bad:
            with self.subTest(request=request):
                response, operation = self.request(request)
                self.assertEqual(response["reason"], "stop_reset_bad_request")
                operation.reset.assert_not_called()

    def test_untrusted_process_gets_no_operation(self):
        response, operation = self.request(b'{"schema_version":1,"op":"reset"}\n', authorized=False)
        self.assertIsNone(response)
        operation.reset.assert_not_called()

    def test_uid_alone_does_not_authorize_an_ssh_login_of_the_same_user(self):
        connection = Mock()
        connection.getsockopt.return_value = struct.pack("3i", 4321, 1001, 1001)
        user = SimpleNamespace(pw_uid=1001)
        with patch.object(reset.pwd, "getpwnam", return_value=user):
            with patch.object(Path, "read_text", return_value="0::/user.slice/session-2.scope\n"):
                self.assertFalse(reset.allowed_peer(connection))
            with patch.object(Path, "read_text", return_value="0::" + reset.GATEWAY_CGROUP + "\n"):
                self.assertTrue(reset.allowed_peer(connection))
            with patch.object(Path, "read_text", return_value="1:name=systemd:" + reset.GATEWAY_CGROUP + "\n"):
                self.assertTrue(reset.allowed_peer(connection))


if __name__ == "__main__":
    unittest.main()
