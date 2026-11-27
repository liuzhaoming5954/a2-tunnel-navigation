"""用假接收器和假证据验证一键启用代理，不加载实机运动 SDK。"""

from contextlib import nullcontext
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import operate_a2_manual_arm as broker


class Receiver:
    def __init__(self, *, active=False, profile=True, response=None):
        self.active, self.profile, self.starts = active, profile, 0
        self.response = response or {'ok': True, 'session_id': 'x' * 32, 'continuous_hold': True}
        self.requests = []
        self.state = {'source': 'unitree', 'armed': False, 'reason': 'local_authorization_required'}

    def status(self):
        return {'ActiveState': 'active' if self.active else 'inactive', 'MainPID': '123' if self.active else '0'}

    def manual_process(self, props):
        return ('invocation', int(props['MainPID'])) if self.active and self.profile else None

    def start(self):
        self.starts += 1
        self.active = True

    def request(self, operation, timeout):
        self.requests.append(operation)
        if operation == 'receiver-status':
            return self.state
        if isinstance(self.response, Exception):
            raise self.response
        return self.response


class ArmTests(unittest.TestCase):
    def activate(self, receiver, *, records=None, preflight=None, reusable_hold=False):
        transaction = SimpleNamespace(locked=nullcontext)
        arm = broker.Arm(transaction, Mock(), receiver, sleep=lambda _duration: None)
        old = {'serial': 0, 'grant': None}
        check = Mock(return_value=('epoch', 'guard')) if preflight is None else Mock(side_effect=preflight)
        with (patch.object(arm, 'preflight', check),
              patch.object(broker, 'identity', return_value=('boot', 'receiver')),
              patch.object(broker, 'grant_status', side_effect=records or [old, old]),
              patch.object(broker, 'issue_cockpit', return_value={'ok': True, 'grant': {'serial': 12}}) as issue,
              patch.object(broker, 'revoke_cockpit', return_value={'ok': True, 'serial': 11}) as revoke):
            reply = arm.activate(reusable_hold=reusable_hold)
        return reply, issue, revoke

    def test_explicit_click_starts_only_manual_receiver_and_returns_continuous_hold(self):
        receiver = Receiver()
        reply, issue, revoke = self.activate(receiver)
        self.assertEqual(reply, {'ok': True, 'session_id': 'x' * 32, 'continuous_hold': True})
        self.assertEqual(receiver.starts, 1)
        self.assertEqual(receiver.requests, ['receiver-status', 'receiver-acquire'])
        issue.assert_called_once_with(0)
        revoke.assert_not_called()

    def test_reusable_click_passes_only_fixed_receiver_operation(self):
        receipt = {'ok': True, 'session_id': 'x' * 32, 'continuous_hold': True, 'reusable_hold': True}
        receiver = Receiver(response=receipt)
        reply, issue, revoke = self.activate(receiver, reusable_hold=True)
        self.assertEqual(reply, receipt)
        self.assertEqual(receiver.requests, ['receiver-status', 'receiver-acquire-reusable-hold'])
        issue.assert_called_once_with(0)
        revoke.assert_not_called()

    def test_reusable_click_rejects_legacy_success_without_retry(self):
        receiver = Receiver()
        reply, issue, revoke = self.activate(receiver, reusable_hold=True)
        self.assertFalse(reply['ok'])
        self.assertEqual(receiver.requests, ['receiver-status', 'receiver-acquire-reusable-hold'])
        issue.assert_called_once_with(0)
        revoke.assert_not_called()

    def test_latch_blocks_before_start_grant_or_acquire(self):
        receiver = Receiver()
        reply, issue, revoke = self.activate(receiver, preflight=[broker.Refused('stop_latched')])
        self.assertEqual(reply, broker.failure('stop_latched'))
        self.assertEqual(receiver.starts, 0)
        self.assertEqual(receiver.requests, [])
        issue.assert_not_called()
        revoke.assert_not_called()

    def test_running_autonomous_profile_is_left_alone(self):
        receiver = Receiver(active=True, profile=False)
        reply, issue, revoke = self.activate(receiver)
        self.assertEqual(reply, broker.failure('manual_arm_profile_unavailable'))
        self.assertEqual(receiver.starts, 0)
        self.assertEqual(receiver.requests, [])
        issue.assert_not_called()
        revoke.assert_not_called()

    def test_legacy_grant_survives_the_click(self):
        receiver = Receiver()
        reply, issue, revoke = self.activate(receiver, records=[{'serial': 7, 'grant': {'serial': 7}}])
        self.assertEqual(reply, broker.failure('local_grant_active'))
        self.assertEqual(receiver.starts, 0)
        issue.assert_not_called()
        revoke.assert_not_called()

    def test_active_session_preserves_broker_grant(self):
        receiver = Receiver(active=True)
        receiver.state = {'source': 'unitree', 'armed': True, 'reason': 'armed'}
        old = {'serial': 10, 'grant': {'serial': 10}, 'cockpit_managed': True}
        reply, issue, revoke = self.activate(receiver, records=[old])
        self.assertEqual(reply, broker.failure('session_busy'))
        self.assertEqual(receiver.requests, ['receiver-status'])
        issue.assert_not_called()
        revoke.assert_not_called()

    def test_new_click_after_confirmed_idle_replaces_only_its_cockpit_grant(self):
        receiver = Receiver(active=True)
        old = {'serial': 10, 'grant': {'serial': 10}, 'cockpit_managed': True}
        reply, issue, revoke = self.activate(receiver, records=[old, old])
        self.assertTrue(reply['ok'])
        self.assertEqual(receiver.starts, 0)
        revoke.assert_called_once_with(10)
        issue.assert_called_once_with(11)

    def test_does_not_issue_while_stop_unconfirmed(self):
        receiver = Receiver(active=True)
        old = {'serial': 10, 'grant': {'serial': 10}, 'cockpit_managed': True}
        with patch.object(receiver, 'request', side_effect=[
            {'source': 'unitree', 'armed': False, 'reason': 'stop_unconfirmed'},
            {'source': 'unitree', 'armed': False, 'reason': 'local_authorization_required'},
            {'ok': True, 'session_id': 'x' * 32, 'continuous_hold': True},
        ]) as request:
            reply, issue, revoke = self.activate(receiver, records=[old, old])
        self.assertTrue(reply['ok'])
        self.assertEqual([call.args[0] for call in request.call_args_list],
                         ['receiver-status', 'receiver-status', 'receiver-acquire'])
        issue.assert_called_once_with(11)
        revoke.assert_called_once_with(10)

    def test_explicit_acquire_refusal_cleans_only_new_grant(self):
        receiver = Receiver(response={'ok': False, 'reason': 'operator_hold_neutral_required'})
        reply, _, revoke = self.activate(receiver)
        self.assertEqual(reply, broker.failure('operator_hold_neutral_required'))
        revoke.assert_called_once_with(12)

    def test_missing_reply_cannot_create_second_session_or_revoke_unconfirmed_one(self):
        receiver = Receiver(response=subprocess.TimeoutExpired('private receiver client', .2))
        reply, issue, revoke = self.activate(receiver)
        self.assertEqual(reply, broker.failure('manual_arm_unavailable'))
        self.assertEqual(receiver.requests.count('receiver-acquire'), 1)
        issue.assert_called_once()
        revoke.assert_not_called()

    def test_dangling_sender_fault_link_blocks_before_grant_or_start(self):
        with TemporaryDirectory() as directory:
            marker = Path(directory) / 'sender-fault'
            marker.symlink_to(Path(directory) / 'missing')
            transaction = SimpleNamespace(locked=nullcontext, has_intent=lambda: False,
                                          files=SimpleNamespace(sender_fault=marker))
            receiver, guard = Receiver(), Mock()
            arm = broker.Arm(transaction, guard, receiver)
            with (patch.object(broker, 'identity', return_value=('boot', 'receiver')),
                  patch.object(broker, '_ready_epoch', return_value=('epoch', None)),
                  patch.object(broker, 'issue_cockpit') as issue):
                reply = arm.activate()
        self.assertEqual(reply, broker.failure('stop_guard_not_ready'))
        guard.invocation.assert_not_called()
        issue.assert_not_called()
        self.assertEqual(receiver.starts, 0)


class RequestTests(unittest.TestCase):
    def test_rejects_non_gateway_without_parsing_or_starting(self):
        connection, arm = Mock(), Mock()
        broker.serve(connection, arm, authorize=lambda _: False)
        connection.recv.assert_not_called()
        connection.sendall.assert_not_called()
        arm.activate.assert_not_called()

    def test_two_fixed_requests_choose_the_expected_activation_mode(self):
        for operation, reusable_hold in (('activate', False), ('activate_reusable_hold', True)):
            with self.subTest(operation=operation):
                connection, arm = Mock(), Mock()
                connection.recv.return_value = json.dumps({'op': operation}).encode() + b'\n'
                arm.activate.return_value = {'ok': False, 'reason': 'stop_latched'}
                broker.serve(connection, arm, authorize=lambda _: True)
                arm.activate.assert_called_once_with(reusable_hold=reusable_hold)

    def test_commands_outside_fixed_set_are_rejected(self):
        for raw in (b'{"op":"activate","direction":"forward"}\n',
                    b'{"op":"activate_reusable_hold","reusable_hold":true}\n',
                    b'{"op":"activate","op":"activate"}\n', b'{"op":"reset"}\n',
                    b'{"op":"activate"}\n{"op":"activate"}\n'):
            with self.subTest(raw=raw):
                connection, arm = Mock(), Mock()
                connection.recv.return_value = raw
                broker.serve(connection, arm, authorize=lambda _: True)
                arm.activate.assert_not_called()
                response = json.loads(connection.sendall.call_args.args[0])
                self.assertEqual(response, broker.failure('invalid_manual_command'))


if __name__ == '__main__':
    unittest.main()
