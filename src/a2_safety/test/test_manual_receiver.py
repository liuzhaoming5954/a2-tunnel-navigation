"""仅注入合成底盘、受控时钟和假 SDK，验证与机载服务相同的本地仲裁路径。"""

from contextlib import closing
from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
from threading import Event
import time
import unittest
from unittest.mock import MagicMock, patch

import manual_receiver as manual
from maintenance_arbiter import Direction, Feedback, FrontLidar, OperatorHold, Owner, Sample, VelocityRequest
from manual_authority import HANDSET_SOURCE


class ManualReceiverTest(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.stopped = Event()
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.latch = Path(self.directory.name) / 'physical-A'

        def capture(event):
            self.events.append(event)
            if event['event'] == 'stop':
                self.stopped.set()

        self.body = manual.MockBody(self.latch, capture)
        self.receiver = manual.ManualReceiver(self.body)
        self.addCleanup(self.receiver.close)

    def request(self, op, **payload):
        return self.receiver.handle({'op': op, 'schema_version': 1, **payload})

    def acquire(self):
        response = self.request('acquire')
        self.assertTrue(response['ok'], response)
        return response['session_id']

    def move(self, session, sequence=1, vx=.05, yaw=0.):
        return self.request('velocity', session_id=session, sequence=sequence, linear_mps=vx, angular_radps=yaw)

    def test_all_body_directions_limits_and_explicit_stop(self):
        control = self.request('status')['control']
        turn = math.pi / 18
        self.assertEqual(control['source'], 'mock')
        self.assertEqual(control['limits'], {'linear_mps': .20, 'forward_mps': .20, 'angular_radps': turn})
        for vx, yaw in [(.20, 0.), (-.20, 0.), (0., turn), (0., -turn)]:
            session = self.acquire()
            for x, z in [(.20001, 0.), (-.20001, 0.), (0., turn + .00001), (0., -turn - .00001),
                         (.02, .01), (float('nan'), 0.)]:
                self.assertFalse(self.move(session, vx=x, yaw=z)['ok'])
            self.assertTrue(self.move(session, vx=vx, yaw=yaw)['ok'])
            self.assertEqual((self.body.linear, self.body.angular), (vx, yaw))
            self.assertFalse(self.move(session, vx=vx, yaw=yaw)['ok'])
            self.assertTrue(self.request('stop', session_id=session)['ok'])
            self.assertEqual((self.body.linear, self.body.angular), (0., 0.))
            self.assertFalse(self.move(session, sequence=2)['ok'])

    def test_station_disappears_stops_without_any_request_and_rejects_late_packet(self):
        session = self.acquire()
        self.assertTrue(self.move(session)['ok'])
        self.assertTrue(self.stopped.wait(.85))
        self.assertEqual([r['event'] for r in self.events], ['move', 'stop'])
        self.assertEqual(self.events[-1]['reason'], 'deadman_expired')
        elapsed = self.events[-1]['monotonic_s'] - self.events[0]['monotonic_s']
        self.assertGreaterEqual(elapsed, manual.LIMITS.deadman_s - .002)
        self.assertLess(elapsed, .8)
        self.assertFalse(self.move(session, sequence=2)['ok'])

    def test_a_input_latches_across_new_requests_and_gateway_reconnection(self):
        session = self.acquire()
        self.assertTrue(self.move(session)['ok'])
        self.latch.touch()
        self.assertTrue(self.stopped.wait(.5))
        self.latch.unlink()
        control = self.request('status')['control']
        self.assertFalse(control['armed'] or control['available'])
        self.assertEqual(control['reason'], 'stop_latched')
        self.assertFalse(self.request('acquire')['ok'])
        self.assertFalse(self.move(session, sequence=2)['ok'])
        self.assertEqual(self.events[-1]['reason'], 'stop_latched')

    def test_lost_evidence_retracts_own_move_retries_failed_stop_without_browser(self):
        session = self.acquire()
        self.assertTrue(self.move(session)['ok'])
        stop = self.body.stop
        calls = []

        def temporary_failure(token, reason):
            calls.append(time.monotonic())
            if len(calls) == 1:
                raise TimeoutError('模拟回执丢失')
            return stop(token, reason)

        with patch.object(self.body, 'stop', temporary_failure), patch.object(self.body, 'observe', side_effect=OSError):
            self.assertTrue(self.stopped.wait(.6))
            self.assertEqual(len(calls), 2)
            self.assertGreaterEqual(calls[1]-calls[0], .09)
            self.assertFalse(self.request('status')['control']['available'])
            self.assertFalse(self.move(session, sequence=2)['ok'])

    def test_original_remote_takeover_does_not_compete_for_motion(self):
        session = self.acquire()
        self.assertTrue(self.move(session)['ok'])
        owner = Sample(Owner.OWNED, time.monotonic())
        data = replace(self.body.observe(), owner=owner)
        with patch.object(self.body, 'observe', return_value=data):
            self.assertFalse(self.request('status')['control']['armed'])
            self.assertFalse(self.move(session, sequence=2)['ok'])
        self.assertEqual([r['event'] for r in self.events], ['move'])

    def test_bad_schema_and_non_control_ops_never_reach_chassis(self):
        for message in [{}, {'schema_version': True, 'op': 'acquire'},
                        {'schema_version': 1, 'op': 'reset_latch'}, {'schema_version': 1, 'op': 'status', 'extra': 1}]:
            self.assertFalse(self.receiver.handle(message)['ok'])
        self.assertEqual(self.events, [])

    def test_idle_heartbeats_do_not_move_and_tardy_first_direction_is_rejected(self):
        # 通过可控的 PC2 单调钟跳过多个心跳周期，不依赖开发机调度精度。
        now = [10.]
        events = []
        body = manual.MockBody(clock=lambda: now[0], emit=events.append)
        receiver = manual.ManualReceiver(body, clock=lambda: now[0])
        self.addCleanup(receiver.close)
        now[0] += .01
        token = receiver.arbiter.acquire({})[1]['session_id']
        def pulse(seq, vx=0.):
            return receiver.arbiter.velocity(dict(session_id=token, sequence=seq, linear_mps=vx, angular_radps=0.))
        for seq in range(1,4):
            now[0] += .2
            self.assertEqual(pulse(seq)[0], 200)
        self.assertEqual(events, [])
        now[0] += .401
        self.assertEqual(pulse(4, .05)[1]['reason'], 'invalid_session')
        self.assertFalse(receiver.arbiter.state()['armed'])
        self.assertEqual([e['event'] for e in events], ['stop'])
        self.assertEqual(events[0]['reason'], 'deadman_expired')

    def test_late_idle_packets_cannot_extend_motion(self):
        now = [10.]
        events = []
        body = manual.MockBody(clock=lambda: now[0], emit=events.append)
        receiver = manual.ManualReceiver(body, clock=lambda: now[0])
        self.addCleanup(receiver.close)
        now[0] += .01
        token = receiver.arbiter.acquire({})[1]['session_id']
        def pulse(seq, vx):
            return receiver.arbiter.velocity(dict(session_id=token, sequence=seq, linear_mps=vx, angular_radps=0.))
        self.assertEqual(pulse(1, 0.)[0], 200)
        self.assertEqual(pulse(2, .05)[0], 200)
        now[0] += .3
        self.assertEqual(pulse(3, 0.)[1]['reason'], 'idle_heartbeat_after_motion')
        now[0] += .101
        self.assertEqual(pulse(4, 0.)[1]['reason'], 'invalid_session')
        self.assertEqual([e['event'] for e in events], ['move', 'stop'])


class IndependentHoldTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.events = []
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.hold = Path(directory.name) / 'hold.json'
        self.latch = Path(directory.name) / 'physical-A'
        self.body = manual.MockBody(stop_file=self.latch, emit=self.events.append, clock=lambda: self.now,
                                    hold_file=self.hold)
        self.receiver = manual.ManualReceiver(self.body, clock=lambda: self.now)
        self.addCleanup(self.receiver.close)
        self.now += 1.
        response = self.request('acquire')
        self.assertTrue(response['ok'])
        self.assertEqual(response['expires_in_ms'], 30000)
        self.session = response['session_id']

    def request(self, op, **payload):
        return self.receiver.handle({'op': op, 'schema_version': 1, **payload})

    def publish(self, pressed, *, epoch='first', timestamp=None):
        sample_time = self.now - .000001 if timestamp is None else timestamp
        staging = self.hold.with_suffix('.next')
        staging.write_text(json.dumps({'pressed': pressed, 'epoch': epoch,
                                      'observed_mono_ns': round(sample_time * 1e9)}))
        staging.chmod(0o600)
        staging.replace(self.hold)

    def velocity(self, sequence, **fields):
        return self.request('velocity', session_id=self.session, sequence=sequence,
                            linear_mps=.20, angular_radps=0., **fields)

    def assert_stopped(self, reason):
        self.assertEqual(self.body.linear, 0.)
        self.assertEqual([row['reason'] for row in self.events if row['event'] == 'stop'], [reason])
        self.assertFalse(self.request('status')['control']['armed'])

    def test_local_release_overrides_continuing_browser_packets(self):
        for seq in range(1, 5):
            self.now += .05
            self.publish(True)
            self.assertTrue(self.velocity(seq)['ok'])
        self.now += .05
        self.publish(False)
        # 故意不给浏览器送松键。下一条正常序号的非零速度也不能继续运动。
        self.assertEqual(self.velocity(5)['reason'], 'invalid_session')
        self.assert_stopped('operator_hold_released')
        self.now += .05
        self.publish(True, epoch='second')
        self.assertEqual(self.velocity(6)['reason'], 'invalid_session')
        self.assert_stopped('operator_hold_released')

    def test_stale_hold_is_not_refreshed_by_velocity_packets(self):
        self.publish(True)
        self.assertTrue(self.velocity(1)['ok'])
        for seq in range(2, 5):
            self.now += .05
            self.assertTrue(self.velocity(seq)['ok'])
        self.now += .06
        # 最新网页请求仅 60 毫秒前开始，远小于 400 毫秒断流门限。
        self.assertEqual(self.velocity(5)['reason'], 'invalid_session')
        self.assert_stopped('operator_hold_stale')

    def test_stale_or_missing_evidence_blocks_motion_without_consuming_web_sequence(self):
        self.assertEqual(self.velocity(1)['reason'], 'operator_hold_stale')
        self.publish(True, timestamp=self.now + 1.)
        self.assertEqual(self.velocity(1)['reason'], 'operator_hold_stale')
        self.publish(False)
        self.assertEqual(self.velocity(1)['reason'], 'operator_hold_released')
        self.publish(True)
        self.assertEqual(self.velocity(1, operator_hold=True)['reason'], 'invalid_velocity')
        self.assertEqual(self.events, [])
        self.assertTrue(self.velocity(1)['ok'])
        self.now += .05
        # 快速放开再按下时，下一份信号即便是按住，也属于新的按压周期。
        self.publish(True, epoch='second')
        self.assertEqual(self.velocity(2)['reason'], 'invalid_session')
        self.assert_stopped('operator_hold_changed')

    def test_hold_does_not_extend_thirty_second_session(self):
        for seq in range(1, 241):
            self.publish(True)
            self.assertTrue(self.velocity(seq)['ok'])
            self.now += .125
        self.publish(True)
        self.assertEqual(self.velocity(241)['reason'], 'invalid_session')
        self.assert_stopped('lease_expired')


class UnitreeContractTest(unittest.TestCase):
    def setUp(self):
        self.body = object.__new__(manual.UnitreeBody)
        self.body.boot_id = 'this-boot'
        self.body.receiver_id = 'this-machine'
        self.body.sport = MagicMock()
        self.body._handed_over_session = None
        now = time.monotonic()
        self.data = {'boot_id':'this-boot','receiver_id':'this-machine',
                     'owner':{'value':'released','observed_at':now},
                     'feedback':{'value':'stopped','observed_at':now,'sport_mode':3,'gait_type':0,'error_code':0},
                     'guard':{'directions':['forward'],'observed_at':now},
                     'grant':{'serial':1,'issued_at':now,'expires_at':now+1.,'directions':['forward']}}
        # 独立权属和 PC2 本地许可由各自的适配器提供。旧合并快照里的同名字段无放行权。
        self.owner = self.enter_patch('read_owner', lambda *_: self.data['owner'])
        self.grants = self.enter_patch('grant_status', lambda *_: {'reason':'granted', 'grant':self.data['grant']})

    def enter_patch(self, name, effect):
        patcher = patch.object(manual, name, side_effect=effect)
        stub = patcher.start()
        self.addCleanup(patcher.stop)
        return stub

    def test_unitree_branch_requires_guard_and_running_autonomy_service(self):
        with patch.object(manual, 'guard_ready', return_value=False):
            with self.assertRaises(RuntimeError):
                manual.UnitreeBody('nonexistent', 0)
            self.assertFalse(self.body.velocity(MagicMock()))
        self.body.sport.Move.assert_not_called()

    def test_evidence_bound_to_machine_boot_and_supported_posture(self):
        with patch.object(manual,'trusted_file', side_effect=lambda _path: json.dumps(self.data)):
            self.assertEqual(self.body.observe().feedback.value, Feedback.STOPPED)
            self.data['feedback']['sport_mode'] = 1
            self.assertIsNone(self.body.observe().feedback)
            for key in ['boot_id','receiver_id']:
                with patch.dict(self.data, {key:'mismatch'}):
                    inputs = self.body.observe()
                    self.assertIsNone(inputs.feedback)
                    self.assertEqual(inputs.guard.value, frozenset())
                    self.assertLess(inputs.guard.observed_at, 0.)

    def test_field_tape_keeps_original_sample_time_without_authorizing_a_direction(self):
        stamp = self.data['guard']['observed_at']
        # 即使静止快照误填前进，尺量仍只提供带原始时刻的观测值。
        self.data['guard'].update(source='field_tape_static', directions=['forward', 'left'])
        with patch.object(manual, 'trusted_file', side_effect=lambda _path: json.dumps(self.data)):
            for distance in (0, 2, 2.35):
                with self.subTest(distance=distance):
                    self.data['guard']['front_clearance_m'] = distance
                    inputs = self.body._field_inputs()
                    self.assertEqual(inputs.static_front_clearance, Sample(distance, stamp))
                    self.assertEqual(inputs.guard, Sample(frozenset(), stamp))
                    self.assertEqual(inputs.feedback.value, Feedback.STOPPED)

    def test_static_front_clearance_rejects_invalid_values_and_other_sources(self):
        self.data['guard'].update(source='field_tape_static')
        with patch.object(manual, 'trusted_file', side_effect=lambda _path: json.dumps(self.data)):
            self.assertIsNone(self.body._field_inputs().static_front_clearance)
            for distance in (True, '2.0', -.01, float('nan'), float('inf'), 10**400):
                with self.subTest(distance=distance):
                    self.data['guard']['front_clearance_m'] = distance
                    inputs = self.body._field_inputs()
                    self.assertIsNone(inputs.static_front_clearance)
                    self.assertEqual(inputs.guard.value, frozenset())
            self.data['guard'].update(source='live_front_sensor', front_clearance_m=2.35)
            inputs = self.body._field_inputs()
            self.assertIsNone(inputs.static_front_clearance)
            self.assertEqual({direction.value for direction in inputs.guard.value}, {'forward'})

    def test_field_tape_cannot_admit_motion_even_with_other_inputs_ready(self):
        now = [10.]
        self.data['guard'].update(source='field_tape_static', front_clearance_m=2.35)
        with patch.object(manual, 'guard_ready', return_value=True), \
                patch.object(manual, 'trusted_file', side_effect=lambda _path: json.dumps(self.data)), \
                closing(manual.ManualReceiver(self.body, clock=lambda: now[0])) as receiver:
            # 使用原本准许前进的合成许可，验证静止尺量永远不能满足方向门禁。
            now[0] += .01
            for field in ('owner', 'feedback', 'guard'):
                self.data[field]['observed_at'] = now[0]
            self.data['grant'].update(issued_at=now[0], expires_at=now[0] + 1)
            self.assertEqual(receiver.arbiter.acquire({})[1], {'ok': False, 'reason': 'obstacle'})
            self.assertFalse(receiver.arbiter.state()['available'])
            self.body.sport.Move.assert_not_called()
            self.body.sport.StopMove.assert_not_called()

    def test_fresh_front_lidar_keeps_original_observation_and_tape_diagnostics(self):
        now = self.data['feedback']['observed_at']
        self.data['guard'].update(source='field_tape_static', front_clearance_m=4.5)
        self.data['front_lidar'] = dict(source='front_lidar_diagnostic', frame_id='hesai_lidar',
                                       observed_at=now - .1, near_return_m=.75, point_count=220)
        with patch.object(manual, 'trusted_file', side_effect=lambda _path: json.dumps(self.data)), \
                patch.object(manual.time, 'monotonic', return_value=now):
            for distance in (.75, None):
                with self.subTest(distance=distance):
                    self.data['front_lidar']['near_return_m'] = distance
                    inputs = self.body._field_inputs()
                    self.assertEqual(inputs.front_lidar, Sample(FrontLidar(distance, 220, 'hesai_lidar'), now - .1))
                    self.assertEqual(inputs.static_front_clearance, Sample(4.5, now))
                    self.assertEqual(inputs.feedback, Sample(Feedback.STOPPED, now))
                    self.assertEqual(inputs.guard, Sample(frozenset(), now))

    def test_invalid_or_stale_lidar_diagnostics_never_restore_directions(self):
        now = self.data['guard']['observed_at']
        valid = dict(source='front_lidar_diagnostic', frame_id='hesai_lidar', observed_at=now,
                     near_return_m=.75, point_count=220)
        malformed = [None, {'near_return_m': None}]
        invalid_values = {
            'source': ['live_forward_guard'], 'frame_id': ['base_link'],
            'observed_at': [now - .201, now + .001, float('nan')],
            'near_return_m': [True, '0.75', -.01, float('inf'), 10**400],
            'point_count': [True, 0, -1, 220.0],
        }
        for field, values in invalid_values.items():
            malformed.extend({**valid, field: value} for value in values)
        with patch.object(manual, 'trusted_file', side_effect=lambda _path: json.dumps(self.data)), \
                patch.object(manual.time, 'monotonic', return_value=now):
            for report in malformed:
                with self.subTest(report=report):
                    self.data['front_lidar'] = report
                    inputs = self.body._field_inputs()
                    self.assertIsNone(inputs.front_lidar)
                    self.assertEqual(inputs.guard, Sample(frozenset(), now))
                    self.assertEqual(inputs.feedback, Sample(Feedback.STOPPED, now))
            del self.data['front_lidar']
            self.assertEqual({direction.value for direction in self.body._field_inputs().guard.value}, {'forward'})

    def test_lidar_diagnostic_with_no_near_return_cannot_admit_motion(self):
        now = [10.]
        # 有效点云只供诊断。即使没有近点、快照误填前进且本轮已有许可，也不能激活遥控。
        with patch.object(manual, 'guard_ready', return_value=True), \
                patch.object(manual, 'trusted_file', side_effect=lambda _path: json.dumps(self.data)), \
                patch.object(manual.time, 'monotonic', side_effect=lambda: now[0]), \
                closing(manual.ManualReceiver(self.body, clock=lambda: now[0])) as receiver:
            now[0] += .01
            for field in ('owner', 'feedback', 'guard'):
                self.data[field]['observed_at'] = now[0]
            self.data['front_lidar'] = dict(source='front_lidar_diagnostic', frame_id='hesai_lidar',
                                           observed_at=now[0], near_return_m=None, point_count=220)
            self.data['grant'].update(issued_at=now[0], expires_at=now[0] + 1)
            self.assertEqual(receiver.arbiter.acquire({})[1], {'ok': False, 'reason': 'obstacle'})
            self.assertFalse(receiver.arbiter.state()['available'])
            self.body.sport.Move.assert_not_called()
            self.body.sport.StopMove.assert_not_called()

    def test_body_axes_and_stop_ack_require_integer_zero(self):
        req = MagicMock(linear_mps=0.,angular_radps=-.15,grant_serial=1,expires_at=time.monotonic()+1.)
        with patch.object(manual,'guard_ready',return_value=True):
            self.body.sport.Move.return_value = 0
            self.assertTrue(self.body.velocity(req))
            self.body.sport.Move.assert_called_once_with(0.,0.,-.15)
            req.expires_at = time.monotonic()-1.
            self.assertFalse(self.body.velocity(req))
            self.assertEqual(self.body.sport.Move.call_count,1)
        self.body.sport.StopMove.return_value = False
        self.assertFalse(self.body.stop('session','operator_stop'))
        self.body.sport.StopMove.return_value = 0
        self.assertTrue(self.body.stop('session','operator_stop'))

    def test_legacy_snapshot_cannot_override_handset_or_issue_grant(self):
        with patch.object(manual, 'trusted_file', return_value=json.dumps(self.data)):
            self.grants.side_effect = lambda *_: {'reason':'local_authorization_required','grant':None}
            self.assertIsNone(self.body.observe().grant)
            self.owner.side_effect = lambda *_: {'value':'unknown','observed_at':None}
            self.assertIsNone(self.body.observe().owner)
            self.owner.side_effect = lambda *_: {'value':'owned','observed_at':time.monotonic()}
            self.assertEqual(self.body.observe().owner.value, Owner.OWNED)
        with patch.object(manual, 'trusted_file', side_effect=FileNotFoundError):
            self.assertEqual(self.body.observe().owner.value, Owner.OWNED)
            self.assertFalse(self.body.stop('old-session','authority_changed'))
            self.owner.side_effect = lambda *_: {'value':'unknown','observed_at':None}
            self.assertFalse(self.body.stop('old-session','retry'))
            self.body.sport.StopMove.assert_not_called()

    def test_last_moment_revoke_or_takeover_blocks_sdk(self):
        req = MagicMock(session_id='old-session',grant_serial=1,linear_mps=.05,angular_radps=0.,
                        expires_at=time.monotonic()+1.)
        with patch.object(manual, 'guard_ready', return_value=True):
            for reason in ('local_grant_expired','control_owned'):
                self.grants.side_effect = lambda *_, why=reason: {'grant':None,'reason':why}
                self.assertFalse(self.body.velocity(req))
            self.assertFalse(self.body.stop('old-session','output_rejected'))
            self.body.sport.Move.assert_not_called()
            self.body.sport.StopMove.assert_not_called()

    def test_site_confirmation_uses_fresh_read_only_body_feedback_when_guard_absent(self):
        now = 10.
        self.data['owner']['observed_at'] = now
        self.data['grant'].update(issued_at=now, expires_at=now + 5., site_front_confirmed=True)
        self.body._site_observer = MagicMock()
        self.body._site_observer.motion_feedback.return_value = {'value': 'stopped', 'observed_at': now}
        with patch.object(manual, 'trusted_file', side_effect=FileNotFoundError), \
                patch.object(manual.time, 'monotonic', return_value=now):
            inputs = self.body.observe()
            self.assertTrue(inputs.grant.site_front_confirmed)
            self.assertIsNone(inputs.guard)
            self.assertEqual(inputs.feedback, Sample(Feedback.STOPPED, now))
            self.body._site_observer.motion_feedback.return_value['observed_at'] = now - .201
            self.assertIsNone(self.body.observe().feedback)
            self.body._site_observer = None
            self.assertIsNone(self.body.observe().feedback)
        self.body.sport.Move.assert_not_called()

    def test_operator_supervised_respects_grant_and_body_feedback_without_clearance_input(self):
        now = [100.]
        self.body.operator_supervised = True
        self.body._site_observer = MagicMock()
        observe = self.body._site_observer.motion_feedback
        self.body.sport.Move.return_value = self.body.sport.StopMove.return_value = 0
        with (
            patch.object(manual, 'guard_ready', return_value=True),
            patch.object(manual, 'trusted_file', side_effect=FileNotFoundError) as clearance,
            patch.object(manual.time, 'monotonic', side_effect=lambda: now[0]),
            closing(manual.ManualReceiver(self.body, clock=lambda: now[0])) as receiver
        ):
            def send(op, **payload):
                return receiver.handle({'schema_version': 1, 'op': op, **payload})

            now[0] += .05
            self.data['owner']['observed_at'] = now[0]
            self.data['grant'] = {'serial': 1, 'issued_at': now[0], 'expires_at': now[0] + 30.,
                                  'directions': ['backward']}
            observe.return_value = None
            self.assertEqual(send('acquire')['reason'], 'feedback_stale')
            observe.return_value = {'value': 'stopped', 'observed_at': now[0]}
            acquired = send('acquire')
            self.assertEqual(acquired['expires_in_ms'], 30000)
            command = {'session_id': acquired['session_id'], 'sequence': 1, 'linear_mps': -.20, 'angular_radps': 0.}
            self.assertTrue(send('velocity', **command)['ok'])
            denied = send('velocity', **{**command, 'sequence': 2, 'linear_mps': .20})
            self.assertEqual(denied['reason'], 'direction_not_authorized')
            self.assertEqual(send('status')['control']['reason'], 'stop_unconfirmed')

            now[0] = self.data['grant']['expires_at'] + .01
            self.data['owner']['observed_at'] = now[0]
            self.data['grant'] = None
            observe.return_value = {'value': 'stopped', 'observed_at': now[0]}
            self.assertEqual(send('acquire')['reason'], 'local_authorization_required')
            clearance.assert_not_called()
        self.body.sport.Move.assert_called_once_with(-.20, 0., 0.)
        self.body.sport.StopMove.assert_called_once_with()

    def test_repeated_action_protocol_waits_for_new_stop_feedback_with_fake_sdk(self):
        now = [100.]
        self.body.operator_supervised = True
        self.body._site_observer = MagicMock()
        observe = self.body._site_observer.motion_feedback
        self.body.sport.Move.return_value = self.body.sport.StopMove.return_value = 0
        with (
            patch.object(manual, 'guard_ready', return_value=True),
            patch.object(manual.time, 'monotonic', side_effect=lambda: now[0]),
            closing(manual.ManualReceiver(self.body, clock=lambda: now[0])) as receiver
        ):
            def send(op, **payload):
                return receiver.handle({'schema_version': 1, 'op': op, **payload})

            def update_feedback(value):
                now[0] += .05
                self.data['owner']['observed_at'] = now[0]
                observe.return_value = {'value': value, 'observed_at': now[0]}

            update_feedback('stopped')
            self.data['grant'].update(issued_at=now[0], expires_at=None, repeated_actions=True)
            reply = send('acquire')
            self.assertIs(reply['repeated_actions'], True)
            self.assertEqual(reply['action_limit_ms'], 30000)
            session = reply['session_id']
            action = send('action_start', session_id=session, sequence=1, direction='forward')
            self.assertEqual(action, {'ok': True, 'action_id': 1, 'expires_in_ms': 30000})
            self.body.sport.Move.assert_not_called()
            self.assertTrue(send('velocity', session_id=session, sequence=2, action_id=1,
                                 linear_mps=.20, angular_radps=0.)['ok'])
            update_feedback('moving')
            self.assertTrue(send('action_release', session_id=session, sequence=3, action_id=1)['ok'])
            stopping = send('status')['control']
            self.assertTrue(stopping['available'] and stopping['armed'])
            self.assertEqual((stopping['action_state'], stopping['action_id']), ('stopping', 1))
            update_feedback('stopped')
            ready = send('status')['control']
            self.assertEqual(ready['action_state'], 'ready')
            self.assertNotIn('action_id', ready)
            self.assertTrue(send('stop', session_id=session)['ok'])
        self.body.sport.Move.assert_called_once_with(.20, 0., 0.)

    def test_repeated_action_rechecks_request_deadline_immediately_before_sdk(self):
        now = [100.]
        self.body.operator_supervised = True
        self.body._site_observer = MagicMock()
        self.body._site_observer.motion_feedback.return_value = {'value': 'stopped', 'observed_at': now[0]}
        self.data['grant'].update(issued_at=now[0], expires_at=None, repeated_actions=True)
        deadline = now[0] + .05
        request = VelocityRequest('session', 2, 1, now[0], deadline, .20, 0.)

        def final_grant(*_):
            # 模拟最后一次取本地许可时跨过动作截止时刻。
            if self.grants.call_count == 2:
                now[0] = deadline
            return {'reason': 'granted', 'grant': self.data['grant']}

        self.grants.side_effect = final_grant
        with patch.object(manual, 'guard_ready', return_value=True), \
                patch.object(manual.time, 'monotonic', side_effect=lambda: now[0]):
            self.assertFalse(self.body.velocity(request))
        self.assertEqual(self.grants.call_count, 2)
        self.body.sport.Move.assert_not_called()

    def test_unitree_keyboard_hold_observation_is_opt_in(self):
        now = 100.
        self.body.operator_supervised = True
        self.body._site_observer = MagicMock()
        self.body._site_observer.motion_feedback.return_value = {'value': 'stopped', 'observed_at': now}
        self.data['owner']['observed_at'] = now
        self.data['grant'].update(issued_at=now, expires_at=now + 30.)
        hold = Sample(OperatorHold(True, 'press-1', Direction.FORWARD), now)
        with (
            patch.object(self.body, '_keyboard_sample', return_value=hold) as read_hold,
            patch.object(manual.time, 'monotonic', return_value=now),
        ):
            legacy = self.body.observe()
            self.assertIsNone(legacy.operator_hold)
            read_hold.assert_not_called()
            self.body.keyboard_hold = True
            configured = self.body.observe()
            self.assertEqual(configured, replace(legacy, operator_hold=hold))
            read_hold.assert_called_once_with()
        self.body.sport.Move.assert_not_called()

    def test_parking_trace_records_initial_hold_check_for_both_command_paths(self):
        self.body.operator_supervised = True
        self.body.keyboard_hold = True
        self.body._site_observer = MagicMock()
        self.body.sport.Move.return_value = self.body.sport.StopMove.return_value = 0
        self.body.parking_trace = MagicMock()
        for repeated in (False, True):
            with self.subTest(repeated=repeated):
                now = [100.]
                self.body.parking_trace.reset_mock()
                self.body.sport.reset_mock()
                with (
                    patch.object(manual, 'guard_ready', return_value=True),
                    patch.object(manual.time, 'monotonic', side_effect=lambda: now[0]),
                    patch.object(self.body, '_keyboard_sample') as read_hold,
                    closing(manual.ManualReceiver(self.body, clock=lambda: now[0])) as receiver,
                ):
                    def send(op, **payload):
                        return receiver.handle({'schema_version': 1, 'op': op, **payload})

                    now[0] += .05
                    self.data['owner']['observed_at'] = now[0]
                    self.data['grant'].update(issued_at=now[0], expires_at=None if repeated else now[0] + 30.,
                                              repeated_actions=repeated)
                    self.body._site_observer.motion_feedback.return_value = {'value': 'stopped', 'observed_at': now[0]}
                    session = send('acquire')['session_id']

                    def start(sequence):
                        if repeated:
                            return send('action_start', session_id=session, sequence=sequence, direction='forward')
                        return send('velocity', session_id=session, sequence=sequence, linear_mps=.20, angular_radps=0.)

                    observed = now[0] - .02
                    read_hold.return_value = None
                    self.assertEqual(start(1)['reason'], 'operator_hold_stale')
                    read_hold.return_value = Sample(OperatorHold(False, 'private-epoch'), observed)
                    self.assertEqual(start(2)['reason'], 'operator_hold_released')
                    self.body.sport.Move.assert_not_called()
                    read_hold.return_value = Sample(OperatorHold(True, 'private-epoch', Direction.FORWARD), observed)
                    self.assertTrue(start(3)['ok'])
                    action = {'action_id': 3} if repeated else {}
                    self.assertTrue(send('velocity', session_id=session, sequence=4,
                                         linear_mps=.20, angular_radps=0., **action)['ok'])
                    checks = [call.kwargs for call in self.body.parking_trace.record.call_args_list
                              if call.args == ('hold_start_check',)]
                    common = {'decision_mono_ns': round(now[0] * 1e9), 'requested_direction': 'forward'}
                    self.assertEqual(checks, [
                        {**common, 'reason': 'operator_hold_stale', 'sample_valid': False,
                         'sample_observed_mono_ns': None, 'sample_pressed': None, 'sample_direction': None},
                        {**common, 'reason': 'operator_hold_released', 'sample_valid': True,
                         'sample_observed_mono_ns': round(observed * 1e9), 'sample_pressed': False,
                         'sample_direction': None},
                        {**common, 'reason': None, 'sample_valid': True,
                         'sample_observed_mono_ns': round(observed * 1e9), 'sample_pressed': True,
                         'sample_direction': 'forward'},
                    ])

    def test_unitree_keyboard_hold_rechecked_before_fake_sdk_output(self):
        now = [100.]
        self.body.operator_supervised = True
        self.body.keyboard_hold = True
        self.body._site_observer = MagicMock()
        self.body._site_observer.motion_feedback.return_value = {'value': 'stopped', 'observed_at': now[0]}
        self.data['grant'].update(issued_at=now[0], expires_at=now[0] + 30., directions=['forward', 'backward'])
        self.body.sport.Move.return_value = 0
        request = VelocityRequest('session', 2, 1, now[0], now[0] + .15, .20, 0., hold_epoch='press-1')
        pressed = {'schema_version': 1, 'boot_id': self.body.boot_id, 'receiver_id': self.body.receiver_id,
                   'pressed': True, 'epoch': 'press-1', 'direction': 'forward',
                   'observed_mono_ns': round(now[0] * 1e9)}
        samples = {
            'missing': None,
            'released': {**pressed, 'pressed': False, 'direction': None},
            'stale': {**pressed, 'observed_mono_ns': round((now[0] - manual.LIMITS.max_sample_age_s - .01) * 1e9)},
            'future': {**pressed, 'observed_mono_ns': round((now[0] + .01) * 1e9)},
            'wrong_direction': {**pressed, 'direction': 'backward'},
            'new_press': {**pressed, 'epoch': 'press-2'},
            'other_boot': {**pressed, 'boot_id': 'other-boot'},
            'other_robot': {**pressed, 'receiver_id': 'other-robot'},
        }

        with (
            patch.object(manual, 'guard_ready', return_value=True),
            patch.object(manual.time, 'monotonic', side_effect=lambda: now[0]),
            patch.object(manual, 'trusted_file') as read_hold,
        ):
            # 第一次仲裁绑定的方向与按压周期必须经 SDK 调用前的独立采样再次确认。
            for label, sample in samples.items():
                with self.subTest(label=label):
                    read_hold.side_effect = FileNotFoundError if sample is None else None
                    read_hold.return_value = json.dumps(sample)
                    self.assertFalse(self.body.velocity(request))
                    self.body.sport.Move.assert_not_called()
                    read_hold.assert_called_with(manual.KEYBOARD_HOLD)
            read_hold.return_value = json.dumps(pressed)
            self.assertFalse(self.body.velocity(replace(request, hold_epoch=None)))
            self.body.sport.Move.assert_not_called()
            self.assertTrue(self.body.velocity(request))
            self.body.sport.Move.assert_called_once_with(.20, 0., 0.)

            def hold_after_expiry(_path):
                # 查询期间动作到期。新鲜键盘证据不能续期已过期的运动指令。
                now[0] = request.expires_at
                return json.dumps({**pressed, 'observed_mono_ns': round(now[0] * 1e9)})

            read_hold.side_effect = hold_after_expiry
            self.assertFalse(self.body.velocity(request))
            self.body.sport.Move.assert_called_once_with(.20, 0., 0.)

    def test_site_stop_continues_to_read_body_feedback_after_grant_expiry(self):
        now = [100.]
        deadline = 105.05
        self.body._site_observer = MagicMock()
        read = self.body._site_observer.motion_feedback
        self.body.sport.Move.return_value = self.body.sport.StopMove.return_value = 0
        with (
            patch.object(manual, 'guard_ready', return_value=True),
            patch.object(manual, 'trusted_file', side_effect=FileNotFoundError),
            patch.object(manual.time, 'monotonic', side_effect=lambda: now[0]),
            closing(manual.ManualReceiver(self.body, clock=lambda: now[0])) as receiver
        ):
            def send(op, **payload):
                return receiver.handle({'schema_version': 1, 'op': op, **payload})

            now[0] = deadline - .10
            self.data['owner']['observed_at'] = now[0]
            self.data['grant'].update(issued_at=100.05, expires_at=deadline, site_front_confirmed=True)
            read.return_value = {'value': 'stopped', 'observed_at': now[0]}
            session = send('acquire')['session_id']
            move = dict(session_id=session, sequence=1, linear_mps=.20, angular_radps=0.)
            self.assertTrue(send('velocity', **move)['ok'])

            stop_at = deadline - .02
            now[0] = stop_at
            self.data['owner']['observed_at'] = now[0]
            read.return_value = {'value': 'moving', 'observed_at': now[0]}
            self.assertTrue(send('stop', session_id=session)['ok'])
            self.data['grant'] = None

            now[0] = deadline + .04
            self.data['owner']['observed_at'] = now[0]
            # 启用期间的旧停稳帧、缺帧和仍在运动的新帧都不能完成停车确认。
            for report in ({'value': 'stopped', 'observed_at': stop_at},
                           {'value': 'stopped', 'observed_at': now[0] - .21},
                           {'value': 'moving', 'observed_at': now[0]}, None):
                with self.subTest(report=report):
                    read.return_value = report
                    self.assertEqual(send('status')['control']['reason'], 'stop_unconfirmed')

            now[0] = deadline + .60
            self.data['owner']['observed_at'] = now[0]
            read.return_value = {'value': 'stopped', 'observed_at': now[0]}
            self.data['feedback'].update(value='moving', observed_at=now[0])
            with patch.object(manual, 'trusted_file', return_value=json.dumps(self.data)):
                self.assertEqual(send('status')['control']['reason'], 'stop_unconfirmed')
            control = send('status')['control']
            self.assertEqual(control['reason'], 'local_authorization_required')
            self.assertFalse(control['available'] or control['armed'])
            self.assertFalse(send('velocity', **{**move, 'sequence': 2})['ok'])
            self.data['grant'] = {'serial': 2, 'issued_at': now[0], 'expires_at': now[0] + 5.,
                                  'directions': ['forward']}
            self.assertEqual(send('acquire')['reason'], 'feedback_stale')
        self.body.sport.Move.assert_called_once_with(.20, 0., 0.)
        self.body.sport.StopMove.assert_called_once_with()

    def test_corrupt_or_untrusted_automatic_evidence_cannot_use_site_fallback(self):
        now = 10.
        self.data['grant'].update(issued_at=now, expires_at=now + 5., site_front_confirmed=True)
        self.body._site_observer = MagicMock()
        self.body._site_observer.motion_feedback.return_value = {'value': 'stopped', 'observed_at': now}
        wrong_boot = {**self.data, 'boot_id': 'foreign-boot'}
        request = MagicMock(session_id='site', grant_serial=1, expires_at=now + .4,
                            linear_mps=.05, angular_radps=0.)
        with patch.object(manual, 'guard_ready', return_value=True), \
                patch.object(manual.time, 'monotonic', return_value=now):
            for invalid in ('not-json', json.dumps(wrong_boot)):
                with self.subTest(invalid=invalid), patch.object(manual, 'trusted_file', return_value=invalid):
                    inputs = self.body.observe()
                    self.assertEqual(inputs.feedback, Sample(Feedback.STOPPED, now))
                    self.assertEqual(inputs.guard.value, frozenset())
                    self.assertLess(inputs.guard.observed_at, 0.)
                    self.assertFalse(self.body.velocity(request))
            with patch.object(manual, 'trusted_file', side_effect=PermissionError):
                self.assertLess(self.body.observe().guard.observed_at, 0.)
                self.assertFalse(self.body.velocity(request))
        self.body.sport.Move.assert_not_called()

    def test_site_confirmation_does_not_resume_when_seen_monitor_disappears(self):
        now = 10.
        self.data['guard']['observed_at'] = now
        self.data['grant'].update(issued_at=now, expires_at=now + 5., site_front_confirmed=True)
        self.body._site_observer = MagicMock()
        self.body._site_observer.motion_feedback.return_value = {'value': 'stopped', 'observed_at': now}
        request = MagicMock(session_id='site', grant_serial=1, expires_at=now + .4,
                            linear_mps=.05, angular_radps=0.)
        with patch.object(manual, 'guard_ready', return_value=True), \
                patch.object(manual.time, 'monotonic', return_value=now):
            with patch.object(manual, 'trusted_file', return_value=json.dumps(self.data)):
                self.assertEqual(self.body.observe().guard.observed_at, now)
            with patch.object(manual, 'trusted_file', side_effect=FileNotFoundError):
                inputs = self.body.observe()
                self.assertEqual(inputs.guard.value, frozenset())
                self.assertLess(inputs.guard.observed_at, 0.)
                self.assertFalse(self.body.velocity(request))
        self.body.sport.Move.assert_not_called()

    def test_site_confirmation_last_sdk_check_rejects_turns_reverse_and_stale_body_feedback(self):
        now = 10.
        self.data['grant'].update(issued_at=now, expires_at=now + 5., site_front_confirmed=True)
        self.body._site_observer = MagicMock()
        self.body._site_observer.motion_feedback.return_value = {'value': 'moving', 'observed_at': now}
        self.body.sport.Move.return_value = 0
        with patch.object(manual, 'guard_ready', return_value=True), \
                patch.object(manual, 'trusted_file', side_effect=FileNotFoundError), \
                patch.object(manual.time, 'monotonic', return_value=now):
            request = MagicMock(session_id='site', grant_serial=1, expires_at=now + .4)
            for linear, angular in ((-.20, 0.), (0., .15), (0., -.15)):
                with self.subTest(linear=linear, angular=angular):
                    request.linear_mps, request.angular_radps = linear, angular
                    self.assertFalse(self.body.velocity(request))
            request.linear_mps, request.angular_radps = .20, 0.
            self.body._site_observer.motion_feedback.return_value['observed_at'] = now - .201
            self.assertFalse(self.body.velocity(request))
            self.body._site_observer.motion_feedback.return_value = None
            self.assertFalse(self.body.velocity(request))
            self.body.sport.Move.assert_not_called()
            self.body._site_observer.motion_feedback.return_value = {'value': 'moving', 'observed_at': now}
            self.assertTrue(self.body.velocity(request))
            self.body.sport.Move.assert_called_once_with(.20, 0., 0.)

    def test_sdk_bridge_with_synthetic_evidence_retracts_on_ready_loss(self):
        # 真实接收模式经过完全相同的仲裁路径，仅将安全快照、systemd 门禁和 SDK 替换为假对象。
        self.body.sport.Move.return_value = self.body.sport.StopMove.return_value = 0
        self.data['guard']['directions'] = self.data['grant']['directions'] = ['backward']
        with patch.object(manual, 'guard_ready', return_value=True) as gate, \
                patch.object(manual, 'trusted_file', side_effect=lambda _path: json.dumps(self.data)):
            receiver = manual.ManualReceiver(self.body)
            try:
                self.data['grant']['issued_at'] = time.monotonic()
                send = lambda op, **kw: receiver.handle({'schema_version': 1, 'op': op, **kw})
                session = send('acquire')['session_id']
                self.assertTrue(send('velocity', session_id=session, sequence=1, linear_mps=-.20, angular_radps=0.)['ok'])
                self.body.sport.Move.assert_called_once_with(-.20, 0., 0.)
                gate.return_value = False
                self.assertEqual(send('status')['control']['reason'], 'stop_guard_not_ready')
                self.body.sport.StopMove.assert_called_once_with()
                self.assertFalse(send('velocity', session_id=session, sequence=2, linear_mps=.05, angular_radps=0.)['ok'])
                gate.return_value = True
                self.assertFalse(send('velocity', session_id=session, sequence=3, linear_mps=.05, angular_radps=0.)['ok'])
            finally:
                receiver.close()

    def test_verified_center_keeps_other_gates_and_yields_on_takeover(self):
        # 走真实接收器的仲裁和 SDK 调用判断，只替换现场输入、时钟与运动客户端。
        now = [time.monotonic()]
        producer = '458b4d06-0e3f-409b-926f-4ec72efd5ace'
        centered = dict(value='unknown', reason='handset_neutral_release_unconfirmed',
                        source=HANDSET_SOURCE, neutral_epoch=f'{producer}:2', frame_count=11, source_tick=33)
        self.data['owner'] = {**centered, 'observed_at': now[0]}
        self.body.sport.Move.return_value = self.body.sport.StopMove.return_value = 0
        authorized = self.data.pop('grant')
        self.grants.side_effect = lambda *_: {'grant': self.data.get('grant'), 'reason': 'granted'}
        with patch.object(manual, 'guard_ready', return_value=True), \
                patch.object(manual, 'trusted_file', side_effect=lambda _path: json.dumps(self.data)), \
                closing(manual.ManualReceiver(self.body, clock=lambda: now[0])) as receiver:
            def send(op, **payload):
                return receiver.handle({'schema_version': 1, 'op': op, **payload})

            def move(session, sequence):
                return send('velocity', session_id=session, sequence=sequence, linear_mps=.05, angular_radps=0.)

            self.assertEqual(self.body.observe().owner.value, Owner.NEUTRAL)
            self.assertEqual(send('acquire')['reason'], 'local_authorization_required')
            self.assertFalse(move('unauthorized', 1)['ok'])
            self.body.sport.Move.assert_not_called()

            now[0] += .01
            self.data['grant'] = {**authorized, 'issued_at': now[0]}
            self.data['guard']['directions'] = []
            self.assertEqual(send('acquire')['reason'], 'obstacle')
            self.data['guard']['directions'] = ['forward']
            session = send('acquire')['session_id']
            self.assertTrue(move(session, 1)['ok'])

            now[0] += .01
            self.data['owner'] = {**centered, 'value': 'owned', 'reason': 'handset_active',
                                  'observed_at': now[0]}
            self.assertEqual(send('status')['control']['reason'], 'control_owned')
            self.body.sport.StopMove.assert_not_called()
            now[0] += .01
            self.data['owner'] = {**centered, 'neutral_epoch': f'{producer}:4', 'observed_at': now[0]}
            self.data['feedback']['observed_at'] = now[0]
            self.assertFalse(move(session, 2)['ok'])
            self.assertEqual(send('acquire')['reason'], 'local_authorization_required')
            self.body.sport.Move.assert_called_once_with(.05, 0., 0.)
            self.body.sport.StopMove.assert_not_called()

    def test_centered_feedback_timeout_stops_and_rejects_late_move(self):
        now = [time.monotonic()]
        self.data['owner'] = {
            'value': 'unknown', 'reason': 'handset_neutral_release_unconfirmed',
            'source': HANDSET_SOURCE, 'neutral_epoch': '458b4d06-0e3f-409b-926f-4ec72efd5ace:2',
            'observed_at': now[0], 'frame_count': 12, 'source_tick': 33,
        }
        self.body.sport.Move.return_value = self.body.sport.StopMove.return_value = 0
        with patch.object(manual, 'guard_ready', return_value=True), \
                patch.object(manual, 'trusted_file', side_effect=lambda _path: json.dumps(self.data)), \
                closing(manual.ManualReceiver(self.body, clock=lambda: now[0])) as receiver:
            now[0] += .01
            self.data['grant']['issued_at'] = now[0]
            session = receiver.arbiter.acquire({})[1]['session_id']
            move = dict(session_id=session, sequence=1, linear_mps=.05, angular_radps=0.)
            self.assertEqual(receiver.arbiter.velocity(move)[0], 200)
            now[0] += manual.LIMITS.max_sample_age_s + .01
            # 原厂手柄证据过期时让其他两路的观测仍然新鲜，独立覆盖手柄断流门禁。
            self.data['feedback']['observed_at'] = now[0]
            self.data['guard']['observed_at'] = now[0]
            self.assertEqual(receiver.arbiter.state()['reason'], 'control_unknown')
            self.body.sport.StopMove.assert_called_once_with()
            self.assertEqual(receiver.arbiter.velocity({**move, 'sequence': 2})[1]['reason'], 'invalid_session')
            self.body.sport.Move.assert_called_once_with(.05, 0., 0.)

    def test_unitree_target_ignores_legacy_receiver_stop_and_keeps_grant_limit(self):
        # 历史快照中若还有凭证字段，接收器仍只读取当前所需的现场反馈和方向。
        self.data['receiver_stop'] = {'receiver_id':'mock-a2-body','expires_at':0}
        now = [20.]
        with patch.object(manual, 'guard_ready', return_value=True), \
                patch.object(manual, 'trusted_file', side_effect=lambda _path: json.dumps(self.data)):
            receiver = manual.ManualReceiver(self.body, clock=lambda: now[0])
            try:
                now[0] = 20.125
                for field in ('owner', 'feedback', 'guard'):
                    self.data[field]['observed_at'] = now[0]
                self.data['grant'].update(issued_at=now[0], expires_at=20.375)
                reply = receiver.handle({'op':'acquire', 'schema_version':1})
                self.assertTrue(reply['ok'], reply)
                self.assertEqual(reply['expires_in_ms'], 250)
                self.body.sport.Move.assert_not_called()
            finally:
                receiver.close()

    def test_cgroup_requires_system_slice(self):
        for cgroup, valid in [('0::/system.slice/a2-autonomy.service\n',True),
                              ('0::/user.slice/a2-autonomy.service\n',False),
                              ('4:name=systemd:/system.slice/a2-autonomy.service\n',True)]:
            with patch.object(Path,'read_text',return_value=cgroup):
                self.assertEqual(manual.in_autonomy_cgroup(),valid)


if __name__ == '__main__':
    unittest.main()
