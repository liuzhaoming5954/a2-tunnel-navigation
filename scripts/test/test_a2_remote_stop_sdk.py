"""独立停车适配器的回归测试，使用假 DDS 与确定性时钟，无需宇树设备或 SDK。"""

from contextlib import ExitStack
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import a2_remote_stop_sdk as sdk


class HardwareTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.t = 100.
        self.stack.enter_context(patch.object(sdk, "monotonic", side_effect=lambda: self.t))
        self.keys, self.bad, self.events = [], [], []
        self.freshness = []
        self.callbacks = {}
        self.client = MagicMock()
        self.client.GetApiVersion.return_value = sdk.API_VERSION
        self.client.GetServerApiVersion.side_effect = self.version
        self.client.StopMove.return_value = 0
        self.sport_cls = MagicMock(return_value=self.client)
        channel = SimpleNamespace(ChannelFactoryInitialize=lambda domain, name: self.events.append((domain, name)),
                                  ChannelSubscriber=self.subscribe)
        self.stack.enter_context(patch.dict(sys.modules, {
            "unitree_sdk2py.core.channel": channel,
            "unitree_sdk2py.idl.unitree_hg.msg.dds_": SimpleNamespace(LowState_=object),
            "unitree_sdk2py.idl.unitree_go.msg.dds_": SimpleNamespace(SportModeState_=object),
            "unitree_sdk2py.a2.sport.sport_client": SimpleNamespace(SportClient=self.sport_cls),
        }))

    def version(self):
        self.events.append("checked")
        return 0, sdk.API_VERSION

    def subscribe(self, topic, _type):
        self.events.append(topic)
        return SimpleNamespace(Init=lambda callback: self.callbacks.update({topic: callback}),
                               Close=lambda: self.events.append(("closed", topic)))

    def hardware(self, observe_only=False):
        def receive(keys, *, fresh=True):
            self.keys.append(keys)
            self.freshness.append(fresh)
        hw = sdk.StopHardware("test0", 7, receive, self.bad.append, observe_only=observe_only)
        self.addCleanup(hw.close)
        hw.start()
        return hw

    def low(self, step, keys=0):
        data = bytearray(40)
        data[2:4] = keys.to_bytes(2, "little")
        # 将某个摇杆置为 NaN，确认按键路径不依赖遥控轴字段。
        data[4:8] = bytes.fromhex("0000c07f")
        return SimpleNamespace(wireless_remote=data, tick=step,
                               imu_state=SimpleNamespace(quaternion=(1, 0, 0, 0), gyroscope=(0, 0, 0)))

    def body(self, step):
        return SimpleNamespace(stamp=SimpleNamespace(sec=11 + step//20, nanosec=(step % 20)*50_000_000), mode=3, gait_type=0, error_code=0,
                               position=(1., 2., .44), velocity=(0., 0., 0.), yaw_speed=0.)

    def frame(self, step, *, change_body=None, change_imu=None, send_body=True, send_imu=True):
        self.t = 100. + step / 20.
        body, low = self.body(step), self.low(step)
        for sample, changes in ((body, change_body), (low.imu_state, change_imu)):
            for field, value in (changes or {}).items():
                setattr(sample, field, value)
        if send_body:
            self.callbacks[sdk.SPORT_TOPIC](body)
        if send_imu:
            self.callbacks[sdk.REMOTE_TOPIC](low)

    def test_learning_has_only_radio_subscription_and_never_constructs_a_client(self):
        hw = self.hardware(observe_only=True)
        self.assertEqual(self.events, [(7, "test0"), sdk.REMOTE_TOPIC])
        radio = self.callbacks[sdk.REMOTE_TOPIC]
        for keys in (0, 0x8100, 0):
            radio(self.low(1, keys))
        self.assertEqual(self.keys, [0, 0x8100, 0])
        self.assertEqual(self.freshness, [True, True, True])
        with self.assertRaises(RuntimeError):
            hw.stop_move()
        hw.close()
        radio(self.low(2, 1))
        self.assertEqual(len(self.keys), 3)
        self.sport_cls.assert_not_called()

    def test_readonly_standstill_reports_the_sample_time_and_expires(self):
        reader = sdk.ReadOnlyStandstill("test0", 7)
        self.addCleanup(reader.close)
        reader.start()
        self.assertEqual(self.events, [(7, "test0"), sdk.SPORT_TOPIC, sdk.REMOTE_TOPIC])
        self.assertIsNone(reader.feedback())
        for step in range(14):
            # 停稳不依赖同一帧中的遥控按键数据。
            self.t = 100. + step / 20.
            self.callbacks[sdk.SPORT_TOPIC](self.body(step))
            low = self.low(step)
            del low.wireless_remote
            self.callbacks[sdk.REMOTE_TOPIC](low)
        observed_at = self.t
        self.t += .1
        self.assertEqual(reader.feedback(), {"value": "stopped", "observed_at": observed_at,
                                             "sport_mode": 3, "gait_type": 0, "error_code": 0})
        self.t += .12
        self.assertIsNone(reader.feedback())
        self.sport_cls.assert_not_called()

    def test_readonly_standstill_rejects_motion_and_nonstanding_mode(self):
        reader = sdk.ReadOnlyStandstill("test0", 7)
        self.addCleanup(reader.close)
        reader.start()
        for step in range(14):
            self.frame(step)
        self.assertIsNotNone(reader.feedback())
        self.frame(14, change_body={"velocity": (.03, 0, 0)})
        self.assertIsNone(reader.feedback())
        for step in range(15, 29):
            self.frame(step)
        self.assertIsNotNone(reader.feedback())
        self.frame(29, change_body={"mode": sdk.STAND_DOWN})
        self.assertIsNone(reader.feedback())
        reader.close()
        self.assertIsNone(reader.feedback())
        self.sport_cls.assert_not_called()

    def test_ready_before_subscription_passthrough_code_and_closed_guard(self):
        hw = self.hardware()
        self.assertEqual(self.events, [(7, "test0"), "checked", sdk.SPORT_TOPIC, sdk.REMOTE_TOPIC])
        self.assertEqual([c.args for c in self.client.SetTimeout.call_args_list], [(1.,), (.15,)])
        self.client.StopMove.return_value = 4201
        self.assertEqual(hw.stop_move(), 4201)
        self.client.StopMove.return_value = False
        with self.assertRaises(RuntimeError):
            hw.stop_move()
        hw.close()
        with self.assertRaises(RuntimeError):
            hw.stop_move()
        self.assertEqual(self.events[-2:], [("closed", sdk.REMOTE_TOPIC), ("closed", sdk.SPORT_TOPIC)])

    def test_wrong_version_or_failed_version_query_does_not_subscribe(self):
        for value in ((0, "wrong"), (4201, sdk.API_VERSION), (False, sdk.API_VERSION)):
            self.client.GetServerApiVersion.side_effect = None
            self.client.GetServerApiVersion.return_value = value
            with self.assertRaises(RuntimeError):
                self.hardware()
        self.assertFalse(self.callbacks)
        self.client.StopMove.assert_not_called()

    def test_subscription_failure_releases_partial_setup(self):
        calls = []
        def subscribe(topic, _type):
            def init(_callback):
                if topic == sdk.REMOTE_TOPIC:
                    raise ValueError("模拟 DDS 初始化失败")
            return SimpleNamespace(Init=init, Close=lambda: calls.append(topic))
        with patch.object(sys.modules["unitree_sdk2py.core.channel"], "ChannelSubscriber", side_effect=subscribe):
            with self.assertRaises(ValueError):
                self.hardware()
        self.assertEqual(calls, [sdk.REMOTE_TOPIC, sdk.SPORT_TOPIC])
        self.client.StopMove.assert_not_called()

    def test_valid_key_is_reported_even_with_bad_imu_and_bad_key_is_never_neutral(self):
        self.hardware()
        radio = self.callbacks[sdk.REMOTE_TOPIC]
        low = self.low(5, 0x0101)
        del low.imu_state
        radio(low)
        low.wireless_remote = bytes(3)
        radio(low)
        self.assertEqual(self.keys, [0x0101])
        self.assertEqual(len(self.bad), 1)

    def test_requires_post_trigger_overlap_and_fresh_distinct_samples(self):
        hw = self.hardware()
        for step in range(21):
            self.frame(step, send_imu=step >= 11)
        self.assertFalse(hw.still_since(100.))
        self.frame(21)
        self.assertTrue(hw.still_since(100.))
        self.assertFalse(hw.still_since(self.t - .49))
        self.t += .25
        self.assertFalse(hw.still_since(100.))
        for step in range(30, 38):
            self.frame(step)
        self.assertFalse(hw.still_since(100.))
        self.t += .7
        # 多次读取相同缓存不会给任一路增加可用保持时间。
        self.assertFalse(hw.still_since(100.))

    def test_motion_fault_tilt_and_drift_each_break_still_observation(self):
        cases = [
            ({"velocity": (0., 0., .020)}, {}),
            ({"position": (1., 2., .50)}, {}),
            ({"yaw_speed": float("nan")}, {}),
            ({"error_code": 1}, {}),
            ({"mode": 0}, {}),
            ({}, {"quaternion": (0., 0., 0., 0.)}),
            ({}, {"quaternion": (0., 1., 0., 0.)}),
            ({}, {"gyroscope": (.1, 0., 0.)}),
        ]
        for body, imu in cases:
            with self.subTest(body=body, imu=imu):
                hw = self.hardware()
                for step in range(11):
                    self.frame(step)
                self.assertTrue(hw.still_since(100.))
                self.frame(11, change_body=body, change_imu=imu)
                self.assertFalse(hw.still_since(100.))
                for step in range(12, 24):
                    self.frame(step)
                self.assertTrue(hw.still_since(100.))
                hw.close()

    def test_stale_or_reordered_source_sequence_cannot_extend_hold(self):
        hw = self.hardware()
        for step in range(21):
            self.frame(step)
        self.assertTrue(hw.still_since(100.))
        for step in (20, 5):
            self.t += .01
            self.callbacks[sdk.SPORT_TOPIC](self.body(step))
            self.callbacks[sdk.REMOTE_TOPIC](self.low(step, 0x0100))
            self.assertFalse(hw.still_since(100.))
        self.assertEqual(self.keys[-2:], [0x100, 0x100])
        # 所有设备帧仍比既有记录旧。即便按正常频率重放半秒以上，也不得确认停稳。
        for step in range(6, 20):
            self.t += .05
            self.callbacks[sdk.SPORT_TOPIC](self.body(step))
            self.callbacks[sdk.REMOTE_TOPIC](self.low(step))
        self.assertFalse(hw.still_since(100.))

    @staticmethod
    def dds_fields(step, *, writer=37, ns=None):
        # 模拟当前 PC2，设备字段为全零，DDS 写者通过 SampleInfo 提供发布时刻。
        return {"stamp": SimpleNamespace(sec=0, nanosec=0),
                "sample_info": SimpleNamespace(valid_data=True, publication_handle=writer,
                                               source_timestamp=10**18 + (step*50_000_000 if ns is None else ns))}

    def posture_frame(self, step, *, mode=1, keys=0, modify=None, send_body=True, send_low=True):
        """以同一对 DDS 写者模拟两种模式，保证模式切换测试不依赖换源或位置跳变。"""
        self.t = 100. + step / 20.
        body = self.body(step)
        for name, value in self.dds_fields(step).items():
            setattr(body, name, value)
        body.mode = mode
        # 下趴运动状态复现现场非零合速度，站立沿用真实的旧判据。
        if mode == 1:
            body.velocity = (.06681709, -.10037690, -.11499640)
        low = self.low(7500000 + 50 * step, keys)
        low.sample_info = self.dds_fields(step, writer=53)["sample_info"]
        low.mode_machine, low.mode_pr = 1, 0
        noise = (step % 3) - 1
        low.imu_state.gyroscope = (.002 * noise, .003, -.001)
        low.imu_state.accelerometer = (.008 * noise, .02, 9.81)
        # 仿照 HG 的 35 个槽位，前 12 个为四条腿。小幅反馈变化代表活跃的读数。
        angles = (-.40, 1.10, -2.64, .42, 1.18, -2.65) * 2
        low.motor_state = [SimpleNamespace(q=q + noise * .000025, dq=noise * .008) for q in angles]
        low.motor_state.extend(SimpleNamespace(q=0., dq=0.) for _ in range(23))
        if modify is not None:
            modify(body, low)
        if send_body:
            self.callbacks[sdk.SPORT_TOPIC](body)
        if send_low:
            self.callbacks[sdk.REMOTE_TOPIC](low)

    def motion_reader(self):
        reader = sdk.ReadOnlyStandstill("test0", 7)
        self.addCleanup(reader.close)
        reader.start()
        return reader

    @staticmethod
    def walking(body, low):
        body.velocity = (.04, .0, .0)
        body.position = (1.01, 2., .44)

    def test_motion_feedback_tracks_only_confirmed_stop_and_positive_motion(self):
        reader = self.motion_reader()
        for step in range(10):
            self.posture_frame(step, mode=3)
        self.assertIsNone(reader.motion_feedback())
        for step in range(10, 14):
            self.posture_frame(step, mode=3)
        self.assertEqual(reader.motion_feedback(), reader.feedback())
        self.assertEqual(reader.motion_feedback()["value"], "stopped")

        self.posture_frame(14, mode=3, modify=self.walking)
        moving = {"value": "moving", "observed_at": self.t, "sport_mode": 3, "gait_type": 0, "error_code": 0}
        self.assertEqual(reader.motion_feedback(), moving)
        self.t += .02
        self.assertEqual(reader.motion_feedback(), moving)
        self.assertIsNone(reader.feedback())

        def turning(body, low):
            body.yaw_speed = .08
            low.imu_state.gyroscope = (0., 0., .08)

        self.posture_frame(15, mode=3, modify=turning)
        self.assertEqual(reader.motion_feedback()["value"], "moving")
        for step in range(16, 23):
            self.posture_frame(step, mode=3)
            self.assertIsNone(reader.motion_feedback())
        for step in range(23, 29):
            self.posture_frame(step, mode=3)
        self.assertEqual(reader.motion_feedback(), reader.feedback())
        self.assertEqual(reader.motion_feedback()["value"], "stopped")
        reader.close()
        self.assertIsNone(reader.motion_feedback())
        self.sport_cls.assert_not_called()

    def test_manual_threshold_accepts_nonzero_speed_only_after_continuous_hold(self):
        reader = self.motion_reader()

        def low_speed(body, _low):
            body.velocity = (.006, .008, 0.)

        def above_manual_threshold(body, _low):
            # 平面速度不足 0.01，但三轴合速度超限，仍须打断确认窗口。
            body.velocity = (.008, .005, .006)

        for step in range(14):
            self.posture_frame(step, mode=3, modify=low_speed)
        self.assertEqual(reader.motion_feedback()["value"], "stopped")
        self.posture_frame(14, mode=3, modify=above_manual_threshold)
        self.assertEqual(reader.motion_feedback()["value"], "moving")
        self.assertIsNone(reader.feedback())
        for step in range(15, 24):
            self.posture_frame(step, mode=3, modify=low_speed)
        self.assertIsNone(reader.motion_feedback())
        for step in range(24, 29):
            self.posture_frame(step, mode=3, modify=low_speed)
        self.assertEqual(reader.motion_feedback()["value"], "stopped")

        # 实体 A 键的独立守护保留 0.016 米/秒判据。
        guard = self.hardware()
        began = self.t
        for step in range(29, 43):
            self.posture_frame(step, mode=3, modify=above_manual_threshold)
        self.assertTrue(guard.still_since(began))

    def test_opt_in_reports_fresh_settling_without_changing_stillness_confirmation(self):
        reader = self.motion_reader()
        for step in range(4):
            self.posture_frame(step, mode=3, modify=self.walking)
        self.assertEqual(reader.motion_feedback(include_settling=True)["value"], "moving")

        for step in range(4, 8):
            self.posture_frame(step, mode=3)
        self.assertIsNone(reader.motion_feedback())
        settling = reader.motion_feedback(include_settling=True)
        self.assertEqual(settling["value"], "settling")
        self.assertEqual(settling["observed_at"], self.t)
        self.assertIsNone(reader.feedback())

        for step in range(8, 18):
            self.posture_frame(step, mode=3)
        self.assertEqual(reader.motion_feedback(include_settling=True), reader.feedback())
        self.assertEqual(reader.motion_feedback(include_settling=True)["value"], "stopped")
        self.posture_frame(18, mode=3, modify=self.walking)
        self.posture_frame(19, mode=3)
        self.assertEqual(reader.motion_feedback(include_settling=True)["value"], "settling")
        self.t += .21
        self.assertIsNone(reader.motion_feedback(include_settling=True))
        self.sport_cls.assert_not_called()

    def test_motion_feedback_requires_fresh_coherent_dual_streams(self):
        reader = self.motion_reader()
        for step in range(4):
            self.posture_frame(step, mode=3, modify=self.walking)
        self.assertEqual(reader.motion_feedback()["value"], "moving")
        for step in range(4, 7):
            self.posture_frame(step, mode=3, modify=self.walking, send_low=False)
        # 两路各自仍未过期，但最近两帧的接收时间已经无法匹配。
        self.assertIsNone(reader.motion_feedback())
        self.posture_frame(7, mode=3, modify=self.walking)
        self.assertIsNone(reader.motion_feedback())
        self.posture_frame(8, mode=3, modify=self.walking)
        self.assertEqual(reader.motion_feedback()["value"], "moving")
        self.t += .21
        self.assertIsNone(reader.motion_feedback())

    def test_motion_feedback_rejects_bad_posture_measurement_and_source(self):
        cases = {
            "姿态": lambda body, low: setattr(body, "mode", 1),
            "步态": lambda body, low: setattr(body, "gait_type", False),
            "故障码": lambda body, low: setattr(body, "error_code", 1),
            "位置": lambda body, low: setattr(body, "position", (1., math.nan, .44)),
            "速度": lambda body, low: setattr(body, "velocity", (math.inf, 0., 0.)),
            "偏航速度": lambda body, low: setattr(body, "yaw_speed", math.nan),
            "机身写者": lambda body, low: setattr(body.sample_info, "publication_handle", 38),
            "机身源时间": lambda body, low: setattr(body.sample_info, "source_timestamp", 10**18),
            "低层帧号": lambda body, low: setattr(low, "tick", 7500100),
            "低层源时间": lambda body, low: setattr(low.sample_info, "source_timestamp", 10**18),
            "低层无效": lambda body, low: setattr(low.sample_info, "valid_data", False),
            "低层角速度": lambda body, low: setattr(low.imu_state, "gyroscope", (0., math.nan, 0.)),
            "低层加速度": lambda body, low: setattr(low.imu_state, "accelerometer", (0., 0., 0.)),
            "过大倾斜": lambda body, low: setattr(low.imu_state, "quaternion", (0., 1., 0., 0.)),
        }
        for name, corrupt in cases.items():
            with self.subTest(name=name):
                self.t = 100.
                with ExitStack() as close:
                    reader = self.motion_reader()
                    close.callback(reader.close)
                    for step in range(3):
                        self.posture_frame(step, mode=3, modify=self.walking)
                    self.assertEqual(reader.motion_feedback()["value"], "moving")

                    def corrupted_motion(body, low):
                        self.walking(body, low)
                        corrupt(body, low)

                    self.posture_frame(3, mode=3, modify=corrupted_motion)
                    self.assertIsNone(reader.motion_feedback())
        self.sport_cls.assert_not_called()

    def test_motion_feedback_does_not_reuse_legacy_hold_across_bad_dds_metadata(self):
        reader = self.motion_reader()
        for step in range(14):
            self.posture_frame(step, mode=3)
        self.assertEqual(reader.motion_feedback()["value"], "stopped")
        self.posture_frame(14, mode=3, modify=lambda body, low: setattr(low.sample_info, "valid_data", False))
        for step in range(15, 20):
            self.posture_frame(step, mode=3)
        # 旧停稳接口维持兼容，新运动接口要求低层 DDS 证据重新覆盖 0.50 秒。
        self.assertIsNotNone(reader.feedback())
        self.assertIsNone(reader.motion_feedback())
        for step in range(20, 28):
            self.posture_frame(step, mode=3)
        self.assertEqual(reader.motion_feedback()["value"], "stopped")

    def test_motion_feedback_preserves_paired_time_after_transport_delay(self):
        reader = self.motion_reader()
        for step in range(6):
            self.posture_frame(step, mode=3, modify=self.walking)

        def delayed_low(body, low):
            self.walking(body, low)
            low.sample_info.source_timestamp -= 40_000_000

        self.posture_frame(6, mode=3, modify=delayed_low)
        report = reader.motion_feedback()
        self.assertEqual(report["value"], "moving")
        # 最快延时基线容许千分之一的时钟漂移，估算只留很小的数值余量。
        self.assertAlmostEqual(report["observed_at"], self.t - .04, delta=.0001)
        self.t += .17
        self.assertIsNone(reader.motion_feedback())

    def test_prone_accepts_frozen_sport_velocity_after_a_complete_post_trigger_window(self):
        hw = self.hardware()
        for step in range(20):
            self.posture_frame(step)
        trigger = 100.54
        self.assertFalse(hw.still_since(trigger))
        for step in range(20, 24):
            self.posture_frame(step)
        self.assertTrue(hw.still_since(trigger))

    def test_every_posture_change_starts_a_new_window_in_both_directions(self):
        hw = self.hardware()
        for step in range(14):
            self.posture_frame(step, mode=3)
        self.assertTrue(hw.still_since(100.))
        for first_step, mode in ((14, 1), (28, 3)):
            with self.subTest(mode=mode):
                for step in range(first_step, first_step + 10):
                    self.posture_frame(step, mode=mode)
                    self.assertFalse(hw.still_since(100.))
                for step in range(first_step + 10, first_step + 14):
                    self.posture_frame(step, mode=mode)
                self.assertTrue(hw.still_since(100.))

    def test_prone_rejects_invalid_modes_unusable_sensors_and_motion(self):
        def zero_leg(body, low):
            for motor in low.motor_state[6:9]:
                motor.q = motor.dq = 0.

        cases = {
            "未知模式": lambda body, low: setattr(body, "mode", 2),
            "步态": lambda body, low: setattr(body, "gait_type", 1),
            "故障": lambda body, low: setattr(body, "error_code", 2),
            "电机槽不足": lambda body, low: setattr(low, "motor_state", low.motor_state[:11]),
            "整条腿占位": zero_leg,
            "关节读数缺失": lambda body, low: delattr(low.motor_state[11], "dq"),
            "关节位置非数": lambda body, low: setattr(low.motor_state[7], "q", float("nan")),
            "关节速度非数": lambda body, low: setattr(low.motor_state[11], "dq", float("inf")),
            "关节超速": lambda body, low: setattr(low.motor_state[11], "dq", -.101),
            "IMU 倾覆": lambda body, low: setattr(low.imu_state, "quaternion", (0., 1., 0., 0.)),
            "IMU 转动": lambda body, low: setattr(low.imu_state, "gyroscope", (0., 0., -.06)),
            "侧向加速": lambda body, low: setattr(low.imu_state, "accelerometer", (.51, 0., 9.81)),
            "加速度缺失": lambda body, low: delattr(low.imu_state, "accelerometer"),
        }
        for label, modify in cases.items():
            with self.subTest(label=label):
                hw = self.hardware()
                for step in range(14):
                    self.posture_frame(step)
                self.assertTrue(hw.still_since(100.))
                self.posture_frame(14, modify=modify, keys=0x100)
                self.assertFalse(hw.still_since(100.))
                # 任一姿态证据失效时，仍须交付合法的急停按键。
                self.assertEqual(self.keys[-1], 0x100)
                hw.close()

    def test_prone_joint_excursion_still_counts_after_returning_to_the_start_position(self):
        hw = self.hardware()
        for step in range(14):
            self.posture_frame(step)
        self.assertTrue(hw.still_since(100.))
        self.posture_frame(14, modify=lambda body, low: setattr(low.motor_state[8], "q", -2.642))
        self.posture_frame(15)
        self.assertFalse(hw.still_since(100.))
        for step in range(16, 30):
            self.posture_frame(step)
        self.assertTrue(hw.still_since(100.))

    def test_prone_requires_live_imu_and_live_joint_payloads_even_with_fresh_ticks(self):
        def frozen_joints(body, low):
            for i, motor in enumerate(low.motor_state[:12]):
                motor.q = (-.4, 1.1, -2.64)[i % 3]
                motor.dq = 0.

        def frozen_imu(body, low):
            low.imu_state.gyroscope = (.002, .003, -.001)
            low.imu_state.accelerometer = (.008, .02, 9.81)

        for modify in (frozen_joints, frozen_imu):
            with self.subTest(modify=modify.__name__):
                hw = self.hardware()
                for step in range(20):
                    self.posture_frame(step, modify=modify)
                self.assertFalse(hw.still_since(100.))
                hw.close()

    def test_prone_acceleration_is_compared_in_the_body_frame(self):
        # 绕 X 轴侧倾 6° 时，静态重力在机身 Y 轴有分量；反向同模读数应被拒绝。
        roll = math.radians(6)
        def tilted(body, low):
            low.imu_state.quaternion = (math.cos(roll / 2), math.sin(roll / 2), 0., 0.)
            low.imu_state.accelerometer = (0., 9.81 * math.sin(roll), 9.81 * math.cos(roll))

        hw = self.hardware()
        for step in range(14):
            self.posture_frame(step, modify=tilted)
        self.assertTrue(hw.still_since(100.))
        def reversed_gravity(body, low):
            tilted(body, low)
            x, y, z = low.imu_state.accelerometer
            low.imu_state.accelerometer = (x, -y, z)

        self.posture_frame(14, modify=reversed_gravity)
        self.assertFalse(hw.still_since(100.))

    def test_prone_recent_frozen_imu_cannot_borrow_an_older_change_in_the_window(self):
        hw = self.hardware()
        for step in range(14):
            self.posture_frame(step)
        self.assertTrue(hw.still_since(100.))

        def frozen_imu(body, low):
            # 仅固定 IMU 的两个传感量；关节、两路时戳和低层帧号照常推进。
            low.imu_state.gyroscope = (0., .003, -.001)
            low.imu_state.accelerometer = (0., .02, 9.81)

        for step in range(14, 19):
            self.posture_frame(step, modify=frozen_imu)
        self.assertFalse(hw.still_since(100.))
        self.posture_frame(19)
        self.assertFalse(hw.still_since(100.))
        for step in range(20, 34):
            self.posture_frame(step)
        self.assertTrue(hw.still_since(100.))

    def test_prone_cannot_reuse_earlier_lowstate_writer_or_ignore_latest_joint_motion(self):
        hw = self.hardware()
        for step in range(14):
            self.posture_frame(step)
        self.assertTrue(hw.still_since(100.))
        # 最近一个机身样本仍在时效内，但更新的低层反馈已经报告关节明显位移。
        self.posture_frame(14, send_body=False,
                           modify=lambda body, low: setattr(low.motor_state[8], "q", -2.643))
        self.assertFalse(hw.still_since(100.))
        for step in range(15, 29):
            self.posture_frame(step)
        self.assertTrue(hw.still_since(100.))
        # 帧号与接收间隔连续也不能拼接两个低层写者的半秒证据。
        self.posture_frame(29, modify=lambda body, low: setattr(low.sample_info, "publication_handle", 54))
        self.assertFalse(hw.still_since(100.))

    def test_prone_loses_confirmation_on_stream_loss_or_stale_data(self):
        cases = {
            "低层断流": {"send_low": False},
            "机身断流": {"send_body": False},
            "低层计数陈旧": {"modify": lambda body, low: setattr(low, "tick", 7500000)},
            "机身时间陈旧": {"modify": lambda body, low: setattr(body.sample_info, "source_timestamp", 10**18)},
            "低层时间陈旧": {"modify": lambda body, low: setattr(low.sample_info, "source_timestamp", 10**18)},
        }
        for label, changes in cases.items():
            with self.subTest(label=label):
                hw = self.hardware()
                for step in range(14):
                    self.posture_frame(step)
                self.assertTrue(hw.still_since(100.))
                for step in range(14, 21):
                    self.posture_frame(step, **changes)
                self.assertFalse(hw.still_since(100.))
                hw.close()

    def test_standing_without_joint_or_acceleration_feedback_keeps_original_speed_limit(self):
        # 旧站立输入只带有效姿态、角速度与运动状态。新下趴字段仅在下趴时必需。
        for speed, expected in ((.016, True), (.01601, False)):
            with self.subTest(speed=speed):
                hw = self.hardware()
                for step in range(14):
                    self.frame(step, change_body={"velocity": (0., 0., speed)})
                self.assertEqual(hw.still_since(100.), expected)
                hw.close()

    def test_zero_device_stamp_uses_valid_dds_writer_and_source_duration(self):
        hw = self.hardware()
        for step in range(11):
            self.frame(step, change_body=self.dds_fields(step))
        self.assertTrue(hw.still_since(100.))
        # writer 切换打断当前证据。另一条生产链又转回正设备时戳时也必须重新保持。
        self.frame(11, change_body=self.dds_fields(11, writer=38))
        self.assertFalse(hw.still_since(100.))
        for step in range(12, 23):
            self.frame(step, change_body=self.dds_fields(step, writer=38))
        self.assertTrue(hw.still_since(100.))
        self.frame(23)
        self.assertFalse(hw.still_since(100.))
        for step in range(24, 35):
            self.frame(step)
        self.assertTrue(hw.still_since(100.))

    def test_invalid_data_or_malformed_device_stamp_never_falls_back(self):
        for field, value in (("valid_data", False), ("publication_handle", 0),
                             ("source_timestamp", 0), ("source_timestamp", True)):
            with self.subTest(field=field):
                hw = self.hardware()
                for step in range(14):
                    fields = self.dds_fields(step)
                    setattr(fields["sample_info"], field, value)
                    self.frame(step, change_body=fields)
                self.assertFalse(hw.still_since(100.))
                hw.close()
        hw = self.hardware()
        for step in range(14):
            fields = self.dds_fields(step)
            fields["stamp"].nanosec = -1
            self.frame(step, change_body=fields)
        self.assertFalse(hw.still_since(100.))

    def test_frozen_positive_device_clock_cannot_be_supplanted_by_dds(self):
        hw = self.hardware()
        for step in range(18):
            fields = self.dds_fields(step)
            fields["stamp"].sec = 15
            self.frame(step, change_body=fields)
        self.assertFalse(hw.still_since(100.))

    def test_backlogged_fast_slow_and_replayed_dds_cannot_extend_hold(self):
        for ns_per_frame in (1, 100_000_000):
            with self.subTest(ns_per_frame=ns_per_frame):
                hw = self.hardware()
                for step in range(20):
                    self.frame(step, change_body=self.dds_fields(step, ns=step*ns_per_frame))
                self.assertFalse(hw.still_since(100.))
                hw.close()
        hw = self.hardware()
        for step in range(21):
            self.frame(step, change_body=self.dds_fields(step))
        self.assertTrue(hw.still_since(100.))
        # 中途混入不同写者，再以原写者旧时戳按正常接收频率重播，亦不得形成保持窗口。
        self.frame(21, change_body=self.dds_fields(21, writer=40))
        for step in range(22, 38):
            self.frame(step, change_body=self.dds_fields(step-18))
        self.assertFalse(hw.still_since(100.))

    def test_frozen_tick_cannot_refresh_radio_watchdog_or_hide_a_brief_stop_button(self):
        self.hardware()
        radio = self.callbacks[sdk.REMOTE_TOPIC]
        radio(self.low(10))
        self.assertFalse(self.keys)
        radio(self.low(11))
        # 使用相同的设备帧号发送中立、其他键和极短的停车键。仅推进的第二帧可续期。
        for keys in (0, 0x0002, 0x0100, 0):
            self.t += .12
            radio(self.low(11, keys))
        self.assertEqual(self.keys, [0, 0x0002, 0x0100])
        self.assertEqual(self.freshness, [True, False, False])
        self.assertFalse(self.bad)
        for tick in (8, 9, 10, 11):
            radio(self.low(tick))
        self.assertEqual(len(self.keys), 3)
        radio(self.low(12))
        self.assertEqual((self.keys[-1], self.freshness[-1]), (0, True))

    def test_first_or_missing_tick_can_trip_and_uint32_rollover_is_fresh(self):
        self.hardware()
        radio = self.callbacks[sdk.REMOTE_TOPIC]
        broken = self.low(4, 0x100)
        del broken.tick
        radio(broken)
        radio(self.low(0xffffffff, 0x100))
        radio(self.low(0, 0))
        self.assertEqual(self.keys, [0x100, 0x100, 0])
        self.assertEqual(self.freshness, [False, False, True])


if __name__ == "__main__":
    unittest.main()
