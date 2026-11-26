import io
import math
import os
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from scripts.publish_a2_platform_state import (
    Sample, Samples, attitude_values, battery_values, platform_details, platform_state, run_sdk_reader,
    sport_values,
)


def details_message():
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=""),
        primary_battery=SimpleNamespace(received_mono_ns=0),
        secondary_battery=SimpleNamespace(received_mono_ns=0),
        velocity_received_mono_ns=0, attitude_received_mono_ns=0,
    )


class PlatformDetailsTests(unittest.TestCase):
    def test_sdk_boundary_routes_four_named_subscriptions_to_separate_sources(self):
        # 走真实管道和采样解码，用各不相同的电池值检测错接、覆盖或两路合并。
        messages = {
            "rt/lf/bmsstate": SimpleNamespace(soc=33, soh=98, cycle=14),
            "rt/lf/secondary_bmsstate": SimpleNamespace(soc=71, soh=91, cycle=20),
            "rt/lf/sportmodestate": SimpleNamespace(mode=2, velocity=[1, 2, 3], yaw_speed=0.4),
            "rt/lf/lowstate": SimpleNamespace(imu_state=SimpleNamespace(rpy=[0.1, -0.2, 0.3])),
        }
        subscribers = {}
        def subscribe(topic, msg_type):
            subscriber = Mock()
            subscriber.Init.side_effect = lambda handler, _: handler(messages[topic])
            subscribers[topic] = (subscriber, msg_type)
            return subscriber

        # 仅提供四种只读依赖，意外引入 SDK 写端或运动客户端会使测试失败。
        init = Mock()
        bms_type, low_type, sport_type = object(), object(), object()
        modules = {
            "unitree_sdk2py.core.channel": SimpleNamespace(ChannelFactoryInitialize=init,
                                                          ChannelSubscriber=subscribe),
            "unitree_sdk2py.idl.unitree_go.msg.dds_": SimpleNamespace(SportModeState_=sport_type),
            "unitree_sdk2py.idl.unitree_hg.msg.dds_": SimpleNamespace(BmsState_=bms_type,
                                                                     LowState_=low_type),
        }
        reader, writer = os.pipe()
        with os.fdopen(reader, "rb") as stream:
            with os.fdopen(writer, "wb"), patch.dict(sys.modules, modules), \
                    patch.object(sys, "stdin", SimpleNamespace(buffer=io.BytesIO())):
                args = SimpleNamespace(sample_fd=writer, unitree_domain_id=0, network_interface="eth0")
                run_sdk_reader(args)
            samples = Samples()
            samples.read(stream)
        expected_types = {"rt/lf/sportmodestate": sport_type, "rt/lf/bmsstate": bms_type,
                          "rt/lf/secondary_bmsstate": bms_type, "rt/lf/lowstate": low_type}
        self.assertEqual({topic: pair[1] for topic, pair in subscribers.items()}, expected_types)
        init.assert_called_once_with(0, "eth0")
        for subscriber, _ in subscribers.values():
            subscriber.Close.assert_called_once_with()
        self.assertEqual({name: sample.values for name, sample in samples.latest().items()}, {
            "primary_battery": [33, 98, 14], "secondary_battery": [71, 91, 20],
            "sport": [2, 1.0, 2.0, 3.0, 0.4], "attitude": [0.1, -0.2, 0.3],
        })

    def test_reports_independent_fields_and_preserves_legacy_mode(self):
        # SOC 哨兵值不能遮蔽同帧的 SOH；即使有电流及状态字也不能推断充电。
        battery = SimpleNamespace(soc=255, soh=100, cycle=0, current=2406, bmsstate=[1] * 5)
        self.assertEqual(battery_values(battery), [None, 100, 0])
        self.assertEqual(sport_values(SimpleNamespace(mode=1)), [1, None, None, None, None])
        values = sport_values(SimpleNamespace(mode=1, velocity=[0, float("inf"), -0.2], yaw_speed=0))
        self.assertEqual(values, [1, 0, None, -0.2, 0])
        self.assertEqual(attitude_values(SimpleNamespace(imu_state=SimpleNamespace(rpy=[]))),
                         [None, None, None])

    def test_stale_primary_battery_does_not_hide_secondary_or_refresh_attitude(self):
        end = 2_000_000_000
        fresh_time = end - 20_000_000
        samples = {
            "primary_battery": Sample(3, end - 500_000_000, [90, 100, 31]),
            "secondary_battery": Sample(2, fresh_time, [0, None, 0]),
            "sport": Sample(7, fresh_time, [1, 0.0, None, -0.2, 0.0]),
            "attitude": Sample(6, end + 1, [0, 0, 0]),
        }
        msg = platform_details(details_message, object(), "a2", "one", samples, end, 500)
        self.assertEqual(msg.max_age_ms, 500)
        self.assertEqual(msg.header.frame_id, "")
        self.assertEqual((msg.primary_battery.received_mono_ns, msg.primary_battery.soc_percent),
                         (0, -1))
        battery = msg.secondary_battery
        self.assertEqual((battery.received_mono_ns, battery.soc_percent, battery.soh_percent,
                          battery.cycle_count, battery.charging_state),
                         (fresh_time, 0, -1, 0, "unknown"))
        self.assertEqual(msg.velocity_received_mono_ns, fresh_time)
        self.assertEqual((msg.velocity_mps[0], msg.velocity_mps[2], msg.yaw_rate_radps), (0, -0.2, 0))
        self.assertTrue(math.isnan(msg.velocity_mps[1]))
        self.assertEqual(msg.attitude_received_mono_ns, 0)
        self.assertTrue(all(math.isnan(x) for x in msg.rpy_rad))

    def test_per_source_order_and_existing_state_contract(self):
        samples = Samples()
        samples.read(io.BytesIO(
            b'["sport",100,[2,1.0,2.0,3.0,0.4]]\n'
            b'["attitude",50,[0.5,0.1,2.5]]\n'
            b'["attitude",40,[1.0,1.0,1.0]]\n'
            b'["sport",100,[3,9.0,9.0,9.0,0.9]]\n'
        ))
        latest = samples.latest()
        details = platform_details(details_message, object(), "a2", "one", latest, 101, 500)
        self.assertEqual(details.rpy_rad, [0.5, 0.1, 2.5])
        self.assertEqual((details.primary_battery.soc_percent, details.secondary_battery.soc_percent),
                         (-1, -1))
        twist = SimpleNamespace(linear=SimpleNamespace(), angular=SimpleNamespace())
        message = SimpleNamespace(header=SimpleNamespace(), body_velocity=twist)
        legacy = platform_state(lambda: message, object(), "a2", "one", latest["sport"], 101, 500)
        self.assertEqual((legacy.source, legacy.mode, legacy.sample_seq), ("unitree_sdk2", 2, 1))
        self.assertFalse(legacy.velocity_valid)
        self.assertFalse(legacy.battery_valid)
        self.assertTrue(math.isnan(twist.linear.x))


if __name__ == "__main__":
    unittest.main()
