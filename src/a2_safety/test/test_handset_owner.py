"""手柄优先接管回归。使用合成 40 字节 LowState，测试代码不初始化宇树 SDK。"""

from pathlib import Path
import json
import socket
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
import manual_authority
from observe_a2_handset_owner import HandsetOwner, answer


class HandsetTests(unittest.TestCase):
    def setUp(self):
        self.now = 300.0
        self.owner = HandsetOwner(clock=lambda: self.now)

    def frame(self, tick, *, keys=0, x=0., writer=11, offset=0):
        packet = bytearray(40)
        struct.pack_into('<Hf', packet, 2, keys, x)
        info = SimpleNamespace(valid_data=True, publication_handle=writer,
                               source_timestamp=round(self.now * 1e9) + offset)
        return SimpleNamespace(tick=tick, wireless_remote=packet, sample_info=info)

    def read(self):
        # 运行生产端原有的 Unix 消息处理和客户端校验，仅将独立 systemd 服务替换为 socketpair。
        def receive(query):
            client, server = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            with client, server:
                client.settimeout(.5)
                client.send(json.dumps(query).encode())
                answer(server, self.owner, 'boot', 'robot')
                return json.loads(client.recv(2048))

        with patch.object(manual_authority, '_receive', side_effect=receive):
            return manual_authority.read_owner('boot', 'robot', now=self.now)

    def test_end_to_end_handset_activity_and_return_to_center(self):
        self.assertEqual(self.read()['reason'], 'handset_no_frames')
        self.owner.receive(self.frame(100))
        neutral = self.read()
        self.assertEqual(neutral['reason'], 'handset_neutral_release_unconfirmed')
        self.assertIsNone(neutral['epoch'])
        first_center = manual_authority.neutral_epoch(neutral)
        self.assertIsNotNone(first_center)
        self.now += .05
        self.owner.receive(self.frame(101, x=.03))
        self.assertEqual(manual_authority.neutral_epoch(self.read()), first_center)
        self.now += .05
        self.owner.receive(self.frame(102, keys=0x0100))
        button = self.read()
        self.assertEqual(button['value'], 'owned')
        self.assertEqual(button['source_tick'], 102)
        self.assertEqual(button['frame_count'], 3)
        self.assertIsNone(manual_authority.neutral_epoch(button))
        self.now += .05
        self.owner.receive(self.frame(103, x=.03))
        returned = self.read()
        self.assertEqual(returned['value'], 'unknown')
        second_center = manual_authority.neutral_epoch(returned)
        self.assertIsNotNone(second_center)
        self.assertNotEqual(second_center, first_center)
        self.now += .05
        self.owner.receive(self.frame(104, x=-.8))
        stick = self.read()
        self.assertEqual(stick['value'], 'owned')
        self.assertNotEqual(stick['epoch'], button['epoch'])
        self.assertIsNone(manual_authority.neutral_epoch(stick))
        self.now += .05
        self.owner.receive(self.frame(105))
        third_center = manual_authority.neutral_epoch(self.read())
        self.assertIsNotNone(third_center)
        self.assertNotEqual(second_center, third_center)
        self.now += .21
        stale = self.read()
        self.assertEqual(stale['reason'], 'handset_frame_stale')
        self.assertIsNone(manual_authority.neutral_epoch(stale))
        self.owner.receive(self.frame(106))
        self.assertNotEqual(manual_authority.neutral_epoch(self.read()), third_center)

    def test_source_sequence_clock_and_writer(self):
        self.owner.receive(self.frame(2**32 - 1))
        self.now += .05
        self.owner.receive(self.frame(0, x=.9))
        self.assertEqual(self.owner.report()['value'], 'owned')
        for tick, fields in ((0, {}), (3, {'writer': 12}), (4, {'offset': -1_000_000_000})):
            with self.subTest(tick=tick):
                self.now += .01
                self.owner.receive(self.frame(tick, keys=1, **fields))
                self.assertEqual(self.owner.report()['reason'], 'handset_invalid_frame')
                self.assertIsNone(self.owner.report()['observed_at'])
        self.now += .01
        malformed = self.frame(5, x=.8)
        malformed.wireless_remote = malformed.wireless_remote[:-1]
        self.owner.receive(malformed)
        self.assertEqual(self.owner.report()['frame_count'], 2)

    def test_handset_protocol_never_accepts_claim_of_release(self):
        def invalid(query):
            return {**query, 'source': manual_authority.HANDSET_SOURCE, 'value':'released',
                    'observed_at': self.now, 'producer_instance': str(self.owner.instance), 'generation': 1}

        with patch.object(manual_authority, '_receive', side_effect=invalid):
            self.assertEqual(manual_authority.read_owner('boot', 'robot', now=self.now)['value'], 'unknown')


if __name__ == '__main__':
    unittest.main()
