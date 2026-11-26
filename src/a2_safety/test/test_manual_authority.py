"""独立原厂权属的只读协议回归。所有响应均在开发机注入，测试期间不连接机器人。"""

import json
from pathlib import Path
import stat
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import manual_authority as authority


class OwnerReaderTests(unittest.TestCase):
    def observe(self, **changes):
        def receive(request):
            self.assertEqual(request['op'], 'status')
            self.assertEqual(len(request['nonce']), 32)
            return {**request, 'value': 'released', 'observed_at': 100.0,
                    'source': 'test-only', 'producer_instance': '458b4d06-0e3f-409b-926f-4ec72efd5ace',
                    'generation': 2, **changes}

        with patch.object(authority, '_receive', receive):
            return authority.read_owner('first-boot', 'receiver-a', now=100.01)

    def test_explicit_owner_and_continuity(self):
        report = self.observe()
        self.assertEqual(report['value'], 'released')
        self.assertEqual(report['observed_at'], 100)
        self.assertIsNotNone(report['epoch'])
        self.assertNotEqual(self.observe(generation=3)['epoch'], report['epoch'])
        self.assertEqual(self.observe(value='owned')['value'], 'owned')
        self.assertIsNone(self.observe(value='unknown')['epoch'])

    def test_fresh_handset_center_has_its_own_confirmed_epoch(self):
        center = {
            'source': authority.HANDSET_SOURCE, 'value': 'unknown',
            'reason': 'handset_neutral_release_unconfirmed', 'frame_count': 7, 'source_tick': 92,
        }
        report = self.observe(**center)
        self.assertEqual(report['value'], 'unknown')
        self.assertIsNone(report['epoch'])
        neutral = authority.neutral_epoch(report)
        self.assertIsNotNone(neutral)
        same_period = {**center, 'frame_count': 8, 'source_tick': 93}
        self.assertEqual(authority.neutral_epoch(self.observe(**same_period)), neutral)
        self.assertNotEqual(authority.neutral_epoch(self.observe(**same_period, generation=3)), neutral)

        for invalid in ({'generation': 0}, {'producer_instance': None}, {'source_tick': True},
                        {'frame_count': 0}, {'observed_at': 99.6}, {'nonce': 'wrong'},
                        {'source': 'other-feedback'}, {'reason': 'handset_invalid_frame'},
                        {'value': 'owned', 'reason': 'handset_active'}):
            with self.subTest(invalid=invalid):
                rejected = self.observe(**{**center, **invalid})
                self.assertIsNone(authority.neutral_epoch(rejected))

    def test_invalid_wrong_boot_peer_or_challenge_cannot_release(self):
        for fields in ({'nonce': 'stale'}, {'boot_id': 'other'}, {'receiver_id': 'other'},
                       {'value': 'ai'}, {'observed_at': True}, {'observed_at': 99.6},
                       {'observed_at': 102.0}, {'observed_at': float('nan')}, {'schema_version': True},
                       {'producer_instance': None}, {'generation': 0}, {'generation': True}):
            with self.subTest(fields=fields):
                report = self.observe(**fields)
                self.assertEqual(report['value'], 'unknown')
                self.assertIsNone(report['observed_at'])
                self.assertIsNone(report['epoch'])

    def test_late_response_does_not_refresh_sample(self):
        with patch.object(authority.time, 'monotonic', side_effect=[200, 200.20]):
            self.assertEqual(self.observe()['reason'], 'vendor_owner_stale')

    def test_source_loss_does_not_reuse_last_sample(self):
        self.assertEqual(self.observe()['value'], 'released')
        for error, reason in ((FileNotFoundError(), 'vendor_owner_unavailable'),
                              (PermissionError(), 'vendor_owner_untrusted'),
                              (ValueError(), 'vendor_owner_invalid')):
            with patch.object(authority, '_receive', side_effect=error):
                report = authority.read_owner('first-boot', 'receiver-a')
                self.assertEqual(report, {'value': 'unknown', 'source': None, 'observed_at': None,
                                          'epoch': None, 'reason': reason})

    def test_system_unit_and_duplicate_fields(self):
        self.assertTrue(authority._system_unit('0::/system.slice/a2-vendor-owner.service\n'))
        self.assertTrue(authority._system_unit('4:name=systemd:/system.slice/a2-vendor-owner.service\n'))
        self.assertFalse(authority._system_unit('0::/user.slice/system.slice/a2-vendor-owner.service\n'))
        with self.assertRaises(ValueError):
            json.loads('{"value":"owned","value":"released"}', object_pairs_hook=authority._distinct_fields)

    def test_untrusted_process_never_receives_even_a_status_request(self):
        # 即使攻击或误配的进程已经监听预期路径，也不会向无权代表原厂来源的进程询问状态。
        for uid, group in ((1000, '0::/system.slice/a2-vendor-owner.service'),
                           (61420, '0::/user.slice/app.scope')):
            with self.subTest(uid=uid, group=group):
                peer = MagicMock()
                peer.__enter__.return_value = peer
                peer.getsockopt.return_value = authority._CRED.pack(321, uid, uid)
                entries = [SimpleNamespace(st_mode=stat.S_IFDIR | 0o755, st_uid=61420),
                           SimpleNamespace(st_mode=stat.S_IFSOCK | 0o660)]
                with patch.object(Path, 'lstat', side_effect=entries), \
                        patch.object(Path, 'read_text', return_value=group), \
                        patch.object(authority.pwd, 'getpwnam', return_value=SimpleNamespace(pw_uid=61420)), \
                        patch.object(authority.socket, 'socket', return_value=peer):
                    report = authority.read_owner('first-boot', 'receiver-a')
                self.assertEqual(report['reason'], 'vendor_owner_untrusted')
                peer.send.assert_not_called()


if __name__ == '__main__':
    unittest.main()
