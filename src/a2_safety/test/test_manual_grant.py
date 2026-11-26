"""在临时目录与模拟单调时钟中验证本地许可，测试进程不初始化 DDS 或运动客户端。"""

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import unittest
from unittest.mock import patch

from manual_authority import HANDSET_SOURCE
import manual_grant as grant


class GrantTest(unittest.TestCase):
    def setUp(self):
        scope = ExitStack()
        self.addCleanup(scope.close)
        self.work = Path(scope.enter_context(tempfile.TemporaryDirectory()))
        self.ready = self.work / 'guard/ready'
        self.ready.parent.mkdir(mode=0o755)
        self.ready.touch(mode=0o600)
        self.latch = self.work / 'latched'
        self.boot = 'boot-a'
        self.receiver = 'unitree-a2-unit-test'
        self.now = 100.
        self.owner = dict(value='released', epoch='producer:1', source='test-feedback')
        scope.enter_context(patch.dict(sys.modules))
        scope.enter_context(patch.object(sys, 'path', [
            str(Path(__file__).resolve().parents[2] / 'a2_gateway'), *sys.path]))
        scope.enter_context(patch.multiple(grant, TRUSTED_UID=os.getuid(), DIRECTORY=self.work/'auth',
                                          READY=self.ready, LATCH=self.latch))
        scope.enter_context(patch.object(grant, 'identity', side_effect=lambda: (self.boot, self.receiver)))
        scope.enter_context(patch.object(grant.time, 'monotonic', side_effect=lambda: self.now))
        self.guard = scope.enter_context(patch('a2_gateway.remote_stop_status._read_guard', return_value=('running', False)))
        self.read_owner = scope.enter_context(patch('manual_authority.read_owner', side_effect=lambda *a, **k: dict(self.owner)))

    def issue(self, serial=0, seconds=2):
        return grant.issue(serial, seconds, ['forward'])

    def status(self):
        return grant.grant_status(self.boot, self.receiver, now=self.now)

    def test_latched_rejects_even_with_leftover_ready_without_writing(self):
        self.latch.touch()
        for _ in range(2):
            self.assertEqual(self.status()['reason'], 'stop_latched')
            self.assertEqual(self.issue(), {'ok': False, 'reason': 'stop_latched'})
            self.assertEqual(grant.issue(0, 5, ['forward'], site_front_confirmed=True),
                             {'ok': False, 'reason': 'stop_latched'})
            self.ready.unlink(missing_ok=True)
        self.guard.assert_not_called()
        self.read_owner.assert_not_called()
        self.assertFalse(grant.DIRECTORY.exists())

    def test_unknown_owner_or_inactive_guard_cannot_preissue(self):
        self.guard.return_value = ('unknown', None)
        self.assertEqual(self.issue()['reason'], 'stop_guard_not_ready')
        self.guard.return_value = ('running', False)
        for owner, expected in [('unknown','control_unknown'), ('owned','control_owned')]:
            self.owner['value'] = owner
            self.assertEqual(self.issue()['reason'], expected)
        self.owner.update(value='released', epoch=None)
        self.assertEqual(self.issue()['reason'], 'owner_continuity_unknown')
        self.assertFalse(grant.DIRECTORY.exists())

    def test_clock_expiry_retries_and_revoke_preserve_frozen_deadline(self):
        self.assertTrue(self.issue()['ok'])
        issued = self.status()['grant']
        self.assertEqual(issued, {'serial': 1, 'directions': ['forward'], 'issued_at': 100., 'expires_at': 102.})
        self.now = 101.
        self.assertEqual(self.issue(0)['reason'], 'local_grant_serial_changed')
        self.assertEqual(self.issue(1)['reason'], 'local_grant_active')
        self.assertEqual(self.status()['grant'], issued)
        self.now = 102.
        self.assertEqual(self.status()['reason'], 'local_grant_expired')
        self.assertTrue(self.issue(1)['ok'])
        self.assertEqual(grant.revoke()['serial'], 3)
        self.assertIsNone(self.status()['grant'])
        self.assertEqual(self.issue(2)['reason'], 'local_grant_serial_changed')

    def test_fresh_runtime_is_readable_even_with_private_umask(self):
        previous_umask = os.umask(0o077)
        try:
            issued = grant.issue(0, 5, ['forward'], site_front_confirmed=True)
        finally:
            os.umask(previous_umask)
        self.assertTrue(issued['ok'])
        self.assertEqual(stat.S_IMODE(grant.DIRECTORY.stat().st_mode), 0o755)
        self.assertEqual(stat.S_IMODE((grant.DIRECTORY / grant.FILENAME).stat().st_mode), 0o644)
        self.assertEqual(self.status()['grant'], issued['grant'])

    def test_repeated_actions_grant_requires_explicit_mode_and_preserves_epochs(self):
        for seconds, site, repeated in ((1, False, True), (None, True, True),
                                         (None, False, False), (None, False, 1)):
            self.assertEqual(grant.issue(0, seconds, ['forward'], site_front_confirmed=site,
                                         repeated_actions=repeated)['reason'], 'invalid_local_grant')
        response = grant.issue(0, None, ['forward', 'backward'], repeated_actions=True)
        self.assertTrue(response['ok'])
        self.assertEqual(response['grant'], {'serial': 1, 'issued_at': self.now, 'expires_at': None,
                                              'directions': ['backward', 'forward'], 'repeated_actions': True})
        self.now += grant.MAX_SECONDS + 1
        self.assertEqual(self.status()['grant'], response['grant'])
        self.assertEqual(self.issue(1)['reason'], 'local_grant_active')
        self.owner.update(value='owned')
        self.assertEqual(self.status()['reason'], 'control_owned')
        self.owner.update(value='released', epoch='producer:2')
        self.assertEqual(self.status()['reason'], 'local_grant_epoch_changed')
        self.latch.touch()
        self.assertEqual(self.status()['reason'], 'stop_latched')
        self.assertEqual(grant.revoke()['serial'], 2)

    def test_repeated_actions_grant_rejects_conflicting_or_incomplete_record(self):
        self.assertTrue(grant.issue(0, None, ['forward'], repeated_actions=True)['ok'])
        path = grant.DIRECTORY / grant.FILENAME
        original = json.loads(path.read_text())
        for update in ({'repeated_actions': False}, {'repeated_actions': 1},
                       {'expires_at': self.now + 5}, {'site_front_confirmed': True}):
            with self.subTest(update=update):
                path.write_text(json.dumps({**original, 'grant': {**original['grant'], **update}}))
                self.assertEqual(self.status()['reason'], 'local_grant_invalid')

    def test_continuous_hold_grant_is_explicit_and_exclusive(self):
        for duration, options in ((30, {'continuous_hold': True}),
                                  (None, {'continuous_hold': True, 'repeated_actions': True}),
                                  (None, {'continuous_hold': True, 'site_front_confirmed': True}),
                                  (None, {'continuous_hold': 1})):
            with self.subTest(duration=duration, options=options):
                self.assertEqual(grant.issue(0, duration, ['forward'], **options)['reason'],
                                 'invalid_local_grant')
        result = grant.issue(0, None, ['forward'], continuous_hold=True)
        self.assertEqual(result['grant'], {'serial': 1, 'issued_at': self.now, 'expires_at': None,
                                           'directions': ['forward'], 'continuous_hold': True})
        self.now += 31
        self.assertEqual(self.status()['grant'], result['grant'])
        path = grant.DIRECTORY / grant.FILENAME
        raw = json.loads(path.read_text())
        for additions in ({'continuous_hold': False}, {'expires_at': self.now + 5},
                          {'repeated_actions': True}, {'site_front_confirmed': True}):
            with self.subTest(additions=additions):
                path.write_text(json.dumps({**raw, 'grant': {**raw['grant'], **additions}}))
                self.assertEqual(self.status()['reason'], 'local_grant_invalid')
        path.write_text(json.dumps(raw))
        self.owner['epoch'] = 'producer:new-cycle'
        self.assertEqual(self.status()['reason'], 'local_grant_epoch_changed')

    def test_site_confirmation_grant_is_exactly_one_forward_direction_and_five_seconds(self):
        invalid = [(4, ['forward'], True), (6, ['forward'], True), (5., ['forward'], True),
                   (5, ['backward'], True), (5, ['forward', 'left'], True),
                   (5, ['forward', 'forward'], True), (5, [], True), (5, ['forward'], 1)]
        for seconds, directions, confirmed in invalid:
            with self.subTest(seconds=seconds, directions=directions, confirmed=confirmed):
                self.assertEqual(grant.issue(0, seconds, directions, site_front_confirmed=confirmed),
                                 {'ok': False, 'reason': 'invalid_local_grant'})
        self.assertFalse(grant.DIRECTORY.exists())
        # 在浮点指数量级变化附近签发，校验以同一开始时刻加五秒的截止值为准。
        self.now = 127.99999999999999
        issued = grant.issue(0, 5, ['forward'], site_front_confirmed=True)
        self.assertEqual(issued['grant'], {'serial': 1, 'directions': ['forward'], 'issued_at': self.now,
                                          'expires_at': self.now + 5, 'site_front_confirmed': True})
        self.assertEqual(self.status()['grant'], issued['grant'])
        self.assertEqual(grant.issue(1, 5, ['forward'], site_front_confirmed=True)['reason'],
                         'local_grant_active')
        self.now = issued['grant']['expires_at']
        self.assertEqual(self.status()['reason'], 'local_grant_expired')

    def test_site_confirmation_on_record_requires_boolean_true_and_same_frozen_scope(self):
        self.assertTrue(grant.issue(0, 5, ['forward'], site_front_confirmed=True)['ok'])
        path = grant.DIRECTORY / grant.FILENAME
        original = json.loads(path.read_text())
        for field, invalid in [('site_front_confirmed', False), ('site_front_confirmed', 1),
                               ('site_front_confirmed', 'true'), ('site_front_confirmed', None),
                               ('expires_at', self.now + 5.01), ('expires_at', self.now + 4),
                               ('directions', ['forward', 'forward']), ('directions', ['left'])]:
            with self.subTest(field=field, invalid=invalid):
                bad = {**original, 'grant': {**original['grant'], field: invalid}}
                path.write_text(json.dumps(bad))
                self.assertEqual(self.status()['reason'], 'local_grant_invalid')
                self.assertIsNone(self.status()['grant'])
        path.write_text(json.dumps(original))
        self.owner.update(value='owned')
        self.assertEqual(self.status()['reason'], 'control_owned')
        self.owner.update(value='released', epoch='producer:2')
        self.assertEqual(self.status()['reason'], 'local_grant_epoch_changed')

    def test_handset_center_grant_cannot_survive_takeover_or_input_gap(self):
        producer = '458b4d06-0e3f-409b-926f-4ec72efd5ace'
        centered = dict(value='unknown', source=HANDSET_SOURCE, epoch=None,
                        reason='handset_neutral_release_unconfirmed', observed_at=self.now,
                        neutral_epoch=f'{producer}:2', frame_count=3, source_tick=5)
        self.owner.update(centered)
        self.assertTrue(self.issue()['ok'])
        self.assertIsNotNone(self.status()['grant'])

        # 短按期间没有调用许可读取。再次回中后仍必须看到新的连续周期。
        self.owner['neutral_epoch'] = f'{producer}:4'
        self.assertIsNone(self.status()['grant'])
        self.assertTrue(self.issue(1)['ok'])
        self.assertIsNotNone(self.status()['grant'])

        self.owner.update(value='owned', reason='handset_active')
        self.assertIsNone(self.status()['grant'])
        self.assertEqual(self.issue(2)['reason'], 'control_owned')
        self.owner.update(value='unknown', reason='handset_frame_stale', observed_at=None)
        self.assertIsNone(self.status()['grant'])
        self.assertEqual(self.issue(2)['reason'], 'control_unknown')
        self.owner.update(centered, neutral_epoch=f'{producer}:7')
        self.assertIsNone(self.status()['grant'])

    def test_old_ownership_source_period_guard_marker_or_boot_cannot_restore_permission(self):
        self.assertTrue(self.issue()['ok'])
        self.owner['value'] = 'unknown'
        self.assertIsNone(self.status()['grant'])
        self.owner.update(value='released', epoch='producer:2')
        self.assertIsNone(self.status()['grant'])
        self.assertTrue(self.issue(1)['ok'])
        self.owner['source'] = 'another-feedback'
        self.assertIsNone(self.status()['grant'])
        self.assertTrue(self.issue(2)['ok'])
        self.ready.rename(self.ready.with_suffix('.old'))
        self.assertIsNone(self.status()['grant'])
        self.ready.touch(mode=0o600)
        self.assertIsNone(self.status()['grant'])
        self.assertTrue(self.issue(3)['ok'])
        self.assertIsNone(grant.grant_status('other-boot', self.receiver, now=self.now)['grant'])
        self.assertIsNone(grant.grant_status(self.boot, 'other-receiver', now=self.now)['grant'])

    def test_concurrent_grants_and_duration_direction_boundaries(self):
        for seconds in (0, -1, grant.MAX_SECONDS + 1, 4.5, True):
            self.assertEqual(self.issue(seconds=seconds)['reason'], 'invalid_local_grant')
        self.assertEqual(grant.issue(0, 2, ['up'])['reason'], 'invalid_local_grant')
        with ThreadPoolExecutor(2) as pool:
            results=list(pool.map(lambda _: self.issue(), range(2)))
        self.assertCountEqual([r.get('reason', 'ok') for r in results], ['ok', 'local_grant_serial_changed'])

    def test_ordinary_writer_invalid_file_modes_and_links(self):
        with patch.object(grant.os, 'geteuid', return_value=grant.TRUSTED_UID + 1):
            self.assertEqual(self.issue()['reason'], 'local_admin_required')
            self.assertEqual(grant.revoke()['reason'], 'local_admin_required')
        self.assertTrue(self.issue()['ok'])
        data = grant.DIRECTORY / grant.FILENAME
        data.chmod(0o664)
        self.assertEqual(self.status()['reason'], 'local_grant_invalid')
        data.rename(self.work / 'untrusted')
        data.symlink_to(self.work / 'untrusted')
        self.assertIsNone(self.status()['grant'])
        data.unlink()
        grant.DIRECTORY.chmod(0o775)
        self.assertEqual(self.status()['reason'], 'local_grant_invalid')


if __name__ == '__main__':
    unittest.main()
