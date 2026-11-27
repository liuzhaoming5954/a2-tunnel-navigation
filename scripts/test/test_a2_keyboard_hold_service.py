"""正式键盘证据协议的隔离测试。只使用本机匿名套接字和临时文件。"""

from contextlib import redirect_stderr
import errno
from io import StringIO
import json
import os
from pathlib import Path
from queue import Queue
import socket
import sys
import tempfile
from threading import Event, Thread
import time
import unittest
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import a2_keyboard_hold_service as hold


class PeerProtocolTests(unittest.TestCase):
    key = bytes(range(32))

    def setUp(self):
        self.pc2, self.station = socket.socketpair()
        self.addCleanup(self.station.close)
        self.addCleanup(self.pc2.close)
        self.samples = Queue()
        self.cleared = Event()
        self.pc2_progress = hold._PeerProgress(1)
        self.station_progress = hold._PeerProgress(1)

    def run_peer(self, peer, method, *arguments):
        errors = Queue()

        def run():
            with peer:
                try:
                    method(peer, *arguments)
                except Exception as error:
                    errors.put(error)

        thread = Thread(target=run, daemon=True)
        thread.start()
        return thread, errors

    def start_receiver(self):
        return self.run_peer(self.pc2, hold.receive_peer, self.key, self.samples.put, self.cleared.set,
                             self.pc2_progress)

    def station_hello(self):
        session = 'ab' * 16
        hold._send(self.station, self.key, b'hello', session, {'session': session}, time.monotonic() + 1)
        return session

    def test_authenticated_samples_follow_key_and_clear_on_disconnect(self):
        receiver, received_errors = self.start_receiver()
        pressed = Event()
        disconnected = Event()
        pressed.set()

        def sample():
            if disconnected.is_set():
                raise OSError('模拟键盘断开')
            if pressed.is_set():
                return {'pressed': True, 'epoch': 'press-1', 'direction': 'forward'}
            return {'pressed': False, 'epoch': 'released', 'direction': None}

        workstation, station_errors = self.run_peer(self.station, hold.serve_peer, self.key, sample,
                                                    self.station_progress)
        first = self.samples.get(timeout=1)
        self.assertEqual(first['direction'], 'forward')
        pressed.clear()
        release_deadline = time.monotonic() + 1
        released = self.samples.get(timeout=1)
        while released['pressed']:
            self.assertLess(time.monotonic(), release_deadline)
            released = self.samples.get(timeout=1)
        self.assertIsNone(released['direction'])
        self.assertGreater(released['observed_mono_ns'], first['observed_mono_ns'])
        self.assertLessEqual(released['observed_mono_ns'], time.monotonic_ns())
        disconnected.set()
        workstation.join(timeout=1)
        receiver.join(timeout=1)
        self.assertTrue(self.cleared.is_set())
        self.assertFalse(workstation.is_alive())
        self.assertFalse(receiver.is_alive())
        self.assertIsInstance(station_errors.get_nowait(), OSError)
        self.assertTrue(received_errors.empty() or isinstance(received_errors.get_nowait(), OSError))
        self.assertEqual(self.station_progress.phase, 'keyboard_sample')
        self.assertGreaterEqual(self.station_progress.frames, 2)
        self.assertGreaterEqual(self.station_progress.max_sample_ms, self.station_progress.last_sample_ms)
        self.assertIsNotNone(self.pc2_progress.last_reply_rtt_ms)

    def test_wrong_station_key_cannot_publish(self):
        receiver, errors = self.start_receiver()
        other = b'x' * 32
        session = 'cd' * 16
        hold._send(self.station, other, b'hello', session, {'session': session}, time.monotonic() + 1)
        receiver.join(timeout=1)
        self.assertFalse(receiver.is_alive())
        self.assertIsInstance(errors.get_nowait(), hold._AuthenticationError)
        self.assertEqual(self.pc2_progress.phase, 'hello_verify')
        self.assertTrue(self.samples.empty())
        self.assertTrue(self.cleared.is_set())

    def test_old_server_challenge_cannot_trigger_sampling(self):
        sampled = Event()
        workstation, errors = self.run_peer(self.station, hold.serve_peer, self.key, sampled.set)
        hello = hold._read_frame(self.pc2.fileno(), time.monotonic() + 1)
        old_session = '00' * 16 if hello['session'] != '00' * 16 else '11' * 16
        hold._send(self.pc2, self.key, b'challenge', old_session, {'nonce': 'ab' * 16}, time.monotonic() + 1)
        workstation.join(timeout=1)
        self.assertFalse(workstation.is_alive())
        self.assertIsInstance(errors.get_nowait(), ValueError)
        self.assertFalse(sampled.is_set())

    def test_replayed_reply_cannot_refresh_and_clears_current_sample(self):
        receiver, errors = self.start_receiver()
        session = self.station_hello()
        challenge = hold._read_frame(self.station.fileno(), time.monotonic() + 1)
        fields = {'nonce': challenge['nonce'], 'pressed': True, 'epoch': 'press', 'direction': 'right'}
        hold._send(self.station, self.key, b'sample', session, fields, time.monotonic() + 1)
        self.assertEqual(self.samples.get(timeout=1)['direction'], 'right')
        following = hold._read_frame(self.station.fileno(), time.monotonic() + 1)
        self.assertNotEqual(following['nonce'], challenge['nonce'])
        hold._send(self.station, self.key, b'sample', session, fields, time.monotonic() + 1)
        receiver.join(timeout=1)
        self.assertIsInstance(errors.get_nowait(), ValueError)
        self.assertTrue(self.cleared.is_set())
        self.assertTrue(self.samples.empty())

    def test_missing_reply_clears_current_sample(self):
        receiver, errors = self.start_receiver()
        session = self.station_hello()
        challenge = hold._read_frame(self.station.fileno(), time.monotonic() + 1)
        fields = {'nonce': challenge['nonce'], 'pressed': True, 'epoch': 'press', 'direction': 'left'}
        hold._send(self.station, self.key, b'sample', session, fields, time.monotonic() + 1)
        self.samples.get(timeout=1)
        receiver.join(timeout=1)
        self.assertFalse(receiver.is_alive())
        self.assertIsInstance(errors.get_nowait(), TimeoutError)
        self.assertTrue(self.cleared.is_set())
        self.assertEqual(self.pc2_progress.phase, 'reply_read')
        self.assertEqual(self.pc2_progress.frames, 1)
        self.assertIsNotNone(self.pc2_progress.last_reply_rtt_ms)

    def test_hello_timeout_distinct_from_reply_timeout(self):
        receiver, errors = self.start_receiver()
        receiver.join(timeout=1)
        self.assertFalse(receiver.is_alive())
        self.assertIsInstance(errors.get_nowait(), TimeoutError)
        self.assertTrue(self.cleared.is_set())
        self.assertEqual(self.pc2_progress.phase, 'hello_read')
        self.assertEqual(self.pc2_progress.frames, 0)
        self.assertIsNone(self.pc2_progress.last_reply_rtt_ms)

    def test_clean_peer_close_has_eof_phase_and_clears_sample(self):
        receiver, errors = self.start_receiver()
        self.station_hello()
        hold._read_frame(self.station.fileno(), time.monotonic() + 1)
        self.station.close()
        receiver.join(timeout=1)
        self.assertFalse(receiver.is_alive())
        self.assertTrue(errors.empty())
        self.assertTrue(self.cleared.is_set())
        self.assertEqual(self.pc2_progress.phase, 'reply_read')


class DiagnosticTests(unittest.TestCase):
    @staticmethod
    def records(output):
        return [json.loads(line.removeprefix('A2_KEYBOARD_DIAG ')) for line in output.getvalue().splitlines()]

    def test_logs_have_fixed_error_fields_and_suppression_count(self):
        diagnostics = hold._Diagnostics('station')
        progress = hold._PeerProgress(attempt=3, phase='keyboard_sample')
        progress.phase_ns = 900_000_000
        secret = 'device path and arbitrary frame contents'
        failure = OSError(errno.EIO, secret)
        output = StringIO()
        with redirect_stderr(output), mock.patch.object(hold.time, 'monotonic_ns',
                side_effect=[1_000_000_000, 1_200_000_000, 2_000_000_000, 2_100_000_000]):
            diagnostics.emit('peer_end', progress, failure)
            diagnostics.emit('peer_end', progress, failure)
            diagnostics.emit('peer_end', progress, failure)
            diagnostics.emit('peer_end', progress, hold._AuthenticationError(secret))
        rows = self.records(output)
        self.assertEqual([row['reason'] for row in rows], ['io_error', 'io_error', 'authentication_failed'])
        self.assertEqual([row['suppressed'] for row in rows], [0, 1, 0])
        self.assertEqual(rows[0]['phase'], 'keyboard_sample')
        self.assertEqual(rows[0]['phase_elapsed_ms'], 100.)
        self.assertEqual(rows[0]['errno'], errno.EIO)
        self.assertEqual(rows[0]['mono_ns'], 1_000_000_000)
        self.assertEqual(rows[0]['attempt'], 3)
        self.assertNotIn(secret, output.getvalue())

    def test_station_reports_keyboard_open_and_loopback_connect_separately(self):
        with mock.patch.object(hold, '_physical_device', return_value=Path('/dev/input/event0')):
            for side_effect, expected_phase, expected_errno in (
                (OSError(errno.ENODEV, 'private path'), 'keyboard_open', errno.ENODEV),
                (None, 'tunnel_connect', errno.ECONNREFUSED),
            ):
                with self.subTest(phase=expected_phase):
                    output = StringIO()
                    with redirect_stderr(output), mock.patch.object(hold, 'open_keyboard', side_effect=side_effect), \
                            mock.patch.object(hold.socket, 'create_connection',
                                              side_effect=ConnectionRefusedError(errno.ECONNREFUSED, 'socket')), \
                            mock.patch.object(hold.time, 'sleep', side_effect=KeyboardInterrupt):
                        with self.assertRaises(KeyboardInterrupt):
                            hold._run_station(b'x' * 32, Path('/ignored'), hold._Diagnostics('station'))
                    ready, ended = self.records(output)
                    self.assertEqual(ready['event'], 'ready')
                    self.assertEqual(ended['event'], 'peer_end')
                    self.assertEqual(ended['phase'], expected_phase)
                    self.assertEqual(ended['errno'], expected_errno)


@unittest.skipUnless(os.geteuid() == 0, '权限检查需要 root 隔离环境')
class PublishPermissionsTests(unittest.TestCase):
    def test_root_sample_is_read_only_for_receiver_group_and_bound_to_robot(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            directory.chmod(0o750)
            gid = os.getegid()
            os.chown(directory, 0, gid)
            snapshot = directory / 'sample.json'
            with hold._snapshot_publisher(directory, gid, 'boot-id', 'receiver-id') as (publish, _clear):
                publish({'pressed': True, 'epoch': 'press', 'direction': 'left', 'observed_mono_ns': 7})
                info = snapshot.stat()
                self.assertEqual((info.st_uid, info.st_gid, info.st_mode & 0o777), (0, gid, 0o640))
                self.assertEqual(json.loads(snapshot.read_text()), {
                    'schema_version': 1, 'boot_id': 'boot-id', 'receiver_id': 'receiver-id',
                    'pressed': True, 'epoch': 'press', 'direction': 'left', 'observed_mono_ns': 7,
                })
                with self.assertRaises(BlockingIOError), hold._snapshot_publisher(directory, gid, 'b', 'r'):
                    self.fail('第二个写入者不应取得锁')
                self.assertTrue(snapshot.is_file())
            self.assertFalse(snapshot.exists())

    def test_key_rejects_nonprivate_mode_and_symbolic_link(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            key = directory / 'keyboard-hold.key'
            key.write_bytes(b'k' * 32)
            key.chmod(0o600)
            self.assertEqual(hold._load_key(key), b'k' * 32)
            key.chmod(0o640)
            with self.assertRaises(PermissionError):
                hold._load_key(key)
            key.chmod(0o600)
            link = directory / 'link.key'
            link.symlink_to(key)
            with self.assertRaises(OSError):
                hold._load_key(link)


if __name__ == '__main__':
    unittest.main()
