"""正式键盘证据协议的隔离测试。只使用本机匿名套接字和临时文件。"""

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
        return self.run_peer(self.pc2, hold.receive_peer, self.key, self.samples.put, self.cleared.set)

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

        workstation, station_errors = self.run_peer(self.station, hold.serve_peer, self.key, sample)
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

    def test_wrong_station_key_cannot_publish(self):
        receiver, errors = self.start_receiver()
        other = b'x' * 32
        session = 'cd' * 16
        hold._send(self.station, other, b'hello', session, {'session': session}, time.monotonic() + 1)
        receiver.join(timeout=1)
        self.assertFalse(receiver.is_alive())
        self.assertIsInstance(errors.get_nowait(), ValueError)
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
