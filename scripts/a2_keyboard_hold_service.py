#!/usr/bin/env python3
"""独立键盘证据服务。仅经 SSH 回环转发通信，不调用底盘 SDK。"""

import argparse
from contextlib import contextmanager
import fcntl
import grp
import hashlib
import hmac
import json
import os
from pathlib import Path
import re
import secrets
import signal
import socket
import stat
import sys
import time

from a2_keyboard_hold_link import (CHALLENGE_TIMEOUT_S, POLL_S, REPLY_TIMEOUT_S,
                                   _checked_sample, _read_frame, _write_frame)
from observe_a2_keyboard_hold import open_keyboard


ADDRESS = ('127.0.0.1', 18778)
KEY_PATH = Path('/etc/a2/keyboard-hold.key')
RUNTIME_DIRECTORY = Path('/run/a2-keyboard-hold')
_PREFIX = b'a2-keyboard-hold/v1/'
_PEER_ERRORS = (OSError, ValueError, RecursionError)


def _proof(key, role, session, fields):
    # 各消息用途分域。公开的站端会话随机数绑定 PC2 挑战和应答，旧连接不能续期。
    body = json.dumps(fields, ensure_ascii=True, sort_keys=True, separators=(',', ':'),
                      allow_nan=False).encode('ascii')
    message = _PREFIX + role + b'\0' + session.encode('ascii') + b'\0' + body
    return hmac.new(key, message, hashlib.sha256).hexdigest()


def _hex(value, size):
    return isinstance(value, str) and len(value) == size and re.fullmatch('[0-9a-f]+', value) is not None


def _validate_frame(frame, key, role, session, fields):
    if frame.keys() != fields | {'proof'} or not _hex(frame['proof'], 64):
        raise ValueError('键盘信号认证帧格式无效')
    body = {name: frame[name] for name in fields}
    if not hmac.compare_digest(frame['proof'], _proof(key, role, session, body)):
        raise ValueError('键盘信号认证失败')
    return body


def _send(peer, key, role, session, fields, deadline):
    _write_frame(peer.fileno(), {**fields, 'proof': _proof(key, role, session, fields)}, deadline)


def receive_peer(peer, key, publish, clear):
    """PC2 端处理一条连接，失败、超时或关闭立即撤下当前样本。"""
    fd = peer.fileno()
    try:
        hello = _read_frame(fd, time.monotonic() + CHALLENGE_TIMEOUT_S)
        if hello is None:
            return
        session = hello.get('session')
        if not _hex(session, 32):
            raise ValueError('键盘信号会话标识无效')
        _validate_frame(hello, key, b'hello', session, {'session'})
        while True:
            nonce = secrets.token_hex(16)
            issued_ns = time.monotonic_ns()
            issued = issued_ns / 1e9
            deadline = issued + REPLY_TIMEOUT_S
            _send(peer, key, b'challenge', session, {'nonce': nonce}, deadline)
            reply = _read_frame(fd, deadline)
            if reply is None:
                return
            if reply.get('nonce') != nonce:
                raise ValueError('键盘信号应答与当前挑战不匹配')
            fields = _validate_frame(reply, key, b'sample', session, {'nonce', 'pressed', 'epoch', 'direction'})
            del fields['nonce']
            sample = _checked_sample(fields)
            if time.monotonic() >= deadline:
                raise TimeoutError('键盘信号应答过期')
            # 采样必在挑战之后。以 PC2 发出挑战的时间保守计龄，避免排队报文延长按住证据寿命。
            publish({**sample, 'observed_mono_ns': issued_ns})
            time.sleep(max(0.0, issued + POLL_S - time.monotonic()))
    finally:
        clear()


def serve_peer(peer, key, sample):
    """上位机端处理一条连接。验证本轮挑战后才同步采样实体设备。"""
    fd = peer.fileno()
    session = secrets.token_hex(16)
    _send(peer, key, b'hello', session, {'session': session}, time.monotonic() + REPLY_TIMEOUT_S)
    while True:
        challenge = _read_frame(fd, time.monotonic() + CHALLENGE_TIMEOUT_S)
        if challenge is None:
            return
        received = time.monotonic()
        fields = _validate_frame(challenge, key, b'challenge', session, {'nonce'})
        nonce = fields['nonce']
        if not _hex(nonce, 32):
            raise ValueError('键盘信号挑战标识无效')
        _send(peer, key, b'sample', session, {**_checked_sample(sample()), 'nonce': nonce},
              received + REPLY_TIMEOUT_S)


def _load_key(path):
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        parent = os.fstat(directory)
        if parent.st_uid != 0 or parent.st_mode & 0o022:
            raise PermissionError('键盘通道密钥目录不由 root 管理')
        fd = os.open(path.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC,
                     dir_fd=directory)
        try:
            info = os.fstat(fd)
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_nlink != 1
                    or stat.S_IMODE(info.st_mode) != 0o600 or info.st_size != 32):
                raise PermissionError('键盘通道密钥须是 root 所有、0600 的 32 字节文件')
            key = os.read(fd, 33)
            if len(key) != 32:
                raise ValueError('键盘通道密钥长度无效')
            return key
        finally:
            os.close(fd)
    finally:
        os.close(directory)


@contextmanager
def _snapshot_publisher(path, group_id, boot_id, receiver_id):
    # 正式路径仅供本机授权的接收器组读取。独占锁覆盖整个发布期间，防止并行实例互删样本。
    directory = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(directory)
        if (info.st_uid != 0 or info.st_gid != group_id or stat.S_IMODE(info.st_mode) != 0o750):
            raise PermissionError('正式键盘证据目录须由 root:a2-autonomy 管理并设为 0750')
        lock = os.open('.publisher.lock', os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC,
                       0o600, dir_fd=directory)
        try:
            lock_info = os.fstat(lock)
            if (not stat.S_ISREG(lock_info.st_mode) or lock_info.st_uid != 0
                    or lock_info.st_nlink != 1 or stat.S_IMODE(lock_info.st_mode) != 0o600):
                raise PermissionError('正式键盘证据写锁不可信')
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

            def clear(name='sample.json'):
                try:
                    os.unlink(name, dir_fd=directory)
                except FileNotFoundError:
                    pass

            def publish(sample):
                temporary = f'.sample-{secrets.token_hex(8)}'
                fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
                             0o600, dir_fd=directory)
                try:
                    with os.fdopen(fd, 'w', encoding='ascii') as stream:
                        os.fchown(stream.fileno(), 0, group_id)
                        os.fchmod(stream.fileno(), 0o640)
                        json.dump({**sample, 'schema_version': 1, 'boot_id': boot_id, 'receiver_id': receiver_id},
                                  stream, separators=(',', ':'), allow_nan=False)
                    os.replace(temporary, 'sample.json', src_dir_fd=directory, dst_dir_fd=directory)
                finally:
                    clear(temporary)

            clear()
            try:
                yield publish, clear
            finally:
                clear()
        finally:
            os.close(lock)
    finally:
        os.close(directory)


def _configure_stream(peer):
    peer.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    # TCP 拆包或短写失败时直接撤销，不让阻塞读写无限保留证据。
    peer.setblocking(False)


def _run_pc2(key):
    group_id = grp.getgrnam('a2-autonomy').gr_gid
    if os.getegid() != group_id:
        raise PermissionError('PC2 发布进程须使用 a2-autonomy 组')
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    receiver_id = 'unitree-a2-' + Path('/etc/machine-id').read_text().strip()
    with _snapshot_publisher(RUNTIME_DIRECTORY, group_id, boot_id, receiver_id) as (publish, clear):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(ADDRESS)
            listener.listen(1)
            while True:
                peer, _address = listener.accept()
                # 同一时刻只接收一路操作输入，不让新连接悄悄续上旧样本。
                with peer:
                    try:
                        _configure_stream(peer)
                        receive_peer(peer, key, publish, clear)
                    except _PEER_ERRORS:
                        clear()


def _physical_device(path):
    if not path.is_absolute():
        raise ValueError('请使用实体键盘设备的绝对路径')
    node = path.resolve(strict=True)
    if not re.fullmatch(r'/dev/input/event[0-9]+', str(node)):
        raise ValueError('请选择实体 evdev 键盘')
    device = (Path('/sys/class/input') / node.name).resolve(strict=True)
    # 保留内置 AT 或外置 USB 键盘，拒绝 Linux uinput 等虚拟注入设备。
    if '/sys/devices/virtual/' in str(device):
        raise ValueError('键盘证据不能来自虚拟输入设备')
    return node


def _run_station(key, device):
    while True:
        try:
            with open_keyboard(_physical_device(device)) as keyboard:
                with socket.create_connection(ADDRESS, timeout=CHALLENGE_TIMEOUT_S) as peer:
                    _configure_stream(peer)
                    serve_peer(peer, key, keyboard.sample)
        except _PEER_ERRORS:
            # 断线、设备拔出或采样异常都结束此轮，重连后由新观察器重新确认回中。
            pass
        time.sleep(.25)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('pc2', help='PC2 回环键盘证据接收和发布')
    station = commands.add_parser('station', help='上位机实体键盘采样')
    station.add_argument('--device', type=Path, required=True)
    args = parser.parse_args()

    def shutdown(_number, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, shutdown)
    try:
        if os.geteuid() != 0:
            raise PermissionError('正式键盘通道须由 root 服务管理')
        key = _load_key(KEY_PATH)
        if args.command == 'pc2':
            _run_pc2(key)
        else:
            _run_station(key, args.device)
    except KeyboardInterrupt:
        return 0
    except (OSError, KeyError, ValueError) as error:
        print(f'键盘证据服务退出：{error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
