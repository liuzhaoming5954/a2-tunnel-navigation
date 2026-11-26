#!/usr/bin/env python3
"""经已认证的双向字节流，将独立键盘读数送入隔离的 PC2 模拟接收器。"""

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import secrets
import select
import stat
import sys
import time


POLL_S = .05
REPLY_TIMEOUT_S = .13
CHALLENGE_TIMEOUT_S = .30
FRAME_BYTES = 256
DIRECTIONS = ('forward', 'backward', 'left', 'right')


def _distinct_fields(pairs):
    fields = dict(pairs)
    if len(fields) != len(pairs):
        raise ValueError('协议帧包含重复字段')
    return fields


def _fd(stream):
    return stream if isinstance(stream, int) else stream.fileno()


def _read_frame(reader, deadline):
    # 传输是逐轮请求和应答。每轮只接收一行，不缓存同批读到的附加帧。
    pending = bytearray()
    while len(pending) < FRAME_BYTES:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([reader], [], [], remaining)[0]:
            raise TimeoutError('未在期限内收到一整帧')
        chunk = os.read(reader, FRAME_BYTES - len(pending))
        if not chunk:
            if pending:
                raise ValueError('传输在帧结束前断开')
            return None
        pending.extend(chunk)
        if b'\n' in pending:
            line, _, tail = pending.partition(b'\n')
            if tail or time.monotonic() >= deadline:
                raise ValueError('收到额外或迟到的协议数据')
            value = json.loads(line.decode('ascii'), object_pairs_hook=_distinct_fields)
            if not isinstance(value, dict):
                raise ValueError('协议帧必须是 JSON 对象')
            return value
    raise ValueError('协议帧过长')


def _write_frame(writer, value, deadline):
    message = json.dumps(value, separators=(',', ':'), allow_nan=False).encode('ascii') + b'\n'
    if len(message) > FRAME_BYTES:
        raise ValueError('协议帧过长')
    remaining = deadline - time.monotonic()
    # 子进程管道由单个写者独占，帧短于 PIPE_BUF。
    if remaining <= 0 or not select.select([], [writer], [], remaining)[1]:
        raise TimeoutError('未在期限内发送协议帧')
    if os.write(writer, message) != len(message):
        raise OSError('协议帧写入不完整')


def _checked_sample(value):
    if not isinstance(value, dict) or value.keys() != {'pressed', 'epoch', 'direction'}:
        raise ValueError('键盘样本字段无效')
    pressed, epoch, direction = (value[key] for key in ('pressed', 'epoch', 'direction'))
    if type(pressed) is not bool or not isinstance(epoch, str) or not re.fullmatch('[a-zA-Z0-9_-]{1,64}', epoch):
        raise ValueError('键盘样本状态或按压周期无效')
    if (pressed and direction not in DIRECTIONS) or (not pressed and direction is not None):
        raise ValueError('键盘样本方向无效')
    return value


@contextmanager
def _mock_snapshot_writer(path):
    # 只允许写入本轮临时目录的新文件，拒绝正式服务路径、共享目录和符号链接。
    path = Path(path)
    parent = path.parent
    if (not path.is_absolute() or parent.resolve(strict=True) != parent
            or not any(base in parent.parents for base in (Path('/tmp'), Path('/var/tmp')))):
        raise ValueError('模拟样本须位于独立临时目录')
    directory = os.open(parent, os.O_DIRECTORY | os.O_NOFOLLOW | os.O_RDONLY)
    try:
        info = os.fstat(directory)
        if info.st_uid != os.geteuid() or info.st_mode & 0o077:
            raise PermissionError('模拟样本目录须仅供当前用户访问')
        claim = f'.{path.name}.writer'
        handle = os.open(claim, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600,
                         dir_fd=directory)
        os.close(handle)
        try:
            if os.path.lexists(path):
                raise FileExistsError('模拟样本目标已存在')
            temporary = f'.{path.name}.{secrets.token_hex(8)}'

            def publish(sample):
                fd = os.open(temporary, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600,
                             dir_fd=directory)
                with os.fdopen(fd, 'w', encoding='ascii') as output:
                    json.dump(sample, output, separators=(',', ':'), allow_nan=False)
                os.replace(temporary, path.name, src_dir_fd=directory, dst_dir_fd=directory)

            try:
                yield publish
            finally:
                for name in (path.name, temporary):
                    try:
                        os.unlink(name, dir_fd=directory)
                    except FileNotFoundError:
                        pass
        finally:
            os.unlink(claim, dir_fd=directory)
    finally:
        os.close(directory)


def receive_keyboard(reader, writer, mock_hold_file):
    """PC2 端轮询。到期、无效帧或管道关闭时立即撤掉本轮模拟证据。"""
    reader, writer = _fd(reader), _fd(writer)
    with _mock_snapshot_writer(mock_hold_file) as publish:
        while True:
            nonce = secrets.token_hex(16)
            issued_ns = time.monotonic_ns()
            issued = issued_ns / 1e9
            deadline = issued + REPLY_TIMEOUT_S
            _write_frame(writer, {'nonce': nonce}, deadline)
            reply = _read_frame(reader, deadline)
            if reply is None:
                return
            if reply.pop('nonce', None) != nonce:
                raise ValueError('键盘应答与本轮挑战不匹配')
            sample = _checked_sample(reply)
            if time.monotonic() >= deadline:
                raise TimeoutError('键盘应答已经过期')
            # 采样发生在上位机收到挑战后，用 PC2 发出挑战的时刻保守计龄。
            # 上位机时间、网页事件时刻及网络包到达时间均不能作为采样时间。
            publish({**sample, 'observed_mono_ns': issued_ns})
            time.sleep(max(0, issued + POLL_S - time.monotonic()))


def serve_keyboard(reader, writer, sample):
    """上位机端应答。sample 须当轮直接查询独立输入，返回一个按键样本。"""
    reader, writer = _fd(reader), _fd(writer)
    while True:
        challenge = _read_frame(reader, time.monotonic() + CHALLENGE_TIMEOUT_S)
        if challenge is None:
            return
        received = time.monotonic()
        nonce = challenge.get('nonce')
        if (challenge.keys() != {'nonce'} or not isinstance(nonce, str)
                or not re.fullmatch('[0-9a-f]{32}', nonce)):
            raise ValueError('挑战格式无效')
        result = _checked_sample(sample())
        _write_frame(writer, {'nonce': nonce, **result}, received + REPLY_TIMEOUT_S)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    receive = commands.add_parser('receive', help='只在 PC2 模拟测试目录发布样本')
    receive.add_argument('--mock-hold-file', type=Path, required=True)
    args = parser.parse_args()
    try:
        receive_keyboard(sys.stdin.buffer, sys.stdout.buffer, args.mock_hold_file)
    except (OSError, TypeError, ValueError, RecursionError) as error:
        print(f'模拟键盘信号已撤销：{error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
