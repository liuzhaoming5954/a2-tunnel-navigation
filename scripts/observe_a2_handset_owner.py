#!/usr/bin/env python3
"""读取 A2 手柄输入并报告原厂占用。只订阅 LowState，不创建运动客户端。"""

import argparse
import errno
import json
import os
from pathlib import Path
import re
import signal
import socket
import stat
import sys
from threading import Event, Lock
import time
from uuid import uuid4

scripts = Path(__file__).resolve().parent
sys.path[:0] = [str(scripts), str(scripts.parent / 'src/a2_safety')]
from manual_authority import HANDSET_SOURCE, _MAX_AGE_S, _MAX_BYTES, _SOCKET, _distinct_fields
from probe_a2_sport import REMOTE_AXIS_THRESHOLD, remote_sample


class HandsetOwner:
    def __init__(self, *, clock=time.monotonic):
        self.clock = clock
        self.lock = Lock()
        self.instance = str(uuid4())
        self.generation = 1
        self.frame_count = 0
        self.tick = None
        self.writer = None
        self.source_ns = None
        self.fastest_offset = None
        self.last_rx_ns = None
        self.observed = None
        self.state = 'handset_no_frames'

    def transition(self, state):
        if self.state != state:
            self.state = state
            self.generation += 1

    def receive(self, msg):
        # tick 是消息源的 uint32 计数。重复、倒退或无效帧不能刷新 PC2 的本地采样时间。
        # 无缓存回调使用接收单调时刻；在旧帧后保留 tick 水位，避免一次倒序后引入旧序列。
        with self.lock:
            now = self.clock()
            if self.observed is not None and now - self.observed >= _MAX_AGE_S:
                self.transition('handset_frame_stale')
            try:
                tick = msg.tick
                if type(tick) is not int or not 0 <= tick < 2**32:
                    raise ValueError('源帧计数无效')
                if self.tick is not None and not 0 < (tick - self.tick) % 2**32 < 2**31:
                    raise ValueError('源帧未推进')
                info = msg.sample_info
                writer, stamp = info.publication_handle, info.source_timestamp
                if (info.valid_data is not True or type(writer) is not int or not 0 < writer < 2**64
                        or type(stamp) is not int or not 0 < stamp < 2**63):
                    raise ValueError('缺少 DDS 写者或原始时间')
                if self.writer is not None and (writer != self.writer or stamp <= self.source_ns):
                    raise ValueError('报文来源改变或源时间倒退')
                # 沿用停车守护的时钟对照：以同一写者的最短延时作基线，容许 1‰ 慢漂，
                # 源时钟无需与 PC2 对时。积压报文不能在本机重新获得完整新鲜期。
                rx_ns = round(now * 1e9)
                offset = rx_ns - stamp
                fastest = offset if self.fastest_offset is None else min(
                    offset, self.fastest_offset + max(0, rx_ns - self.last_rx_ns) // 1000)
                age = (offset - fastest) / 1e9
                if age >= _MAX_AGE_S:
                    raise ValueError('DDS 报文到达时已经过期')
                data = remote_sample('rt/lf/lowstate', msg)
                active = bool(data['keys']) or any(abs(v) > REMOTE_AXIS_THRESHOLD for v in data['axes'].values())
            except (AttributeError, TypeError, ValueError, OverflowError):
                self.observed = None
                self.transition('handset_invalid_frame')
                return
            self.tick, self.observed = tick, now - age
            self.writer, self.source_ns = writer, stamp
            self.fastest_offset, self.last_rx_ns = fastest, rx_ns
            self.frame_count += 1
            self.transition('handset_active' if active else 'handset_neutral_release_unconfirmed')

    def report(self):
        with self.lock:
            if self.observed is not None and self.clock() - self.observed >= _MAX_AGE_S:
                self.transition('handset_frame_stale')
                self.observed = None
            return {'value': 'owned' if self.state == 'handset_active' else 'unknown',
                    'reason': self.state, 'source': HANDSET_SOURCE, 'observed_at': self.observed,
                    'producer_instance': self.instance, 'generation': self.generation,
                    'frame_count': self.frame_count, 'source_tick': self.tick}


def answer(conn, owner, boot_id, receiver_id):
    # 每次只收一条 status；身份取 PC2 本机，拒绝代其他机器答复，网页也不能注入 owner。
    conn.settimeout(.015)
    data, _, flags, _ = conn.recvmsg(_MAX_BYTES)
    if not data or flags & socket.MSG_TRUNC:
        return
    req = json.loads(data, object_pairs_hook=_distinct_fields)
    if (not isinstance(req, dict) or set(req) != {'schema_version', 'op', 'nonce', 'boot_id', 'receiver_id'}
            or type(req['schema_version']) is not int or req['schema_version'] != 1 or req['op'] != 'status'
            or req['boot_id'] != boot_id or req['receiver_id'] != receiver_id
            or not isinstance(req['nonce'], str) or not re.fullmatch('[0-9a-f]{32}', req['nonce'])):
        return
    conn.sendall(json.dumps({**req, **owner.report()}, separators=(',', ':')).encode())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--domain', type=int, choices=range(233), required=True, metavar='0..232')
    args = parser.parse_args()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    owner = HandsetOwner()
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    receiver_id = 'unitree-a2-' + Path('/etc/machine-id').read_text().strip()
    done = Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: done.set())
    ChannelFactoryInitialize(args.domain, args.interface)
    reader = ChannelSubscriber('rt/lf/lowstate', LowState_)
    reader.Init(owner.receive)
    try:
        # 目录由 systemd 独占管理。重启后只回收自己目录下的旧套接字，保留其他文件。
        parent = _SOCKET.parent.lstat()
        if not stat.S_ISDIR(parent.st_mode) or parent.st_uid != os.geteuid() or parent.st_mode & 0o022:
            raise PermissionError('实时权属目录身份不符')
        if os.path.lexists(_SOCKET):
            old = _SOCKET.lstat()
            if not stat.S_ISSOCK(old.st_mode) or old.st_uid != os.geteuid():
                raise PermissionError('反馈入口已被其他文件占用')
            with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as probe:
                probe.settimeout(.05)
                if probe.connect_ex(str(_SOCKET)) != errno.ECONNREFUSED:
                    raise FileExistsError('反馈服务仍在运行')
            _SOCKET.unlink()
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as server:
            server.bind(str(_SOCKET))
            try:
                # 权属接口只允许查询。系统运行目录除了此动态账号以外均不可写。
                _SOCKET.chmod(0o666)
                server.listen(16)
                server.settimeout(.2)
                while not done.is_set():
                    try:
                        conn, _ = server.accept()
                        with conn:
                            answer(conn, owner, boot_id, receiver_id)
                    except (OSError, ValueError, RecursionError):
                        continue
            finally:
                _SOCKET.unlink(missing_ok=True)
    finally:
        reader.Close()


if __name__ == '__main__':
    main()
