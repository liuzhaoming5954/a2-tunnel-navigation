#!/usr/bin/env python3
"""在 PC2 静止窗口内发布停稳快照，可同时登记现场测量的前方净空。"""

import argparse
from contextlib import contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import socket
import stat
import subprocess
import sys
import tempfile
import time
from uuid import uuid4

from a2_remote_stop_sdk import ReadOnlyStandstill


root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root / 'src/a2_safety'))
from manual_grant import LATCH, READY, identity
from manual_receiver import SAFETY_DIRECTORY, STATIC_CLEARANCE_SOURCE, trusted_file


NAME = 'evidence.json'
MOVE_UNIT = 'a2-autonomy.service'
PERIOD_S = 0.05


def require_idle():
    """现场运行这里只读检查发令单元，出现其他任务立即结束本次采样。"""
    report = subprocess.check_output(
        ['systemctl', 'show', MOVE_UNIT, '-pActiveState', '-pMainPID'],
        text=True, timeout=.5,
    )
    values = dict(line.split('=', 1) for line in report.splitlines())
    if values != {'ActiveState': 'inactive', 'MainPID': '0'}:
        raise RuntimeError('静止窗口要求底盘发令服务处于未启动状态')


def require_latched():
    """现场量距和前雷达诊断只在停车守护持续锁存的静止窗口登记。"""
    guard = subprocess.check_output(
        ['systemctl', 'show', 'a2-remote-stop.service', '-pActiveState', '-pStatusText'],
        text=True, timeout=.5,
    )
    fields = dict(line.split('=', 1) for line in guard.splitlines())
    marker = LATCH.lstat()
    if (fields.get('ActiveState') != 'active' or not fields.get('StatusText', '').startswith('停车已锁存：')
            or not stat.S_ISREG(marker.st_mode) or marker.st_uid != 0 or marker.st_mode & 0o022
            or os.path.lexists(READY)):
        raise RuntimeError('登记净空诊断要求停车守护保持锁存')


@contextmanager
def locked_directory():
    # 许可记录与快照共享目录。使用同一把短锁，避免两次维护写入竞争。
    fd = os.open(SAFETY_DIRECTORY, os.O_DIRECTORY | os.O_NOFOLLOW | os.O_RDONLY)
    try:
        info = os.fstat(fd)
        if info.st_uid != 0 or info.st_mode & 0o022:
            raise PermissionError('机载安全快照目录不可信')
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield fd
    finally:
        os.close(fd)


def snapshot_owner(fd):
    try:
        source = os.open(NAME, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=fd)
    except FileNotFoundError:
        return None
    with os.fdopen(source, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022 or info.st_size > 8192:
            raise PermissionError('已有安全快照不可信')
        # 已有生产快照即使没有试验标识也不能覆盖。
        return json.load(stream).get('probe_id', '')


def publish(payload):
    with locked_directory() as fd:
        if snapshot_owner(fd) not in (None, payload['probe_id']):
            raise RuntimeError('已有其他安全快照生产者')
        with tempfile.NamedTemporaryFile(dir=SAFETY_DIRECTORY, mode='w', encoding='utf-8', delete=False) as stream:
            temp = Path(stream.name)
            try:
                json.dump(payload, stream, ensure_ascii=False, allow_nan=False)
                stream.flush()
                os.fchmod(stream.fileno(), 0o644)
                os.replace(temp.name, NAME, src_dir_fd=fd, dst_dir_fd=fd)
            finally:
                temp.unlink(missing_ok=True)


def clear_own(probe_id):
    with locked_directory() as fd:
        if snapshot_owner(fd) == probe_id:
            os.unlink(NAME, dir_fd=fd)


def observe(args):
    if os.geteuid() != 0 or socket.gethostname().split('.')[0] != 'unitree-a2-pc2':
        raise PermissionError('须在 PC2 以管理员身份运行静止采集')
    require_idle()
    locked_probe = args.front_clearance_m is not None or args.front_lidar_file is not None
    if locked_probe:
        require_latched()
    recorded_clearance = None
    if args.front_clearance_m is not None:
        # 登记时刻固定，连续发布机身样本不能让同一次人工量距自动续期。
        recorded_clearance = {'directions': [], 'observed_at': time.monotonic(),
                              'source': STATIC_CLEARANCE_SOURCE, 'front_clearance_m': args.front_clearance_m}
    SAFETY_DIRECTORY.mkdir(mode=0o755, exist_ok=True)
    with locked_directory() as fd:
        if snapshot_owner(fd) is not None:
            raise RuntimeError('已有安全快照，本次静止采集不覆盖')
    boot_id, receiver_id = identity()
    probe_id = uuid4().hex
    readout = ReadOnlyStandstill(args.interface, args.domain)
    count = 0
    try:
        readout.start()
        next_check = time.monotonic()
        expires = next_check + args.seconds
        while time.monotonic() < expires:
            now = time.monotonic()
            if now >= next_check:
                require_idle()
                if locked_probe:
                    require_latched()
                next_check = now + .25
            feedback = readout.feedback()
            if feedback is not None:
                # 无可用的实时净空源时，空集合明确禁止任何方向，不能用旧场地信息续期。
                guard = recorded_clearance or {
                    'directions': [], 'observed_at': time.monotonic(), 'source': 'static_no_clearance',
                }
                snapshot = dict(boot_id=boot_id, receiver_id=receiver_id, probe_id=probe_id,
                                guard=guard, feedback=feedback)
                if args.front_lidar_file is not None:
                    try:
                        snapshot['front_lidar'] = json.loads(trusted_file(args.front_lidar_file))
                    except (OSError, ValueError, RecursionError):
                        # 即使雷达缺帧，仍保留诊断输入标记，方向集合持续为空。
                        snapshot['front_lidar'] = None
                publish(snapshot)
                count += 1
            time.sleep(PERIOD_S)
    finally:
        try:
            readout.close()
        finally:
            clear_own(probe_id)
    print(json.dumps({'snapshots': count, 'source': 'sport_and_imu', 'authorized_directions': [],
                      **({'front_clearance_m': recorded_clearance['front_clearance_m']}
                         if recorded_clearance is not None else {})}))
    return 0 if count else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--seconds', type=int, default=8, metavar='3–30')
    parser.add_argument('--front-clearance-m', type=float, help='现场量得的前方净距（米），供锁存下静止验收')
    parser.add_argument('--front-lidar-file', type=Path, help='同轮只读前雷达诊断快照的绝对路径')
    args = parser.parse_args()
    if not 3 <= args.seconds <= 30:
        parser.error('静止采集时长须为 3 至 30 秒')
    if args.front_clearance_m is not None and (not math.isfinite(args.front_clearance_m) or args.front_clearance_m < 0):
        parser.error('现场净空须为非负的有限米数')
    if args.front_lidar_file is not None and not args.front_lidar_file.is_absolute():
        parser.error('前雷达诊断快照须为绝对路径')
    return observe(args)


if __name__ == '__main__':
    sys.exit(main())
