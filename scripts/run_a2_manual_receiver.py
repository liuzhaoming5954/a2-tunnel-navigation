#!/usr/bin/env python3
"""在 PC2 或独立开发机运行维护遥控 Unix 接收器；默认仅模拟。"""

import argparse
from contextlib import closing, contextmanager, nullcontext
import errno
import fcntl
import json
import os
from pathlib import Path
import pwd
from queue import Empty, Full, Queue
import signal
import socket
import stat
import struct
import sys
from threading import Event, Thread
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'a2_safety'))
from manual_receiver import ManualReceiver, MockBody, UnitreeBody


class ParkingTrace:
    """停车试验候选。事件在发令线程取 PC2 单调时戳，独立线程写标准错误的 JSON 行。"""

    def __init__(self):
        # 允许短时突发。队列满时丢弃诊断事件，整个进程最多输出 4096 条样本。
        self.pending = Queue(maxsize=256)
        self.queued = 0
        self.dropped = 0
        self.closing = Event()
        self.worker = Thread(target=self._drain, name='a2-parking-trace', daemon=True)
        self.worker.start()

    def record(self, event, **fields):
        # 生产端各调用由同一仲裁锁串行；此处不等待队列空位，不做文件、终端或网络 I/O。
        if self.queued >= 4096:
            self.dropped += 1
            return
        row = dict(trace='a2_parking', mono_ns=time.monotonic_ns(), event=event,
                   dropped=self.dropped, **fields)
        try:
            self.pending.put_nowait(row)
            self.queued += 1
        except Full:
            self.dropped += 1

    @staticmethod
    def _write(row):
        # 只序列化调用点明确列出的数值和固定原因，不记录网关报文、会话或异常详情。
        line = (json.dumps(row, separators=(',', ':'), allow_nan=False) + '\n').encode()
        while line:
            line = line[os.write(2, line):]

    def _drain(self):
        try:
            while True:
                try:
                    row = self.pending.get(timeout=.05)
                except Empty:
                    if self.closing.is_set():
                        break
                    continue
                self._write(row)
            self._write(dict(trace='a2_parking', mono_ns=time.monotonic_ns(), event='trace_closed',
                             queued=self.queued, dropped=self.dropped))
        except (OSError, ValueError):
            # 日志下游关闭时退出记录线程；接收器与停车看门狗继续原流程。
            return

    def close(self):
        # 仲裁和 SDK 的关闭先完成。日志下游堵塞不能推迟机体停车或无限等待进程退出。
        self.closing.set()
        self.worker.join(timeout=.15)


@contextmanager
def listen(path):
    # 套接字应置于自己的运行目录。锁住目录直至退出，避免新旧实例相互清理套接字。
    directory = os.open(path.parent, os.O_DIRECTORY | os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(directory)
        if info.st_uid not in (0, os.geteuid()) or info.st_mode & 0o022:
            raise PermissionError('套接字目录须由接收端或 root 管理，且仅属主可写')
        fcntl.flock(directory, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            old = path.lstat()
        except FileNotFoundError:
            old = None
        if old is not None:
            if not stat.S_ISSOCK(old.st_mode) or old.st_uid != os.geteuid():
                raise FileExistsError('套接字路径已被其他文件占用')
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
                probe.settimeout(.15)
                if probe.connect_ex(str(path)) != errno.ECONNREFUSED:
                    raise FileExistsError('该套接字仍在使用')
            # A 键锁存可能以 SIGKILL 结束受管进程，仅回收同一受控目录里已失效的自有套接字。
            path.unlink()
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(path))
            try:
                path.chmod(0o660)
                server.listen(8)
                server.settimeout(.15)
                yield server
            finally:
                path.unlink(missing_ok=True)
    finally:
        os.close(directory)


def process_connection(conn, receiver, gateway_uid):
    # 除文件权限外再次检查调用方进程身份；网关不能通过 JSON 指定调用者或切换接收端。
    credentials = conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize('3i'))
    _, uid, _ = struct.unpack('3i', credentials)
    if uid != gateway_uid:
        return
    deadline = time.monotonic() + .15
    message = bytearray()
    while len(message) < 4096 and b'\n' not in message:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        conn.settimeout(remaining)
        chunk = conn.recv(4096 - len(message))
        if not chunk:
            return
        message.extend(chunk)
    line, term, tail = message.partition(b'\n')
    try:
        if not term or tail:
            raise ValueError('参数必须是一行 JSON')
        result = receiver.handle(json.loads(line))
    except (ValueError, TypeError, RecursionError):
        result = {'ok': False, 'reason': 'invalid_request'}
    conn.sendall(json.dumps(result, allow_nan=False).encode() + b'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('mock', 'unitree'), default='mock')
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--gateway-user', help='允许访问套接字的 PC2 网关账号，默认同一用户')
    parser.add_argument('--interface', help='实机 SDK 使用的本机网卡')
    parser.add_argument('--domain', type=int, default=0, choices=range(233), metavar='0..232')
    policy = parser.add_mutually_exclusive_group()
    policy.add_argument('--site-forward-feedback', action='store_true', help='实机：订阅旧现场前向许可的独立机身反馈')
    policy.add_argument('--operator-supervised', action='store_true', help='实机：场地净空由操作员负责，机身反馈由 PC2 订阅')
    parser.add_argument('--mock-stop-file', type=Path, help='仅模拟：文件存在模拟 A 键并锁存')
    parser.add_argument('--mock-hold-file', type=Path, help='仅模拟：独立发布按住信号的本地快照')
    parser.add_argument('--mock-keyboard-hold', action='store_true', help='仅模拟：要求快照的方向与控制指令一致')
    parser.add_argument('--keyboard-hold', action='store_true', help='受管实机候选：要求可信的机载键盘证据')
    parser.add_argument('--continuous-hold', action='store_true', help='显式启用持续按住候选模式，须有独立方向键证据')
    parser.add_argument('--trace-parking', action='store_true', help='仅观测：向标准错误异步写入有界停车试验 JSON 行')
    parser.add_argument('--mock-parent-pid', type=int, help='仅模拟：随指定的直接父进程退出')
    args = parser.parse_args()
    if not args.socket.is_absolute() or args.socket.parent == args.socket:
        parser.error('--socket 需要受控目录内的绝对路径')
    if args.backend == 'unitree' and (not args.interface or args.mock_stop_file is not None or args.mock_hold_file is not None):
        parser.error('实机模式须提供 --interface，且不接受模拟停车或按住信号')
    if args.mock_hold_file is not None and not args.mock_hold_file.is_absolute():
        parser.error('--mock-hold-file 需要绝对路径')
    if args.mock_keyboard_hold and args.mock_hold_file is None:
        parser.error('--mock-keyboard-hold 须与 --mock-hold-file 一起使用')
    if args.keyboard_hold and (args.backend != 'unitree' or not args.operator_supervised):
        parser.error('--keyboard-hold 仅用于受管实机人工驾驶')
    if args.continuous_hold:
        if args.backend == 'mock' and not (args.mock_keyboard_hold and args.mock_hold_file):
            parser.error('模拟 --continuous-hold 需要 --mock-keyboard-hold 和 --mock-hold-file')
        if args.backend == 'unitree' and not (args.keyboard_hold and args.operator_supervised):
            parser.error('实机 --continuous-hold 需要 --keyboard-hold 和 --operator-supervised')
    if args.trace_parking and args.backend != 'unitree':
        parser.error('--trace-parking 记录实机 SDK 接口事件；模拟后端已有独立的事件流')
    if args.mock_parent_pid is not None and (args.backend != 'mock' or args.mock_parent_pid <= 1):
        parser.error('--mock-parent-pid 仅用于随模拟网关启动的子进程')
    if (args.site_forward_feedback or args.operator_supervised) and args.backend != 'unitree':
        parser.error('--site-forward-feedback 和 --operator-supervised 仅用于实机受管接收器')
    gateway_uid = pwd.getpwnam(args.gateway_user).pw_uid if args.gateway_user else os.geteuid()
    exiting = False

    def shutdown(_signum, _frame):
        nonlocal exiting
        exiting = True

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    os.umask(0o077)
    with listen(args.socket) as server, (closing(ParkingTrace()) if args.trace_parking else nullcontext()) as trace:
        if args.backend == 'mock':
            body = MockBody(args.mock_stop_file, lambda event: print(json.dumps(event), flush=True),
                            hold_file=args.mock_hold_file, keyboard_hold=args.mock_keyboard_hold,
                            continuous_hold=args.continuous_hold)
        else:
            body = UnitreeBody(args.interface, args.domain, site_forward_feedback=args.site_forward_feedback,
                               operator_supervised=args.operator_supervised, keyboard_hold=args.keyboard_hold,
                               allow_continuous_hold=args.continuous_hold)
            body.parking_trace = trace
        receiver = ManualReceiver(body, allow_continuous_hold=args.continuous_hold)
        try:
            # 子进程即使没收到终止信号，也会在父网关被杀死后自行清理私有套接字。
            while not exiting and (args.mock_parent_pid is None or os.getppid() == args.mock_parent_pid):
                try:
                    conn, _ = server.accept()
                    with conn:
                        process_connection(conn, receiver, gateway_uid)
                except (TimeoutError, ConnectionError):
                    # 单个本地网关请求可以超时或主动取消；独立看门狗不依赖连接存活。
                    continue
        finally:
            try:
                receiver.close()
            finally:
                if args.backend == 'unitree':
                    body.close()


if __name__ == '__main__':
    main()
