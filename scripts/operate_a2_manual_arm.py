#!/usr/bin/env python3
"""为已认证的本机网关执行一次受管遥控启用。"""

import argparse
import json
import os
from pathlib import Path
import pwd
import shlex
import socket
import stat
import subprocess
import sys
import time


PROGRAM = Path(__file__).resolve()
RELEASE = PROGRAM.parents[1]
sys.path[:0] = [str(RELEASE / name) for name in ('scripts', 'src/a2_safety', 'src/a2_gateway')]
from a2_gateway.motion import LocalMotionClient, _command_receipt
from a2_remote_stop import GuardFiles, RUN_DIR, STATE_DIR
from manual_authority import _distinct_fields, neutral_epoch, read_owner
from manual_grant import _ready_epoch, grant_status, identity, issue_cockpit, revoke_cockpit
from manual_receiver import keyboard_sample
from operate_a2_stop_reset import GATEWAY_USER, GuardService, Transaction, allowed_peer, foreign_or_missing


SENDER = 'a2-autonomy.service'
SENDER_USER = 'a2-autonomy'
LAUNCHER = Path('/etc/a2/autonomy-start')
MOTION_SOCKET = Path('/run/a2-manual/receiver.sock')
REQUEST_TIMEOUT = .15
ACTIVATION_TIMEOUT = 6.
POLL_INTERVAL = .05
MAX_REQUEST_BYTES = 96


class Refused(Exception):
    """只向网关返回固定的状态原因。"""


def failure(reason):
    return {'ok': False, 'reason': reason}


def trusted(path, kind):
    if foreign_or_missing(path, kind):
        raise Refused('manual_arm_profile_unavailable')


class ManagedReceiver:
    """只允许启动固定手动配置，状态和会话查询降为原网关普通账号。"""

    def __init__(self, release=RELEASE, launcher=LAUNCHER):
        self.launcher = launcher
        self.entry = release / 'scripts/start_a2_manual_receiver.sh'
        self.receiver = release / 'scripts/run_a2_manual_receiver.py'

    @staticmethod
    def command(*args, timeout=.6):
        return subprocess.run(['/usr/bin/systemctl', '--system', *args, SENDER],
                              check=True, timeout=timeout, capture_output=True, text=True).stdout

    def status(self):
        fields = 'LoadState,ActiveState,MainPID,InvocationID,User,Restart,ExecStart,Environment'
        output = self.command('show', '--all', f'--property={fields}')
        props = dict(row.split('=', 1) for row in output.splitlines() if '=' in row)
        # PID1 从 root 管理的固定路径启动进程；不能借启用按钮启动其他任务入口。
        trusted(self.launcher.parent, stat.S_ISDIR)
        link = self.launcher.lstat()
        if link.st_uid != 0 or not stat.S_ISLNK(link.st_mode) or self.launcher.resolve(strict=True) != self.entry:
            raise Refused('manual_arm_profile_unavailable')
        for file in (self.entry, self.receiver):
            trusted(file, stat.S_ISREG)
            for parent in file.parents:
                trusted(parent, stat.S_ISDIR)
        env = set(shlex.split(props.get('Environment', '')))
        if (props.get('LoadState') != 'loaded' or props.get('User') != SENDER_USER
                or props.get('Restart') != 'no'
                or f'argv[]=/usr/bin/env -- {self.launcher} ;' not in props.get('ExecStart', '')
                or not {'A2_REQUIRE_KEYBOARD_HOLD=1', 'A2_ALLOW_CONTINUOUS_HOLD=1'} <= env):
            raise Refused('manual_arm_profile_unavailable')
        return props

    def manual_process(self, props):
        pid = int(props['MainPID'])
        if props['ActiveState'] != 'active' or pid <= 1:
            return None
        cmdline = (Path('/proc') / str(pid) / 'cmdline').read_bytes()
        args = {os.fsdecode(arg) for arg in cmdline.split(b'\0') if arg}
        if {str(self.receiver), '--backend', 'unitree', '--operator-supervised', '--keyboard-hold',
            '--continuous-hold', str(MOTION_SOCKET)} <= args:
            return props['InvocationID'], pid
        return None

    def start(self):
        self.command('start', '--no-block')

    @staticmethod
    def request(operation, timeout):
        account = pwd.getpwnam(GATEWAY_USER)
        # 接收器的 SO_PEERCRED 依然只准许网关账号访问。root 不直接发送运动协议。
        result = subprocess.run(['/usr/bin/python3', '-I', '-sB', str(PROGRAM), operation],
                                user=account.pw_uid, group=account.pw_gid, extra_groups=[],
                                stdin=subprocess.DEVNULL, capture_output=True, check=True,
                                text=True, timeout=timeout)
        return json.loads(result.stdout, object_pairs_hook=_distinct_fields)


class Arm:
    def __init__(self, transaction, guard, receiver, *, clock=time.monotonic, sleep=time.sleep):
        self.transaction, self.guard, self.receiver = transaction, guard, receiver
        self.clock, self.sleep = clock, sleep

    def preflight(self, boot_id, receiver_id):
        epoch, reason = _ready_epoch()
        if reason:
            raise Refused(reason)
        if self.transaction.has_intent():
            raise Refused('manual_arm_busy')
        try:
            self.transaction.files.sender_fault.lstat()
        except FileNotFoundError:
            pass
        else:
            # 任何现存故障标志均阻挡授权，包括不可信链接或非普通文件。
            raise Refused('stop_guard_not_ready')
        invocation = self.guard.invocation(latched=False)
        if not invocation:
            raise Refused('stop_guard_not_ready')
        owner = read_owner(boot_id, receiver_id)
        if neutral_epoch(owner) is None:
            raise Refused('control_owned' if owner.get('value') == 'owned' else 'control_unknown')
        hold = keyboard_sample(boot_id, receiver_id)
        if hold is None:
            raise Refused('operator_hold_stale')
        if hold.value.pressed:
            raise Refused('operator_hold_neutral_required')
        return epoch, invocation

    def activate(self, *, reusable_hold=False):
        try:
            # 与停车锁存复位共用互斥，拒绝在另一事务的临时布防窗口中启用。
            with self.transaction.locked():
                return self._activate_locked(reusable_hold=reusable_hold)
        except Refused as exc:
            return failure(str(exc))
        except BlockingIOError:
            return failure('manual_arm_busy')
        except (OSError, RuntimeError, subprocess.SubprocessError, TypeError, ValueError, KeyError, RecursionError):
            return failure('manual_arm_unavailable')

    def _activate_locked(self, *, reusable_hold):
        boot_id, receiver_id = identity()
        deadline = self.clock() + ACTIVATION_TIMEOUT
        epoch = self.preflight(boot_id, receiver_id)
        initial = grant_status(boot_id, receiver_id)
        if type(initial.get('serial')) is not int:
            raise Refused('local_grant_invalid')
        if initial.get('grant') is not None and initial.get('cockpit_managed') is not True:
            raise Refused('local_grant_active')
        props = self.receiver.status()
        active = props['ActiveState'] == 'active'
        if active and not self.receiver.manual_process(props):
            raise Refused('manual_arm_profile_unavailable')
        if props['ActiveState'] in ('inactive', 'failed') and props['MainPID'] == '0':
            self.receiver.start()
        elif not active:
            raise Refused('manual_arm_busy')

        def remaining():
            budget = deadline - self.clock()
            if budget <= 0:
                raise Refused('manual_arm_timeout')
            return min(.65, budget)

        while True:
            remaining()
            instance = self.receiver.manual_process(self.receiver.status())
            if instance:
                state = self.receiver.request('receiver-status', remaining())
                reason = state.get('reason')
                if state.get('source') == 'unitree' and state.get('armed') is True:
                    raise Refused('session_busy')
                if state.get('source') == 'unitree' and state.get('armed') is False:
                    if reason in ('ready', 'local_authorization_required'):
                        break
                    if reason not in ('stop_unconfirmed', 'standstill_required', 'feedback_stale', 'operator_hold_stale'):
                        raise Refused('manual_arm_not_ready')
            self.sleep(POLL_INTERVAL)

        if self.preflight(boot_id, receiver_id) != epoch:
            raise Refused('stop_guard_not_ready')
        if self.receiver.manual_process(self.receiver.status()) != instance:
            raise Refused('manual_arm_profile_unavailable')
        current = grant_status(boot_id, receiver_id)
        if current.get('serial') != initial['serial']:
            raise Refused('local_grant_serial_changed')
        serial = current['serial']
        if current['grant'] is not None:
            # 唯有受管接收器已停稳且无会话时，才能清理同一代的驾驶舱旧许可。
            if current.get('cockpit_managed') is not True:
                raise Refused('local_grant_active')
            revoked = revoke_cockpit(serial)
            if not revoked.get('ok'):
                raise Refused(revoked['reason'])
            serial = revoked['serial']
        remaining()
        granted = issue_cockpit(serial)
        if not granted.get('ok'):
            raise Refused(granted['reason'])
        created = granted['grant']['serial']
        may_have_acquired = False
        try:
            if self.preflight(boot_id, receiver_id) != epoch:
                raise Refused('stop_guard_not_ready')
            budget = remaining()
            may_have_acquired = True
            # 仅通过固定子命令选择接收器的会话模式，授权和看门狗沿用同一流程。
            operation = 'receiver-acquire-reusable-hold' if reusable_hold else 'receiver-acquire'
            payload = {'reusable_hold': True} if reusable_hold else {}
            receipt = self.receiver.request(operation, budget)
            if receipt.get('ok') is not True:
                reason = receipt.get('reason', 'motion_arbiter_unavailable')
                may_have_acquired = reason == 'motion_arbiter_unavailable'
                raise Refused(reason)
            result = _command_receipt('acquire', payload, receipt)
            if result.get('continuous_hold') is not True:
                raise Refused('manual_arm_not_ready')
            return result
        finally:
            if not may_have_acquired:
                # 在已知未取得会话时才清理本事务创建的许可。超时后由现有看门狗停车；
                # 下次点击必须先确认停稳，再按同代 CAS 清理旧许可。
                revoke_cockpit(created)


def read_request(connection, *, clock=time.monotonic):
    deadline = clock() + REQUEST_TIMEOUT
    record = bytearray()
    while len(record) < MAX_REQUEST_BYTES and b'\n' not in record:
        remaining = deadline - clock()
        if remaining <= 0:
            break
        connection.settimeout(remaining)
        chunk = connection.recv(MAX_REQUEST_BYTES - len(record))
        if not chunk:
            break
        record.extend(chunk)
    if not record.endswith(b'\n') or record.count(b'\n') != 1:
        raise ValueError('启用接口只接受单条有界请求')
    request = json.loads(record, object_pairs_hook=_distinct_fields)
    if request == {'op': 'activate'}:
        return False
    if request == {'op': 'activate_reusable_hold'}:
        return True
    raise ValueError('启用接口不接受其他操作或参数')


def serve(connection, arm, *, authorize=allowed_peer):
    try:
        if not authorize(connection):
            return
    except (OSError, KeyError, ValueError):
        return
    try:
        reusable_hold = read_request(connection)
    except (OSError, ValueError, RecursionError):
        result = failure('invalid_manual_command')
    else:
        result = arm.activate(reusable_hold=reusable_hold)
    try:
        connection.settimeout(.10)
        connection.sendall(json.dumps(result, separators=(',', ':')).encode('ascii') + b'\n')
    except OSError:
        # 意外断线后，本机接收器按既有的空闲看门狗撤销未继续保活的会话。
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('serve', 'receiver-status', 'receiver-acquire',
                                             'receiver-acquire-reusable-hold'))
    args = parser.parse_args(argv)
    if args.operation != 'serve':
        if os.geteuid() != pwd.getpwnam(GATEWAY_USER).pw_uid:
            parser.error('仅受管网关账号可查询接收器')
        client = LocalMotionClient(MOTION_SOCKET, peer_user=SENDER_USER, require_source=True)
        if args.operation == 'receiver-status':
            result = client.state()
        else:
            payload = {'reusable_hold': True} if args.operation == 'receiver-acquire-reusable-hold' else {}
            result = client.acquire(payload)[1]
        print(json.dumps(result, separators=(',', ':')))
        return
    if os.geteuid() != 0 or not RELEASE.is_relative_to('/opt/a2'):
        parser.error('仅 PC2 固定发布目录的 root 系统服务可以运行')
    arm = Arm(Transaction(GuardFiles(STATE_DIR, RUN_DIR)), GuardService(), ManagedReceiver())
    with socket.socket(fileno=os.dup(sys.stdin.fileno())) as connection:
        serve(connection, arm)


if __name__ == '__main__':
    main()
