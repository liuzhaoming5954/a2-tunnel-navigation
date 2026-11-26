"""只读核验 PC2 原厂手柄反馈及其连续周期。"""

import json
import math
from pathlib import Path
import pwd
import re
import secrets
import socket
import stat
import struct
import time
from uuid import UUID


# 原厂手柄操作会优先接管机身。保留旧观察器的报文字段，单独识别可信回中周期。
_SOCKET = Path('/run/a2-vendor-owner/feedback.sock')
_UNIT_GROUP = '/system.slice/a2-vendor-owner.service'
_SERVICE_USER = 'a2-owner-input'
HANDSET_SOURCE = 'rt/lf/lowstate/wireless_remote'
HANDSET_NEUTRAL_REASON = 'handset_neutral_release_unconfirmed'
_MAX_BYTES = 2048
_MAX_AGE_S = .20
_DEADLINE_S = .015
_CRED = struct.Struct('3i')


def _unknown(reason):
    # 不以读文件或调用本函数的时间填入机身采样时间，不为失效的旧证据续期。
    return {'value': 'unknown', 'observed_at': None, 'source': None, 'epoch': None, 'reason': reason}


def _system_unit(groups):
    # systemd 的 cgroup v2 与 v1 命名层级均按完整系统服务路径核对。
    for line in groups.splitlines():
        hierarchy, controllers, group = line.split(':', 2)
        if (hierarchy == '0' and not controllers) or 'name=systemd' in controllers.split(','):
            return group == _UNIT_GROUP
    return False


def _receive(request):
    # 接口只允许 status 请求。签发权、锁存控制、用户提交数据及模拟模式均不在此协议内。
    parent = _SOCKET.parent.lstat()
    try:
        service_uid = pwd.getpwnam(_SERVICE_USER).pw_uid
    except KeyError as error:
        raise PermissionError('原厂只读服务身份缺失') from error
    if (not stat.S_ISDIR(parent.st_mode) or not service_uid or parent.st_uid != service_uid
            or parent.st_mode & 0o022):
        raise PermissionError('原厂反馈套接字目录不可信')
    if not stat.S_ISSOCK(_SOCKET.lstat().st_mode):
        raise PermissionError('原厂反馈目标必须是本地套接字')
    start = time.monotonic()
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as peer:
        def remaining():
            seconds = _DEADLINE_S - (time.monotonic() - start)
            if seconds <= 0:
                raise TimeoutError('原厂反馈超时')
            peer.settimeout(seconds)

        remaining()
        peer.connect(str(_SOCKET))
        pid, uid, _ = _CRED.unpack(peer.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, _CRED.size))
        if pid <= 1 or uid != service_uid or not _system_unit(Path(f'/proc/{pid}/cgroup').read_text()):
            raise PermissionError('原厂反馈对端未通过系统服务身份核验')
        remaining()
        packet = json.dumps(request, separators=(',', ':')).encode()
        if peer.send(packet) != len(packet):
            raise OSError('原厂反馈查询未送达')
        remaining()
        data, _, flags, _ = peer.recvmsg(_MAX_BYTES)
        if not data or flags & socket.MSG_TRUNC:
            raise ValueError('原厂反馈报文为空或超过上限')
        return json.loads(data, object_pairs_hook=_distinct_fields)


def _distinct_fields(pairs):
    fields = {}
    for name, value in pairs:
        if name in fields:
            raise ValueError('原厂反馈报文存在重复字段')
        fields[name] = value
    return fields


def neutral_epoch(report):
    """仅取自 read_owner 已验真的原厂手柄回中周期。"""
    if (report.get('source') == HANDSET_SOURCE and report.get('value') == 'unknown'
            and report.get('reason') == HANDSET_NEUTRAL_REASON):
        epoch = report.get('neutral_epoch')
        if isinstance(epoch, str) and 0 < len(epoch) <= 256:
            return epoch
    return None


def read_owner(boot_id, receiver_id, *, now=None):
    """读取原厂手柄状态和采样时间。周期标识用于绑定本地授权。"""
    request = {'schema_version': 1, 'op': 'status', 'boot_id': boot_id, 'receiver_id': receiver_id,
               'nonce': secrets.token_hex(16)}
    started = time.monotonic()
    try:
        report = _receive(request)
        if (not isinstance(report, dict) or type(report.get('schema_version')) is not int
                or report.get('schema_version') != 1 or any(report.get(k) != request[k]
                for k in ('nonce', 'boot_id', 'receiver_id'))):
            return _unknown('vendor_owner_invalid')
        source, value, observed = report['source'], report['value'], report['observed_at']
        counters = {}
        handset_neutral = False
        if source == HANDSET_SOURCE:
            # 低层字段提供按键和摇杆观测，接口中的原厂权属字段维持旧协议。
            if value == 'released':
                return _unknown('vendor_owner_invalid')
            frames, tick = report.get('frame_count'), report.get('source_tick')
            if (type(frames) is not int or frames < 0 or
                    (tick is not None and (type(tick) is not int or not 0 <= tick < 2**32))
                    or (frames > 0) != (tick is not None)):
                return _unknown('vendor_owner_invalid')
            counters = {'frame_count': frames, 'source_tick': tick}
            if value == 'unknown' and observed is None and report.get('reason') in (
                    'handset_no_frames', 'handset_invalid_frame', 'handset_frame_stale'):
                return {**_unknown(report['reason']), 'source': source, **counters}
            if not frames or report.get('reason') != ('handset_active' if value == 'owned'
                                                      else HANDSET_NEUTRAL_REASON):
                return _unknown('vendor_owner_invalid')
            handset_neutral = value == 'unknown'
        if (not isinstance(source, str) or not re.fullmatch(r'[A-Za-z0-9_.:/-]{1,128}', source)
                or value not in ('owned', 'released', 'unknown') or type(observed) not in (int, float)
                or not math.isfinite(observed) or observed < 0):
            return _unknown('vendor_owner_invalid')
        # 原始观测的年龄包括请求传输耗时。独立来源显式报告未知也会立即取消连续性。
        received = time.monotonic()
        current = received if now is None else now + (received - started)
        if not 0 <= current - observed < _MAX_AGE_S:
            return _unknown('vendor_owner_stale')
        if value == 'unknown' and not handset_neutral:
            return {'value': value, 'source': source, 'observed_at': observed,
                    'epoch': None, 'reason': 'vendor_owner_unknown', **counters}
        # producer_instance 随生产进程重启更新。原厂来源失效、丢帧超时、重新连接、
        # 权属变化时，生产方均须递增 generation；即使最终同为 released，也不能沿用旧许可。
        instance, generation = report['producer_instance'], report['generation']
        if not isinstance(instance, str) or len(instance) != 36:
            return _unknown('vendor_owner_invalid')
        producer = UUID(instance)
        if (str(producer) != instance or producer.int == 0
                or type(generation) is not int or not 1 <= generation <= 2**63 - 1):
            return _unknown('vendor_owner_invalid')
        epoch = f'{instance}:{generation}'
        if handset_neutral:
            return {'value': value, 'source': source, 'observed_at': observed, 'epoch': None,
                    'reason': HANDSET_NEUTRAL_REASON, 'neutral_epoch': epoch, **counters}
        return {'value': value, 'source': source, 'observed_at': observed,
                'epoch': epoch, 'reason': 'vendor_owner_observed', **counters}
    except PermissionError:
        return _unknown('vendor_owner_untrusted')
    except OSError:
        return _unknown('vendor_owner_unavailable')
    except (ValueError, TypeError, KeyError, OverflowError, RecursionError):
        return _unknown('vendor_owner_invalid')
