"""PC2 本地遥控许可，签发、撤销及读取均与驾驶舱的 HTTP 权限分开。"""

from contextlib import contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import secrets
import stat
import time

from maintenance_arbiter import Direction, SITE_FRONT_SECONDS


DIRECTORY = Path('/run/a2-manual-safety')
FILENAME = 'local-grant.json'
READY = Path('/run/a2-remote-stop/ready')
LATCH = Path('/var/lib/a2-remote-stop/latched')
# 普通单次会话的上限。重复点动的单段期限由接收器限制。
MAX_SECONDS = 30
DIRECTIONS = {v.value for v in Direction}
TRUSTED_UID = 0


def identity():
    return (Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'unitree-a2-' + Path('/etc/machine-id').read_text().strip())


def _trusted(info, kind):
    if not kind(info.st_mode) or info.st_uid != TRUSTED_UID or info.st_mode & 0o022:
        raise PermissionError('本地许可或守护路径不可信')


def _ready_epoch():
    """守护每次布防独占新建 ready。周期内的授权不能跨越删除和重建。"""
    try:
        # 管理员签发时可直接读取持久锁存，遇到残留 ready 也必须拒绝。
        LATCH.lstat()
        return None, 'stop_latched'
    except FileNotFoundError:
        pass
    except PermissionError:
        # 普通读者不能遍历持久目录；原守护在锁存和退出前会撤去运行目录中的 ready。
        if os.geteuid() == TRUSTED_UID:
            return None, 'stop_guard_not_ready'
    try:
        info = READY.parent.lstat()
        _trusted(info, stat.S_ISDIR)
        marker = READY.lstat()
        _trusted(marker, stat.S_ISREG)
    except FileNotFoundError:
        return None, 'stop_guard_not_ready'
    return [marker.st_dev, marker.st_ino, marker.st_ctime_ns], None


def _control_epoch(boot_id, receiver_id, now):
    # A2 使用经实机反馈验证的回中周期。手柄介入或反馈断流后，旧许可不能沿用。
    from manual_authority import neutral_epoch, read_owner
    owner = read_owner(boot_id, receiver_id, now=now)
    centered = neutral_epoch(owner)
    if centered is not None:
        return {'source': owner['source'], 'epoch': centered, 'basis': 'handset_neutral'}, None
    if owner.get('value') != 'released':
        return None, 'control_owned' if owner.get('value') == 'owned' else 'control_unknown'
    epoch = owner.get('epoch')
    if not isinstance(epoch, str) or not epoch or len(epoch) > 256:
        return None, 'owner_continuity_unknown'
    source = owner.get('source')
    if not isinstance(source, str) or not source:
        return None, 'owner_continuity_unknown'
    return {'source': source, 'epoch': epoch}, None


@contextmanager
def _directory(create=False):
    created = False
    if create:
        # /run 在 PC2 上由系统管理，禁止通过环境变量把许可重定向到工作目录。
        try:
            DIRECTORY.mkdir(mode=0o755)
            created = True
        except FileExistsError:
            pass
    fd = os.open(DIRECTORY, os.O_DIRECTORY | os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(fd)
        _trusted(info, stat.S_ISDIR)
        if created and stat.S_IMODE(info.st_mode) != 0o755:
            # 首次签发也须让非特权发令进程读取。仅修正新目录，保留已有目录的权限。
            os.fchmod(fd, 0o755)
        if create:
            # 独占同一目录 inode，串行化两名操作员或重试进程的比较、签发与撤销。
            fcntl.flock(fd, fcntl.LOCK_EX)
        yield fd
    finally:
        os.close(fd)


def _record(fd):
    try:
        rawfd = os.open(FILENAME, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW, dir_fd=fd)
    except FileNotFoundError:
        return {}
    with os.fdopen(rawfd, 'rb') as source:
        _trusted(os.fstat(source.fileno()), stat.S_ISREG)
        raw = source.read(4097)
    if len(raw) > 4096:
        raise ValueError('本地授权记录过大')
    data = json.loads(raw)
    if (not isinstance(data, dict) or data.get('version') != 1
            or type(data.get('serial')) is not int or data['serial'] < 1
            or not isinstance(data.get('boot_id'), str) or not isinstance(data.get('receiver_id'), str)):
        raise ValueError('本地授权记录格式无效')
    return data


def _write(fd, data):
    # 同目录独占临时文件，写完后原子替换。读取方只能看到完整的旧或新版本。
    name = '.grant-' + secrets.token_hex(12)
    handle = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=fd)
    try:
        with os.fdopen(handle, 'w', encoding='utf-8') as target:
            json.dump(data, target, allow_nan=False)
            target.flush()
            os.fchmod(target.fileno(), 0o644)
            os.fsync(target.fileno())
        os.replace(name, FILENAME, src_dir_fd=fd, dst_dir_fd=fd)
        os.fsync(fd)
    finally:
        try:
            os.unlink(name, dir_fd=fd)
        except FileNotFoundError:
            pass


def _active(data, boot_id, receiver_id, ready, now):
    grant = data.get('grant')
    if grant is None:
        return None, 'local_authorization_required'
    if not isinstance(grant, dict):
        raise ValueError('本地授权字段无效')
    if (data['boot_id'] != boot_id or data['receiver_id'] != receiver_id
            or data['guard_epoch'] != ready):
        return None, 'local_grant_epoch_changed'
    start, end = grant['issued_at'], grant['expires_at']
    directions = grant['directions']
    continuous_modes = {'repeated_actions', 'continuous_hold'} & grant.keys()
    if (type(start) not in (float, int) or not math.isfinite(start)
            or type(grant.get('serial')) is not int or grant['serial'] != data['serial']
            or not isinstance(directions, list) or not directions or not set(directions) <= DIRECTIONS):
        raise ValueError('本地授权范围或时钟无效')
    if continuous_modes:
        # 这两种模式使用不同的起步、停车语义。录入多种模式或非布尔真值时，拒绝整份许可。
        if (len(continuous_modes) != 1 or grant[next(iter(continuous_modes))] is not True
                or end is not None or 'site_front_confirmed' in grant):
            raise ValueError('持续操作许可格式无效')
    elif (type(end) not in (float, int) or not math.isfinite(end)
          or not 0 < end-start <= MAX_SECONDS):
        raise ValueError('单次许可的截止时间无效')
    if 'site_front_confirmed' in grant and (
            grant['site_front_confirmed'] is not True or directions != [Direction.FORWARD.value]
            or end != start + SITE_FRONT_SECONDS):
        raise ValueError('现场前向许可只能用于一次五秒直行')
    if now < start or (end is not None and now >= end):
        return None, 'local_grant_expired'
    epoch, reason = _control_epoch(boot_id, receiver_id, now)
    if reason or epoch != data['owner_epoch']:
        return None, reason or 'local_grant_epoch_changed'
    return grant, 'granted'


def grant_status(boot_id=None, receiver_id=None, *, now=None):
    """给机载仲裁器/监督方的只读接口。读状态不创建许可，不更新过期时间。"""
    try:
        if boot_id is None or receiver_id is None:
            boot_id, receiver_id = identity()
        try:
            with _directory() as fd:
                data = _record(fd)
        except FileNotFoundError:
            data = {}
        serial = data.get('serial', 0)
        ready, reason = _ready_epoch()
        if reason:
            return {'serial': serial, 'grant': None, 'reason': reason}
        grant, reason = _active(data, boot_id, receiver_id, ready, time.monotonic() if now is None else now)
        return {'serial': serial, 'grant': grant, 'reason': reason}
    except (OSError, ValueError, KeyError, TypeError, OverflowError, RecursionError):
        return {'serial': None, 'grant': None, 'reason': 'local_grant_invalid'}


def _rejection(reason):
    return {'ok': False, 'reason': reason}


def issue(expected_serial, seconds, directions, site_front_confirmed=False, *,
          repeated_actions=False, continuous_hold=False):
    """签一次本地许可，使用前一代序号进行比较交换，重复调用不能续期。"""
    if os.geteuid() != TRUSTED_UID:
        return _rejection('local_admin_required')
    try:
        requested = tuple(directions)
        unique_directions = set(requested)
    except TypeError:
        return _rejection('invalid_local_grant')
    if (type(expected_serial) is not int or expected_serial < 0
            or not requested or not unique_directions <= DIRECTIONS
            or type(site_front_confirmed) is not bool or type(repeated_actions) is not bool
            or type(continuous_hold) is not bool or (continuous_hold and repeated_actions)):
        return _rejection('invalid_local_grant')
    if repeated_actions or continuous_hold:
        if seconds is not None or site_front_confirmed:
            return _rejection('invalid_local_grant')
    elif type(seconds) is not int or not 1 <= seconds <= MAX_SECONDS:
        return _rejection('invalid_local_grant')
    # 现场确认只绑定本轮五秒前向许可，全部守护、身份和序号检查仍共用下方路径。
    if site_front_confirmed and (seconds != SITE_FRONT_SECONDS or requested != (Direction.FORWARD.value,)):
        return _rejection('invalid_local_grant')
    boot_id, receiver_id = identity()
    ready, reason = _ready_epoch()
    if reason:
        return _rejection(reason)
    # 只在签发入口读取系统管理器；接收器的高频看门狗不执行外部命令。
    from a2_gateway.remote_stop_status import _read_guard
    if _read_guard() != ('running', False):
        return _rejection('stop_guard_not_ready')
    owner, reason = _control_epoch(boot_id, receiver_id, time.monotonic())
    if reason:
        return _rejection(reason)
    with _directory(create=True) as fd:
        previous = _record(fd)
        if previous.get('serial', 0) != expected_serial:
            return _rejection('local_grant_serial_changed')
        now = time.monotonic()
        active, _ = _active(previous, boot_id, receiver_id, ready, now)
        if active:
            return _rejection('local_grant_active')
        current_ready, reason = _ready_epoch()
        if reason or current_ready != ready:
            return _rejection(reason or 'local_grant_epoch_changed')
        current_owner, reason = _control_epoch(boot_id, receiver_id, now)
        if reason or current_owner != owner:
            return _rejection(reason or 'local_grant_epoch_changed')
        grant = {'serial': expected_serial + 1, 'issued_at': now,
                 'expires_at': None if repeated_actions or continuous_hold else now + seconds,
                 'directions': sorted(unique_directions)}
        if repeated_actions:
            grant['repeated_actions'] = True
        if continuous_hold:
            # 许可只标明模式和方向。接收器在启用该模式时独立确认按键并决定运动是否能继续。
            grant['continuous_hold'] = True
        if site_front_confirmed:
            grant['site_front_confirmed'] = True
        record = {'version': 1, 'boot_id': boot_id, 'receiver_id': receiver_id,
                  'serial': grant['serial'], 'guard_epoch': ready, 'owner_epoch': owner, 'grant': grant}
        _write(fd, record)
    return {'ok': True, 'grant': grant}


def revoke():
    """撤销会增加代号，阻止撤销前排队的签发请求。无人授权时也保留此次撤销的代号。"""
    if os.geteuid() != TRUSTED_UID:
        return _rejection('local_admin_required')
    boot_id, receiver_id = identity()
    with _directory(create=True) as fd:
        serial = _record(fd).get('serial', 0) + 1
        _write(fd, {'version': 1, 'boot_id': boot_id, 'receiver_id': receiver_id,
                    'serial': serial, 'grant': None})
    return {'ok': True, 'serial': serial, 'grant': None}
