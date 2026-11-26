#!/usr/bin/env python3
"""只读采样 Linux 键盘的方向键状态，供模拟底盘联调。"""

import argparse
from contextlib import contextmanager
import errno
import fcntl
import json
import os
from pathlib import Path
import re
import stat
import struct
from threading import Lock
import time
from uuid import uuid4


EV_SYN = 0
EV_KEY = 1
SYN_REPORT = 0
SYN_DROPPED = 3
KEY_DIRECTIONS = {
    17: "forward",    # W
    30: "left",       # A
    31: "backward",   # S
    32: "right",      # D
    103: "forward",   # ↑
    105: "left",      # ←
    106: "right",     # →
    108: "backward",  # ↓
}

# Linux input.h 的本机结构和 asm-generic ioctl 编码，适用于开发机 x86_64 和常规 arm64。
_INPUT_EVENT = struct.Struct("@llHHi")
_KEY_BITMAP_SIZE = 96  # KEY_MAX = 0x2ff。
_EVIOCGKEY = (2 << 30) | (_KEY_BITMAP_SIZE << 16) | (ord("E") << 8) | 0x18
_READ_LIMIT = 256 * _INPUT_EVENT.size


class KeyboardHoldState:
    """用事件记录按压周期，只在键位与内核即时位图一致时报告按住。"""

    def __init__(self):
        self.keys = frozenset()
        self.epoch = uuid4().hex
        self._needs_neutral = True
        self._dropped = False

    def invalidate(self):
        # 数据不连续时拒绝沿用旧周期。必须先观察到真实回中，再接受新按压。
        self.keys = frozenset()
        self.epoch = uuid4().hex
        self._needs_neutral = True

    def event(self, event_type, code, value):
        if event_type == EV_SYN and code == SYN_DROPPED:
            self.invalidate()
            self._dropped = True
            return
        if self._dropped:
            # 内核要求忽略溢出后到下一个 SYN_REPORT 的事件，包括该结束事件。
            if event_type == EV_SYN and code == SYN_REPORT:
                self._dropped = False
            return
        if event_type != EV_KEY or code not in KEY_DIRECTIONS or value == 2:
            return
        if value not in (0, 1):
            self.invalidate()
            return
        keys = self.keys | {code} if value else self.keys - {code}
        if keys != self.keys:
            self.keys = keys
            self.epoch = uuid4().hex
        if len(keys) > 1:
            # 同一方向的两个键也不能替代原按键，避免从旧按压直接续为新按压。
            self._needs_neutral = True

    def snapshot(self, pressed_keys):
        current = frozenset(code for code in pressed_keys if code in KEY_DIRECTIONS)
        if not self._dropped:
            if current != self.keys:
                self.invalidate()
                self.keys = current
            if not current:
                self._needs_neutral = False
        direction = None
        if not self._dropped and not self._needs_neutral and len(current) == 1:
            direction = KEY_DIRECTIONS[next(iter(current))]
        return {"pressed": direction is not None, "epoch": self.epoch, "direction": direction}


def _pressed_keys(fd):
    bitmap = bytearray(_KEY_BITMAP_SIZE)
    fcntl.ioctl(fd, _EVIOCGKEY, bitmap)
    return frozenset(code for code in KEY_DIRECTIONS if bitmap[code // 8] & (1 << (code % 8)))


class KeyboardObserver:
    """对显式选择的 evdev 设备做单次同步采样。"""

    def __init__(self, fd):
        self.fd = fd
        self.state = KeyboardHoldState()
        self._lock = Lock()

    def _read_events(self):
        try:
            raw = os.read(self.fd, _READ_LIMIT)
        except BlockingIOError:
            return
        if not raw:
            raise OSError(errno.ENODEV, "键盘设备已断开")
        if len(raw) % _INPUT_EVENT.size:
            raise OSError(errno.EIO, "键盘事件长度异常")
        if len(raw) == _READ_LIMIT:
            # 本次读取达到限量，可能仍有积压。立刻废弃连续按压证据。
            self.state.invalidate()
            return
        for _seconds, _microseconds, event_type, code, value in _INPUT_EVENT.iter_unpack(raw):
            self.state.event(event_type, code, value)

    def sample(self):
        with self._lock:
            self._read_events()
            current = _pressed_keys(self.fd)
            if self.state.keys != current:
                # 状态变动可能恰好落在非阻塞读与 ioctl 之间。最多补读一次，不等待设备。
                self._read_events()
                current = _pressed_keys(self.fd)
            return self.state.snapshot(current)


@contextmanager
def open_keyboard(device):
    """只读打开选定设备，接受 /dev/input/by-id 指向的 Linux evdev 节点。"""
    path = Path(device).resolve(strict=True)
    if not re.fullmatch(r"/dev/input/event[0-9]+", str(path)):
        raise ValueError("请选择 /dev/input/eventN 或指向该节点的设备链接")
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        if not stat.S_ISCHR(os.fstat(fd).st_mode):
            raise ValueError("所选设备不是 Linux evdev 字符设备")
        yield KeyboardObserver(fd)
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, type=Path, help="当前用户有权读取的 evdev 键盘设备")
    args = parser.parse_args()
    try:
        with open_keyboard(args.device) as keyboard:
            while True:
                started = time.monotonic()
                print(json.dumps(keyboard.sample(), separators=(",", ":")), flush=True)
                time.sleep(max(0.0, 0.05 - (time.monotonic() - started)))
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError) as exc:
        parser.exit(1, f"键盘采样退出：{exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
