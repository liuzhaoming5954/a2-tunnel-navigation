#!/usr/bin/env python3
"""搭建独立的手动遥控模拟链路，可单独在 PC2 上启动回环网关和模拟底盘。"""

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import secrets
import select
import signal
import subprocess
import sys
import tempfile
from threading import Event, Lock, Thread
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/a2_gateway"))

from a2_gateway.motion import LocalMotionClient
from a2_gateway.manual_link import read_key
from a2_gateway.server import DEFAULT_STATIC_ROOT, GatewayServer


class DemoTelemetry:
    """用模拟底盘实际收到的运动、停车事件生成机身反馈，让验收能观察指令有没有到达。"""

    def __init__(self, receiver):
        self.receiver = receiver
        self.lock = Lock()
        self.linear = self.angular = 0.0
        self.reader = Thread(target=self.read_output, name="mock-chassis-output", daemon=True)

    def close(self):
        stop_receiver(self.receiver)
        if self.reader.ident is not None:
            self.reader.join(timeout=2)

    def read_output(self):
        for line in self.receiver.stdout:
            try:
                event = json.loads(line)
            except ValueError:
                print(line, end="", file=sys.stderr, flush=True)
                continue
            if event.get("event") in ("move", "stop"):
                with self.lock:
                    self.linear = event["linear_mps"]
                    self.angular = event["angular_radps"]
            # 直接保留接收端单调时刻，不能用浏览器发起请求的时间代替底盘接收时刻。
            print(line, end="", flush=True)

    def snapshot(self):
        with self.lock:
            moving = bool(self.linear or self.angular)
            return {"robot": {
                "id": "A2-MOCK", "model": "模拟 A2", "online": self.receiver.poll() is None,
                "mode": "moving" if moving else "standing", "linear_mps": self.linear,
                "angular_radps": self.angular, "telemetry_stamp_ms": time.time_ns() // 1_000_000,
                "telemetry_age_ms": 0,
            }}


def stop_receiver(process):
    """只终止本次创建的模拟接收器，并等它释放本次私有套接字。"""
    if process.poll() is None:
        process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def watch_maintenance_link(done, parent_pid):
    """无伪终端的 SSH 会话有时只关闭输出管道，静止的模拟器也须及时退出。"""
    poller = select.poll()
    poller.register(sys.stdout.fileno(), select.POLLERR | select.POLLHUP | select.POLLNVAL)
    while not done.wait(.2):
        if os.getppid() != parent_pid or poller.poll(0):
            os.kill(os.getpid(), signal.SIGTERM)
            return


def serve(port, static_root, *, pc2_only=False, key_file=None):
    if not pc2_only and not (static_root / "index.html").is_file():
        raise ValueError("请先构建 Vue 页面，或用 --static-root 指定本次模拟构建的目录")
    # 独立 PC2 模式必须与站端配对。双端同机时只在内存中使用一次性的模拟凭据。
    key = read_key(key_file) if pc2_only else secrets.token_urlsafe(32)
    parent_pid = os.getppid()
    with ExitStack() as stack:
        work = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix="a2-manual-demo-")))
        socket_path = work / "manual.sock"
        latch = work / "mock-A-stop"
        # 端点全部由本次创建，无外部 PC2 地址、硬件参数或现场 SDK 的配置入口。
        receiver = stack.enter_context(subprocess.Popen(
            [sys.executable, "-I", "-S", "-B", "-u", str(ROOT / "scripts/run_a2_manual_receiver.py"),
             "--backend", "mock", "--socket", str(socket_path), "--mock-stop-file", str(latch),
             "--mock-parent-pid", str(os.getpid())],
            stdout=subprocess.PIPE, text=True,
        ))
        stack.callback(stop_receiver, receiver)
        telemetry = DemoTelemetry(receiver)
        deadline = time.monotonic() + 5
        while not socket_path.is_socket():
            if receiver.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("模拟底盘接收器未就绪")
            time.sleep(0.02)
        pc2 = stack.enter_context(GatewayServer(
            ("127.0.0.1", port if pc2_only else 0), telemetry, role="pc2", mode="manual_control",
            manual_control=LocalMotionClient(socket_path, require_source=True), manual_key=key,
        ))
        if pc2_only:
            # 保留首行作为远端端口的机器可读回执，输出事件的线程在公布地址后才启动。
            server = pc2
            watch_done = Event()
            watcher = Thread(target=watch_maintenance_link, args=(watch_done, parent_pid), daemon=True)
            watcher.start()
            stack.callback(watch_done.set)
            print(f"模拟 PC2 http://127.0.0.1:{pc2.server_port}", flush=True)
        else:
            from a2_gateway.station import StationSource

            thread = Thread(target=pc2.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
            thread.start()
            stack.callback(thread.join)
            stack.callback(pc2.shutdown)
            source = StationSource(f"http://127.0.0.1:{pc2.server_port}", manual_key=key)
            server = stack.enter_context(GatewayServer(
                ("127.0.0.1", port), source, role="station", mode="manual_control",
                manual_control=source, manual_key=key, static_root=static_root, odin1_map_dir=work,
            ))
            print(f"模拟驾驶舱 http://127.0.0.1:{server.server_port}/#/maintenance", flush=True)
        print(f"模拟原厂遥控 A 键：新建 {latch} 即触发本次模拟停车锁存。", flush=True)
        telemetry.reader.start()
        server.serve_forever(poll_interval=0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=0, help="回环监听端口，默认由系统分配")
    parser.add_argument("--pc2-only", action="store_true", help="仅启动 PC2 回环模拟接口和独立模拟底盘")
    parser.add_argument("--manual-key-file", type=Path, help="独立 PC2 模拟接口与站端配对的私有密钥文件")
    parser.add_argument("--static-root", type=Path, default=DEFAULT_STATIC_ROOT, help="Vue 构建目录")
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error("端口超出范围")
    if args.pc2_only != (args.manual_key_file is not None):
        parser.error("--pc2-only 和 --manual-key-file 必须配对使用")
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    signal.signal(signal.SIGHUP, signal.default_int_handler)
    try:
        serve(args.port, args.static_root, pc2_only=args.pc2_only, key_file=args.manual_key_file)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"模拟驾驶舱启动失败：{exc}\n")


if __name__ == "__main__":
    main()
