#!/usr/bin/env python3
"""离线验证候选人工入口；实际 SDK 二进制只检查符号，绝不执行。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time


TEST_DIR = Path(__file__).resolve().parent
CANDIDATE_DIR = TEST_DIR.parent
DEFAULT_INCLUDE = CANDIDATE_DIR.parent.parent / "third_party/src/odin_ros_driver-0.14.5/include"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wait_for(predicate, description, processes=(), timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        if any(process.poll() is not None for process in processes):
            break
        time.sleep(0.02)
    raise AssertionError(f"等待失败：{description}")


def contents(path):
    return path.read_text() if path.is_file() else ""


def stop_group(process):
    # 失败时清理整组模拟进程，包括入口中的 tee、grep 和模拟建图程序。
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait(timeout=5)


def run_script(script, *args, env, timeout=10):
    process = subprocess.Popen(
        ["bash", script, *args], env=env, text=True,
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        output, _ = process.communicate("", timeout=timeout)
        return process.returncode, output
    except subprocess.TimeoutExpired:
        stop_group(process)
        output, _ = process.communicate()
        raise AssertionError(f"入口未按期退出：{script}\n{output}")


def write_executable(path, text):
    path.write_text(text)
    path.chmod(0o755)


class Fixtures:
    def __init__(self, root, fake_binary, bundle):
        self.root = root
        self.fake_binary = fake_binary
        self.bundle = bundle
        self.mock_bin = root / "mock-bin"
        self.mock_bin.mkdir()
        self.runtime = root / "runtime"
        self.runtime.mkdir()
        # 入口必须先找到这些替身；mock 永远不会转发到真实命令。
        write_executable(
            self.mock_bin / "pgrep",
            '#!/bin/sh\nexit "${A2_TEST_PGREP_STATUS:-1}"\n',
        )
        write_executable(
            self.mock_bin / "gnome-terminal",
            "#!/usr/bin/env python3\n"
            "import json, os, sys\n"
            "with open(os.environ['A2_TEST_TERMINAL_CALLS'], 'a') as output:\n"
            "    output.write(json.dumps(sys.argv[1:], ensure_ascii=False) + '\\n')\n",
        )
        self.env = {
            **os.environ,
            "PATH": f"{self.mock_bin}{os.pathsep}{os.environ['PATH']}",
            "XDG_RUNTIME_DIR": str(self.runtime),
            "A2_TEST_PGREP_STATUS": "1",
            "A2_ODIN_TEST_CASE": "entrypoints",
        }
        # 构建清单只列本目录文件，拒绝意外的外部路径，避免测试复制越界。
        self.names = []
        for line in (self.bundle / "current-program.sha256").read_text().splitlines():
            _, name = line.split(maxsplit=1)
            assert Path(name).name == name, f"清单含非本地文件：{name}"
            self.names.append(name)
        assert {"odin1-local-mapping", "enable-usb.sh", "run-visible.sh", "control.sh",
                "start-manual.sh", "sdk-files.sha256"}.issubset(self.names)

    def snapshot(self, name):
        directory = self.root / name
        directory.mkdir()
        for relative in self.names:
            if relative == "odin1-local-mapping":
                shutil.copy2(self.fake_binary, directory / relative)
            elif relative == "enable-usb.sh":
                # 禁止将真实 USB 脚本带入测试快照；这里只留下可断言的标记。
                write_executable(
                    directory / relative,
                    '#!/usr/bin/env bash\nset -euo pipefail\n'
                    'printf "mock USB only\\n" > "$(dirname -- "$0")/usb-mock.called"\n',
                )
            else:
                shutil.copy2(self.bundle / relative, directory / relative)
        (directory / "current-program.sha256").write_text("".join(
            f"{digest(directory / relative)}  {relative}\n" for relative in self.names
        ))
        assert digest(directory / "odin1-local-mapping") == digest(self.fake_binary)
        return directory


def manual_controls(fixture):
    directory = fixture.snapshot("manual controls")
    visible_output = directory / "visible-test.txt"
    control_output = directory / "control-test.txt"
    log = directory / "probe.log"
    visible = control = None
    with visible_output.open("w") as visible_log, control_output.open("w") as control_log:
        try:
            visible = subprocess.Popen(
                ["bash", directory / "run-visible.sh", directory], env=fixture.env,
                stdin=subprocess.DEVNULL, stdout=visible_log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            wait_for(lambda: "PROBE MAPPING_STARTED" in contents(log),
                     "模拟建图启动", (visible,))
            control = subprocess.Popen(
                ["bash", directory / "control.sh", directory], env=fixture.env,
                text=True, stdin=subprocess.PIPE, stdout=control_log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )

            def action(value):
                control.stdin.write(value + "\n")
                control.stdin.flush()

            action("s")
            wait_for(lambda: (directory / "save-map.busy").is_dir(),
                     "保存占用标记", (visible, control))
            action("s")
            wait_for(lambda: "本次 s 未新增保存请求" in contents(control_output),
                     "人工入口拒绝忙时重复保存", (visible, control))
            assert not (directory / "save-map.request").exists()
            wait_for(lambda: "PROBE MAP_SAVED" in contents(log),
                     "地图取回", (visible, control))
            maps = list(directory.glob("*.bin"))
            assert len(maps) == 1 and maps[0].read_bytes() == b"fake .bin 1"
            assert "FAKE STOP mode=SLAM" not in contents(log)
            action("c")
            wait_for(lambda: "PROBE MAPPING_CANCELLED mode=raw" in contents(log),
                     "人工取消建图进入 RAW", (visible, control))
            action("q")
            assert visible.wait(timeout=10) == 0
            control.stdin.close()
            assert control.wait(timeout=5) == 0
            output = contents(log)
            assert output.count("FAKE SAVE_FLAG value=1") == 1
            assert output.count("FAKE FETCH save=1") == 1
            assert "FAKE START mode=RAW" in output
            assert "PROBE API lidar_system_deinit rc=0" in output
            assert (directory / "exit-code.txt").read_text().strip() == "0"
            assert (directory / "cancel-map.accepted").is_file()
            assert (directory / "exit.accepted").is_file()
            assert (directory / "usb-mock.called").is_file()
        except Exception as error:
            raise AssertionError(
                f"人工入口测试失败：{error}\n{contents(visible_output)}\n{contents(control_output)}"
            ) from error
        finally:
            for process in (control, visible):
                if process is not None and process.poll() is None:
                    stop_group(process)
    print("PASS manual-s-busy-c-q", flush=True)


def reject_previous_runs(fixture):
    for marker in ("probe.log", "probe.pid", "exit-code.txt", ".run-started"):
        for dangling in (False, True):
            directory = fixture.snapshot(f"old-{marker}-{dangling}")
            previous = directory / marker
            if dangling:
                previous.symlink_to("missing-old-evidence")
            else:
                previous.write_bytes(b"original evidence\n")
            before = {p.name: digest(p) for p in directory.iterdir() if p.is_file()}
            rc, output = run_script(directory / "run-visible.sh", directory, env=fixture.env)
            assert rc == 2 and "拒绝复用已有运行目录" in output, output
            assert not (directory / "usb-mock.called").exists()
            assert {p.name: digest(p) for p in directory.iterdir() if p.is_file()} == before
            if dangling:
                assert previous.is_symlink() and os.readlink(previous) == "missing-old-evidence"
            else:
                assert previous.read_bytes() == b"original evidence\n"
    print("PASS reject-old-files-and-dangling-links", flush=True)


def reject_running_program(fixture):
    directory = fixture.snapshot("pgrep busy")
    rc, output = run_script(
        directory / "run-visible.sh", directory,
        env={**fixture.env, "A2_TEST_PGREP_STATUS": "0"},
    )
    assert rc == 2 and "已有 ODIN1 程序在运行" in output, output
    assert not (directory / "usb-mock.called").exists()
    assert not (directory / "probe.pid").exists()
    assert not (directory / "probe.log").exists()
    assert (directory / "exit-code.txt").read_text().strip() == "2"
    print("PASS reject-existing-process-before-usb", flush=True)


def start_manual_snapshot(fixture):
    source = fixture.snapshot("launcher source")
    destination = fixture.root / "fresh run snapshot"
    calls_path = fixture.root / "terminal-calls.jsonl"
    env = {**fixture.env, "A2_TEST_TERMINAL_CALLS": str(calls_path)}
    rc, output = run_script(source / "start-manual.sh", destination, env=env)
    assert rc == 0, output
    calls = [json.loads(line) for line in calls_path.read_text().splitlines()]
    assert len(calls) == 2
    for call, script in zip(calls, ("run-visible.sh", "control.sh")):
        assert call[0] == "--window" and call[1].startswith("--title=ODIN1 v0.14.5 ")
        assert call[2:] == ["--", "bash", str(destination / script), str(destination)]
    for name in (*fixture.names, "current-program.sha256"):
        assert digest(source / name) == digest(destination / name)
    assert not (destination / "usb-mock.called").exists()
    assert not (destination / "probe.pid").exists()
    # 同一目录再次启动必须拒绝；既不改快照，也不增加终端调用。
    before = {p.name: digest(p) for p in destination.iterdir()}
    rc, _ = run_script(source / "start-manual.sh", destination, env=env)
    assert rc != 0
    assert len(calls_path.read_text().splitlines()) == 2
    assert {p.name: digest(p) for p in destination.iterdir()} == before
    print("PASS manual-launcher-snapshot-and-window-arguments", flush=True)


def check_native_deinit(candidate):
    # 对真实二进制只使用 nm；执行路径永远只接收本脚本编译的 fake SDK 程序。
    symbols = subprocess.run(
        ["nm", "-C", "--defined-only", candidate],
        check=True, capture_output=True, text=True,
    ).stdout
    assert "lidar_system_deinit" in symbols
    for forbidden in ("__wrap_free", "__wrap_libusb_cancel_transfer", "odin_sdk_system_deinit"):
        assert forbidden not in symbols, f"仍包含旧兼容符号：{forbidden}"
    print("PASS candidate-native-deinit-symbols", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-include", type=Path, default=DEFAULT_INCLUDE)
    parser.add_argument("--candidate-binary", required=True, type=Path,
                        help="build.sh 输出目录下的 odin1-local-mapping")
    args = parser.parse_args()
    check_native_deinit(args.candidate_binary)
    with tempfile.TemporaryDirectory(prefix="odin1-entrypoint-test-") as temporary:
        root = Path(temporary)
        binary = root / "fake-sdk-program"
        subprocess.run([
            "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic", "-pthread",
            "-I", args.sdk_include, CANDIDATE_DIR / "odin1-local-mapping.cpp",
            CANDIDATE_DIR / "odin1-local-mapping-support.cpp", TEST_DIR / "fake_sdk.cpp",
            "-Wl,--wrap=_ZNSt6chrono3_V212steady_clock3nowEv", "-o", binary,
        ], check=True)
        fixture = Fixtures(root, binary, args.candidate_binary.resolve().parent)
        manual_controls(fixture)
        reject_previous_runs(fixture)
        reject_running_program(fixture)
        start_manual_snapshot(fixture)


if __name__ == "__main__":
    main()
