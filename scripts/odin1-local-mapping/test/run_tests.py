#!/usr/bin/env python3
"""用模拟 SDK 验证 ODIN1 建图会话，无需连接设备。"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


TEST_DIR = Path(__file__).resolve().parent
BENCH_DIR = TEST_DIR.parent
SDK_INCLUDE = BENCH_DIR.parents[1] / "third_party/src/odin_ros_driver-0.14.5/include"


class Probe:
    def __init__(self, binary, directory, case, *args):
        self.directory = directory / case
        self.directory.mkdir()
        self.log_file = self.directory / "probe.log"
        with self.log_file.open("w") as log:
            self.process = subprocess.Popen(
                [binary, self.directory, *args],
                env={**os.environ, "A2_ODIN_TEST_CASE": case},
                stdout=log,
                stderr=log,
            )

    def output(self):
        return self.log_file.read_text()

    def wait_for(self, marker, count=1, timeout=7):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            output = self.output()
            if output.count(marker) >= count:
                return output
            if self.process.poll() is not None:
                break
            time.sleep(0.025)
        raise AssertionError(f"未看到预期日志 {marker!r}")

    def request(self, name):
        (self.directory / f"{name}.request").touch(exist_ok=False)

    def finish(self, interrupt=True):
        if interrupt:
            self.process.send_signal(signal.SIGINT)
        return self.process.wait(timeout=15)


def assert_in_order(output, *markers):
    position = 0
    for marker in markers:
        position = output.index(marker, position) + len(marker)


def assert_mapping_active(probe):
    assert probe.process.poll() is None
    assert "FAKE STOP" not in probe.output()
    assert not list(probe.directory.glob("*.bin"))


def assert_map_then_raw(probe, timeout=7):
    probe.wait_for("PROBE MAP_SAVED", timeout=timeout)
    # 等待保存循环已经回到建图，检查切换 RAW 只能由人工停止请求启动。
    output = probe.wait_for("maps_saved=1")
    assert probe.process.poll() is None
    assert "FAKE STOP mode=SLAM" not in output
    maps = list(probe.directory.glob("*.bin"))
    assert len(maps) == 1 and maps[0].read_bytes() == b"fake .bin 1"
    probe.request("cancel-map")
    probe.wait_for("FAKE START mode=RAW")
    assert probe.finish() == 130
    output = probe.output()
    assert_in_order(
        output,
        "FAKE DEPTH hz=10",
        "FAKE CONFIGURE mode=SLAM",
        "PROBE MODE_READBACK phase=before_start requested=1 rc=0 value=1",
        "FAKE START mode=SLAM map_mode=1",
        "FAKE SAVE_FLAG value=0",
        "PROBE SAVE_MAP_READBACK phase=after_start requested=0 rc=0 value=0",
        "FAKE SAVE_FLAG value=1",
        "FAKE SAVE_POLL value=1",
        "FAKE SAVE_POLL value=0",
        "FAKE FETCH save=1",
        "PROBE MAP_SAVED",
        "FAKE STOP mode=SLAM",
        "FAKE CONFIGURE mode=RAW",
        "FAKE ACTIVATE type=",
        "FAKE START mode=RAW",
        "FAKE STOP mode=RAW",
    )
    assert output.count("FAKE ACTIVATE type=") == 3


def save_after_previous_deadline(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.wait_for("PROBE MAPPING_DURATION_IGNORED")
    # 复现以前会丢失地图的等候场景，故意越过命令行传入的 10 秒。
    time.sleep(10.5)
    assert_mapping_active(probe)
    live_output = probe.output()
    assert "PROBE MAPPING_ACTIVE" in live_output
    assert "PROBE MAPPING_DEVICE_STATUS phase=after_start status=unavailable" in live_output
    assert "cloud_n=" not in live_output and "odom_n=" not in live_output
    probe.request("save-map")
    assert_map_then_raw(probe)


def save_default_session(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.wait_for("FAKE ENCRYPTED_LOG_READY")
    probe.request("save-map")
    assert_map_then_raw(probe)


def mapping_status_is_diagnostic(probe):
    probe.wait_for("PROBE MAPPING_DEVICE_STATUS phase=after_start status=fresh")
    probe.request("save-map")
    assert_map_then_raw(probe)
    output = probe.output()
    assert_in_order(
        output,
        "PROBE MAP_SAVE_WAIT phase=before_save total_wait_s=",
        "PROBE MAPPING_DEVICE_STATUS phase=before_save",
        "PROBE HEARTBEAT phase=before_save rc=0 state=3",
        "PROBE MODE_READBACK phase=before_save requested=1 rc=0 value=1",
        "FAKE SAVE_POLL value=1",
        "PROBE MAP_SAVE_WAIT phase=save_poll total_wait_s=",
        "PROBE MAPPING_DEVICE_STATUS phase=save_poll status=stale",
        "PROBE HEARTBEAT phase=save_poll rc=0 state=3",
        "PROBE MODE_READBACK phase=save_poll requested=1 rc=0 value=1",
        "FAKE SAVE_POLL value=0",
        "PROBE MAP_SAVED",
    )
    assert "uptime_s=42.00 soc_temp_c=55" in output
    assert "cpu_pct=17,0,0,0,0,0,0,0 ram_pct=44" in output
    assert "dtof_cfg_hz=10.00 dtof_tx_hz=0.00" in output


def generation_after_old_timeout_completes(probe):
    # 模拟设备在旧 120 秒限制之后完成；新候选应直接取图，不要求操作员重试。
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.request("save-map")
    assert_map_then_raw(probe)
    output = probe.output()
    assert "MAP_SAVE_FAILED" not in output
    assert output.count("FAKE SAVE_FLAG value=1") == 1
    saved = next(line for line in output.splitlines() if line.startswith("PROBE MAP_SAVED "))
    assert 120 < float(saved.split("total_wait_s=")[1]) < 300


def save_init_failure_stops_stream(probe, *failure_markers):
    # 此时开流已成功。初始化失败必须按顺序停流并释放资源，也不能接受保存请求。
    assert probe.finish(interrupt=False) == 13
    output = probe.output()
    assert_in_order(
        output,
        "FAKE DEPTH hz=10",
        "FAKE CONFIGURE mode=SLAM",
        "PROBE MODE_READBACK phase=before_start requested=1 rc=0 value=1",
        "FAKE START mode=SLAM map_mode=1",
        "FAKE SAVE_FLAG value=0",
        *failure_markers,
        "FAKE STOP mode=SLAM saves=0 fetches=0",
        "PROBE API lidar_unregister_stream_callback rc=0",
        "PROBE API lidar_close_device rc=0",
        "PROBE API lidar_destory_device rc=0",
        "PROBE API lidar_system_deinit rc=0",
    )
    assert "PROBE MAPPING_STARTED" not in output and "FAKE SAVE_FLAG value=1" not in output


def mode_init_readback_refuses_start(probe, readback):
    assert probe.finish(interrupt=False) == 12
    output = probe.output()
    assert_in_order(
        output,
        "FAKE DEPTH hz=10",
        "FAKE CONFIGURE mode=SLAM",
        "FAKE MAP_MODE value=1",
        f"PROBE MODE_READBACK phase=before_start requested=1 {readback}",
        "PROBE MODE_INIT_FAILED reason=map_mode_not_confirmed",
    )
    assert "FAKE START" not in output and "PROBE MAPPING_STARTED" not in output


def signal_during_open_does_not_configure(probe):
    assert probe.finish(interrupt=False) == 130
    output = probe.output()
    assert_in_order(
        output,
        "FAKE INTERRUPT during=open",
        "PROBE API lidar_open_device rc=0",
        "PROBE EXIT_SIGNAL signal=2",
        "PROBE API lidar_close_device rc=0",
    )
    assert "FAKE DEPTH" not in output and "FAKE CONFIGURE" not in output and "FAKE START" not in output


def manual_save_then_stop(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    for number, suffix in ((1, ""), (2, "-2")):
        probe.request("save-map")
        probe.wait_for("PROBE MAP_SAVED", count=number)
        # 在下一次建图状态输出确认仍在 SLAM，保存指令与停止指令分别生效。
        probe.wait_for(f"save_attempts={number} maps_saved={number}")
        assert probe.process.poll() is None
        assert "FAKE STOP" not in probe.output()
        map_path = probe.directory / f"odin1-local-test{suffix}.bin"
        assert map_path.read_bytes() == f"fake .bin {number}".encode()

    probe.request("cancel-map")
    probe.wait_for("FAKE START mode=RAW")
    assert probe.finish() == 130
    output = probe.output()
    assert_in_order(
        output,
        "FAKE SAVE_FLAG value=1",
        "FAKE FETCH save=1",
        "PROBE MAP_SAVED file=odin1-local-test.bin",
        "FAKE SAVE_FLAG value=1",
        "FAKE FETCH save=2",
        "PROBE MAP_SAVED file=odin1-local-test-2.bin",
        "FAKE STOP mode=SLAM",
        "FAKE CONFIGURE mode=RAW",
        "FAKE START mode=RAW",
        "FAKE STOP mode=RAW",
    )
    assert output.count("FAKE SAVE_FLAG value=1") == 2
    assert (probe.directory / "cancel-map.accepted").is_file()
    assert not (probe.directory / "cancel-map.request").exists()


def retry_save(probe, failure, writes, fetches):
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.request("save-map")
    probe.wait_for(failure)
    probe.wait_for("PROBE MAP_SAVE_RETRY_AVAILABLE")
    assert_mapping_active(probe)
    probe.request("save-map")
    # 重新触发的路径包含五秒间隔和两次轮询，给调度留出余量。
    assert_map_then_raw(probe, timeout=10 if writes > 1 else 7)
    output = probe.output()
    assert output.count("FAKE SAVE_FLAG value=1") == writes
    assert output.count("FAKE FETCH save=1") == fetches
    assert len(list(probe.directory.glob("save-map*.accepted"))) == 2


def accepted_but_unconfirmed_save_does_not_retrigger(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.request("save-map")
    probe.wait_for("PROBE API lidar_set_custom_parameter(save_map=1) rc=-41")
    probe.wait_for("PROBE MAP_SAVE_RETRY_AVAILABLE")
    assert_mapping_active(probe)

    probe.request("save-map")
    probe.wait_for("PROBE SAVE_TRIGGER_READBACK rc=-49 value=-1")
    probe.wait_for("PROBE MAP_SAVE_RETRY_AVAILABLE", count=2)
    assert_mapping_active(probe)

    probe.request("save-map")
    assert_map_then_raw(probe)
    output = probe.output()
    assert_in_order(
        output,
        "PROBE API lidar_set_custom_parameter(save_map=1) rc=-41",
        "PROBE SAVE_TRIGGER_READBACK rc=-49 value=-1",
        "PROBE SAVE_TRIGGER_READBACK rc=0 value=1",
        "FAKE SAVE_POLL value=0",
        "FAKE FETCH save=1",
        "PROBE MAP_SAVED file=odin1-local-test-3.bin",
    )
    assert output.count("FAKE SAVE_FLAG value=1") == 1
    assert output.count("FAKE FETCH save=1") == 1


def duplicate_save_while_busy_is_coalesced(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.request("save-map")
    probe.wait_for("FAKE SAVE_POLL value=1")
    busy = probe.directory / "save-map.busy"
    assert busy.is_dir()
    # 文件接口可能被其他本地调用方直接写入，程序也须防止重复保存。
    probe.request("save-map")
    assert_map_then_raw(probe)
    output = probe.output()
    assert output.count("FAKE SAVE_FLAG value=1") == 1
    assert output.count("FAKE FETCH save=1") == 1
    assert (probe.directory / "save-map.coalesced").is_file()
    assert not busy.exists()
    assert not (probe.directory / "save-map.request").exists()


def pending_generation_stays_active(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    for count in (1, 2):
        probe.request("save-map")
        probe.wait_for("generation_timeout", count=count)
        probe.wait_for("PROBE MAP_SAVE_RETRY_AVAILABLE", count=count)
        assert_mapping_active(probe)
    output = probe.output()
    assert output.count("FAKE SAVE_FLAG value=1") == 1
    assert output.count("FAKE SAVE_POLL value=1") >= 2
    assert "FAKE FETCH" not in output and "FAKE START mode=RAW" not in output
    timeout_waits = [
        float(line.rsplit("total_wait_s=", 1)[1])
        for line in output.splitlines()
        if line.startswith("PROBE MAP_SAVE_WAIT phase=save_timeout")
    ]
    assert len(timeout_waits) == 2 and timeout_waits[0] >= 300 and timeout_waits[1] >= 600
    assert output.count("timeout_s=300 last_save_state=1") == 2
    assert output.count("PROBE HEARTBEAT phase=save_timeout rc=0 state=3") == 2
    assert output.count("PROBE MODE_READBACK phase=save_timeout requested=1 rc=0 value=1") == 2
    assert probe.finish() == 130


def queued_exit_waits_for_map(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.request("save-map")
    probe.wait_for("FAKE SAVE_POLL value=1")
    probe.request("exit")
    assert probe.finish(interrupt=False) == 0
    output = probe.output()
    assert_in_order(
        output,
        "FAKE SAVE_POLL value=1",
        "PROBE STOP_DEFERRED waiting_for=map_saved",
        "FAKE SAVE_POLL value=0",
        "FAKE FETCH save=1",
        "PROBE MAP_SAVED",
        "PROBE EXIT_REQUEST_ACCEPTED source=control_file",
        "FAKE STOP mode=SLAM",
    )
    assert "FAKE START mode=RAW" not in output and "PROBE EXIT_SIGNAL" not in output
    assert (probe.directory / "odin1-local-test.bin").read_bytes() == b"fake .bin 1"
    assert (probe.directory / "exit.accepted").is_file()
    assert not (probe.directory / "exit.request").exists()


def queued_exit_waits_after_set_failure(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.request("save-map")
    probe.wait_for("PROBE API lidar_set_custom_parameter(save_map=1) rc=-41")
    probe.wait_for("PROBE MAP_SAVE_RETRY_AVAILABLE")
    # 即使设备拒绝第一次保存指令，普通退出也应为已接受的保存请求保留重试机会。
    probe.request("exit")
    probe.wait_for("PROBE STOP_DEFERRED waiting_for=map_saved retry_save_request=1")
    assert_mapping_active(probe)
    assert (probe.directory / "exit.request").is_file()

    probe.request("save-map")
    assert probe.finish(interrupt=False) == 0
    output = probe.output()
    assert_in_order(
        output,
        "PROBE API lidar_set_custom_parameter(save_map=1) rc=-41",
        "PROBE STOP_DEFERRED waiting_for=map_saved retry_save_request=1",
        "PROBE SAVE_TRIGGER_READBACK rc=0 value=0",
        "PROBE API lidar_set_custom_parameter(save_map=1) rc=0",
        "FAKE FETCH save=1",
        "PROBE MAP_SAVED",
        "PROBE EXIT_REQUEST_ACCEPTED source=control_file",
        "FAKE STOP mode=SLAM",
    )
    assert (probe.directory / "odin1-local-test-2.bin").read_bytes() == b"fake .bin 1"
    assert (probe.directory / "exit.accepted").is_file()
    assert not (probe.directory / "exit.request").exists()
    assert "FAKE START mode=RAW" not in output and "PROBE EXIT_SIGNAL" not in output


def pending_save_defers_cancel_and_exit(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.request("save-map")
    probe.wait_for("PROBE API lidar_set_custom_parameter(save_map=1) rc=0")
    probe.request("cancel-map")
    probe.request("exit")
    # 单次生成等待超时后，两个普通控制请求都不能截断重试机会。
    probe.wait_for("generation_timeout")
    probe.wait_for("PROBE STOP_DEFERRED waiting_for=map_saved retry_save_request=1")
    assert_mapping_active(probe)
    assert (probe.directory / "cancel-map.request").is_file()
    assert (probe.directory / "exit.request").is_file()

    # 用户发出明确的中断信号仍可以结束等待，日志应保留发送者。
    probe.request("save-map")
    probe.wait_for("PROBE MAP_SAVE_REQUEST_ACCEPTED", count=2)
    assert probe.finish() == 130
    output = probe.output()
    assert_in_order(
        output,
        "PROBE MAP_SAVE_INTERRUPTED signal=2",
        f"PROBE EXIT_SIGNAL signal=2 sender_pid={os.getpid()} code=",
        "FAKE STOP mode=SLAM",
    )
    assert output.count("FAKE SAVE_FLAG value=1") == 1
    assert "FAKE FETCH" not in output and "FAKE START mode=RAW" not in output
    assert not (probe.directory / "cancel-map.accepted").exists()
    assert not (probe.directory / "exit.accepted").exists()


def explicit_cancel(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.request("cancel-map")
    probe.wait_for("FAKE START mode=RAW")
    assert probe.finish() == 130
    output = probe.output()
    assert_in_order(output, "FAKE STOP mode=SLAM", "FAKE CONFIGURE mode=RAW", "FAKE START mode=RAW")
    assert "FAKE SAVE_FLAG value=1" not in output and "FAKE FETCH" not in output


def failed_switch_stop_is_retried_only_during_cleanup(probe):
    probe.wait_for("PROBE MAPPING_STARTED")
    probe.request("cancel-map")
    assert probe.finish(interrupt=False) == 21
    output = probe.output()
    assert_in_order(
        output,
        "FAKE STOP_FAILED mode=SLAM",
        "PROBE API lidar_stop_stream(LIDAR_MODE_SLAM) rc=-48",
        "FAKE STOP mode=SLAM",
        "PROBE API lidar_stop_stream(LIDAR_MODE_SLAM) rc=0",
        "PROBE API lidar_close_device rc=0",
    )
    assert "FAKE CONFIGURE mode=RAW" not in output and "FAKE START mode=RAW" not in output


def odometry_finishes(probe):
    assert probe.finish(interrupt=False) == 0
    output = probe.output()
    assert_in_order(output, "FAKE ACTIVATE type=", "FAKE START mode=SLAM map_mode=0")
    assert "PROBE ODOMETRY_STREAMS_READY" in output
    assert output.count("FAKE ACTIVATE type=") == 5
    assert "FAKE FETCH" not in output


def run_case(binary, directory, name, test, args):
    probe = Probe(binary, directory, name, *args)
    try:
        test(probe)
        output = probe.output()
        assert "FAKE UNEXPECTED_LEGACY_SAVE_CALL" not in output
        assert output.count("FAKE STOP mode=SLAM") == output.count("FAKE START mode=SLAM")
        assert "PROBE API lidar_system_deinit rc=0" in output
        assert "PROBE BUILD sdk=v0.14.5 cleanup=native save_wait_timeout_s=300" in output
        assert "SDK_CLEANUP_COMPAT" not in output
    except (AssertionError, ValueError, subprocess.TimeoutExpired) as error:
        raise AssertionError(f"{name}: {error}\n{probe.output()}") from error
    finally:
        if probe.process.poll() is None:
            probe.process.kill()
            probe.process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-include", type=Path, default=SDK_INCLUDE)
    args = parser.parse_args()
    cases = (
        ("save-after-deadline", save_after_previous_deadline, ("10",)),
        ("default-mapping-save", save_default_session, ("--encrypted-logs",)),
        ("mapping-status", mapping_status_is_diagnostic, ()),
        ("within-new-generation-window", generation_after_old_timeout_completes, ()),
        ("init-save-write-fail", lambda probe: save_init_failure_stops_stream(
            probe, "PROBE API lidar_set_custom_parameter(save_map=0) rc=-1"), ()),
        ("init-save-read-fail", lambda probe: save_init_failure_stops_stream(
            probe, "PROBE SAVE_MAP_READBACK phase=after_start requested=0 rc=-46 value=-1",
            "PROBE MAPPING_INIT_FAILED reason=save_map_not_cleared"), ()),
        ("init-save-read-nonzero", lambda probe: save_init_failure_stops_stream(
            probe, "PROBE SAVE_MAP_READBACK phase=after_start requested=0 rc=0 value=1",
            "PROBE MAPPING_INIT_FAILED reason=save_map_not_cleared"), ()),
        ("init-mode-read-fail", lambda probe: mode_init_readback_refuses_start(probe, "rc=-47 value=-1"), ()),
        ("init-mode-read-mismatch", lambda probe: mode_init_readback_refuses_start(probe, "rc=0 value=0"), ()),
        ("signal-during-open", signal_during_open_does_not_configure, ()),
        ("manual-save-then-stop", manual_save_then_stop, ("--manual-stop",)),
        ("set-once-fail", lambda probe: retry_save(probe, "rc=-41", 2, 1), ()),
        ("set-applied-then-fail", accepted_but_unconfirmed_save_does_not_retrigger, ()),
        ("duplicate-s", duplicate_save_while_busy_is_coalesced, ()),
        ("fetch-once-fail", lambda probe: retry_save(probe, "rc=-44", 1, 2), ()),
        ("empty-once", lambda probe: retry_save(probe, "output_missing_or_empty", 1, 2), ()),
        ("late-completion", lambda probe: retry_save(probe, "generation_timeout", 1, 1), ()),
        ("pending", pending_generation_stays_active, ()),
        ("queued-exit", queued_exit_waits_for_map, ("--manual-stop",)),
        ("set-once-fail-queued-exit", queued_exit_waits_after_set_failure, ("--manual-stop",)),
        ("pending-queued-stop", pending_save_defers_cancel_and_exit, ("--manual-stop",)),
        ("explicit-cancel", explicit_cancel, ()),
        ("switch-stop-once-fail", failed_switch_stop_is_retried_only_during_cleanup, ()),
        ("odometry", odometry_finishes, ("10", "--odometry")),
    )
    with tempfile.TemporaryDirectory(prefix="odin1-mapping-test-") as temp:
        directory = Path(temp)
        binary = directory / "odin1-local-mapping-fake-sdk"
        # 链接生产源码与模拟 SDK。链接包装仅在待生成状态的用例里推进时间。
        subprocess.run(
            [
                "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic", "-pthread",
                "-I", args.sdk_include, BENCH_DIR / "odin1-local-mapping.cpp",
                BENCH_DIR / "odin1-local-mapping-support.cpp", TEST_DIR / "fake_sdk.cpp",
                "-Wl,--wrap=_ZNSt6chrono3_V212steady_clock3nowEv", "-o", binary,
            ],
            check=True,
        )
        with ThreadPoolExecutor(max_workers=len(cases)) as executor:
            jobs = [
                (name, executor.submit(run_case, binary, directory, name, test, test_args))
                for name, test, test_args in cases
            ]
            for name, job in jobs:
                job.result()
                print(f"PASS {name}", flush=True)


if __name__ == "__main__":
    main()
