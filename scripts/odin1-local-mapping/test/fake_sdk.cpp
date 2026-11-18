// 使用厂商头文件链接模拟实现。测试进程不会加载实际 SDK 或读取 USB。
#include "lidar_api.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

namespace {
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
int mode = -1;
int map_mode = -1;
int save_flag = -1;
int activations = 0;
bool opened = false;
bool streaming = false;
bool depth_configured = false;
bool initial_mode_read = false;
bool initial_save_read = false;
bool stop_failed = false;
bool reconcile_read_failed = false;
int set_attempts = 0;
Clock::time_point previous_save_write{};
int save_calls = 0;
int save_polls = 0;
int fetch_calls = 0;
int reported_complete = 0;
bool map_fetched = false;
Clock::duration clock_offset{};
const std::string test_case = [] {
    const auto *value = std::getenv("A2_ODIN_TEST_CASE");
    return value ? value : "";
}();
lidar_data_callback_info_t receiver{};
std::thread worker;
std::atomic<bool> done{false};
int handle_storage;

template <typename T>
void emit(int type, T &payload) {
    lidar_data_t sample{};
    sample.type = type;
    sample.stream.imageCount = 1;
    sample.stream.imageList[0].pAddr = &payload;
    sample.stream.imageList[0].length = sizeof(payload);
    receiver.data_callback(&sample, receiver.user_data);
}

void odometry_samples() {
    imu_convert_data_t imu{};
    ros_odom_convert_complete_t odom{};
    lidar_device_status_t status{};
    status.imu_sensor.tx_odr = 400000;
    status.dtof_sensor.tx_odr = 10000;
    status.slam_odom_tx_odr = 10000;
    int step = 0;
    while (!done) {
        emit(LIDAR_DT_RAW_IMU, imu);
        if (step % 10 == 0) {
            status.uptime_seconds += 0.04;
            odom.timestamp_ns += 40000000;
            odom.orient[3] = 1000000;
            emit(LIDAR_DT_DEV_STATUS, status);
            emit(LIDAR_DT_SLAM_ODOMETRY, odom);
        }
        ++step;
        std::this_thread::sleep_for(4ms);
    }
}
}

// 在模拟设备返回仍在生成时推进测试进程的单调时钟，
// 使节流诊断和超时的用例无需等待完整的实际时长。
extern "C" Clock::time_point __real__ZNSt6chrono3_V212steady_clock3nowEv() noexcept;
extern "C" Clock::time_point __wrap__ZNSt6chrono3_V212steady_clock3nowEv() noexcept {
    return __real__ZNSt6chrono3_V212steady_clock3nowEv() + clock_offset;
}

int lidar_system_init(lidar_device_callback_t callback) {
    lidar_device_info_t device{};
    device.initial_state = LIDAR_DEVICE_NOT_INITIALIZED;
    std::memcpy(device.model, "fake-odin1", 10);
    callback(&device, true);
    return 0;
}
int lidar_system_deinit() {
    assert(!streaming && !opened && !worker.joinable());
    return 0;
}
int lidar_create_device(lidar_device_info_t *, device_handle *handle) {
    *handle = &handle_storage;
    return 0;
}
int lidar_destory_device(device_handle) { return 0; }
int lidar_get_version(device_handle, lidar_fireware_version_t *version) {
    *version = {};
    return 0;
}
int lidar_enable_encrypted_device_log(device_handle handle, const char *dir) {
    // 官网要求在打开设备前，对 create_device 返回的句柄设置已有日志目录。
    assert(handle == &handle_storage && !opened && std::filesystem::is_directory(dir));
    std::puts("FAKE ENCRYPTED_LOG_READY");
    return 0;
}
int lidar_get_calib_file(device_handle, const char *) { return 0; }
int lidar_register_stream_callback(device_handle, lidar_data_callback_info_t callback) {
    receiver = callback;
    return 0;
}
int lidar_unregister_stream_callback(device_handle) {
    assert(!streaming);
    receiver = {};
    return 0;
}
int lidar_open_device(device_handle) {
    opened = true;
    if (test_case == "signal-during-open") {
        std::puts("FAKE INTERRUPT during=open");
        std::raise(SIGINT);
    }
    return 0;
}
int lidar_close_device(device_handle) {
    assert(!streaming);
    opened = false;
    return 0;
}
int lidar_set_depth_parameter(device_handle, const lidar_depth_para_t *depth) {
    assert(opened && !streaming && mode == -1 && depth && depth->odr == LIDAR_DEPTH_ODR_10HZ);
    depth_configured = true;
    std::puts("FAKE DEPTH hz=10");
    return 0;
}
int lidar_set_mode(device_handle, int value) {
    assert(opened && !streaming);
    if (value == LIDAR_MODE_SLAM) assert(depth_configured);
    mode = value;
    std::printf("FAKE CONFIGURE mode=%s\n", mode == LIDAR_MODE_SLAM ? "SLAM" : "RAW");
    return 0;
}
int lidar_set_custom_parameter(device_handle, const char *key, const void *value, size_t length) {
    assert(length == sizeof(int));
    int num;
    std::memcpy(&num, value, sizeof(num));
    if (std::strcmp(key, "map_mode") == 0) {
        map_mode = num;
        std::printf("FAKE MAP_MODE value=%d\n", num);
    } else if (std::strcmp(key, "save_map") == 0) {
        // 0.14.3 固件在开流前拒绝 save_map=0。对所有用例强制执行相同顺序。
        assert(streaming && map_mode == 1 && mode == LIDAR_MODE_SLAM);
        std::printf("FAKE SAVE_FLAG value=%d\n", num);
        if (num == 0) {
            assert(save_flag == -1 && !initial_save_read);
            if (test_case == "init-save-write-fail") return -1;
        }
        // 每次写入 1 都重新开始模拟设备的制图过程。
        if (num == 1) {
            assert(initial_save_read);
            // 直接在模拟 SDK 入口核对两次写入时间，包含上次返回错误的情形。
            const auto now = Clock::now();
            if (set_attempts > 0) assert(now - previous_save_write >= 5s);
            previous_save_write = now;
            ++set_attempts;
            const bool fail_first_set = test_case == "set-once-fail" ||
                                        test_case == "set-once-fail-queued-exit";
            if (fail_first_set && set_attempts == 1) return -41;
            ++save_calls;
            save_polls = 0;
            reported_complete = 0;
            map_fetched = false;
        }
        save_flag = num;
        if (test_case == "set-applied-then-fail" && num == 1 && set_attempts == 1) return -41;
    } else assert(false);
    return 0;
}
int lidar_get_custom_parameter(device_handle, const char *key, int *value) {
    if (std::strcmp(key, "map_mode") == 0) {
        if (!streaming && test_case == "init-mode-read-fail") return -47;
        *value = !streaming && test_case == "init-mode-read-mismatch" ? 0 : map_mode;
        if (!streaming) initial_mode_read = *value == map_mode;
    } else if (std::strcmp(key, "save_map") == 0) {
        assert(streaming && mode == LIDAR_MODE_SLAM && map_mode == 1);
        if (!initial_save_read) {
            assert(save_flag == 0);
            if (test_case == "init-save-read-fail") return -46;
            *value = test_case == "init-save-read-nonzero" ? 1 : 0;
            initial_save_read = *value == 0;
            std::printf("FAKE SAVE_INIT_READBACK value=%d\n", *value);
            return 0;
        }
        // 模拟设备已收到写入，但主机没有收到确认，第一次确认查询也失败。
        if (test_case == "set-applied-then-fail" && save_flag == 1 && !reconcile_read_failed) {
            reconcile_read_failed = true;
            return -49;
        }
        // 一般在第二次轮询回写 0。挂起用例始终是 1。
        if (save_flag == 1) {
            ++save_polls;
            const bool pending = test_case == "pending" || test_case == "pending-queued-stop";
            if (pending || (test_case == "late-completion" && save_polls == 1)) {
                // 超过候选的 300 秒生成等待；只推进模拟时钟，不实际等待五分钟。
                clock_offset += 301s;
            } else if (test_case == "within-new-generation-window" && save_polls == 1) {
                // 超过旧 120 秒、仍在新 300 秒内，下一次查询完成，不能提前报超时。
                clock_offset += 121s;
            } else if (test_case == "mapping-status" && save_polls == 1) {
                clock_offset += 10s;
            } else if (save_polls >= 2) {
                save_flag = 0;
                reported_complete = save_calls;
            }
        }
        *value = save_flag;
        std::printf("FAKE SAVE_POLL value=%d\n", *value);
    } else assert(false);
    return 0;
}
int lidar_get_device_state(lidar_device_initial_state_e *value) {
    *value = streaming ? LIDAR_DEVICE_STREAMING : LIDAR_DEVICE_STREAM_STOPPED;
    return 0;
}
int lidar_activate_stream_type(device_handle, int type) {
    if (mode == LIDAR_MODE_SLAM) {
        assert(map_mode == 0);
        ++activations;
    } else {
        assert(mode == LIDAR_MODE_RAW);
        // RAW 允许采集原始流，拒绝仍启用融合里程计或 SLAM 点云。
        assert(type == LIDAR_DT_RAW_RGB || type == LIDAR_DT_RAW_IMU || type == LIDAR_DT_RAW_DTOF);
    }
    std::printf("FAKE ACTIVATE type=%d\n", type);
    return 0;
}
int lidar_start_stream(device_handle, int type, uint32_t &rate) {
    assert(opened && !streaming && type == mode);
    if (type == LIDAR_MODE_SLAM) {
        assert(initial_mode_read);
        assert(map_mode == 0 || map_mode == 1);
        assert(save_flag == -1 && !initial_save_read);
    } else assert(type == LIDAR_MODE_RAW);
    rate = 10000;
    streaming = true;
    // 建图默认不下发数据回调。诊断用例只发一份状态包，随后任其变旧，仍可保存地图。
    if (test_case == "mapping-status") {
        lidar_device_status_t report{};
        report.uptime_seconds = 42;
        report.soc_thermal.package_temp = 55;
        report.cpu_use_rate[0] = 17;
        report.ram_use_rate = 44;
        report.rgb_sensor.configured_odr = 10000;
        report.rgb_sensor.tx_odr = 2000;
        report.imu_sensor.configured_odr = 400000;
        report.imu_sensor.tx_odr = 500;
        report.dtof_sensor.configured_odr = 10000;
        report.dtof_sensor.tx_odr = 0;
        emit(LIDAR_DT_DEV_STATUS, report);
    }
    // 里程计模式单独模拟 IMU、状态和位姿输出。
    if (type == LIDAR_MODE_SLAM && map_mode == 0) worker = std::thread(odometry_samples);
    std::printf("FAKE START mode=%s map_mode=%d\n", type == LIDAR_MODE_SLAM ? "SLAM" : "RAW", map_mode);
    return 0;
}
int lidar_stop_stream(device_handle, int type) {
    assert(type == mode);
    if (type == LIDAR_MODE_SLAM && test_case == "switch-stop-once-fail" && !stop_failed) {
        stop_failed = true;
        std::puts("FAKE STOP_FAILED mode=SLAM");
        return -48;
    }
    if (type == LIDAR_MODE_SLAM && map_mode == 0) assert(activations == 5);
    if (worker.joinable()) {
        done = true;
        worker.join();
    }
    streaming = false;
    std::printf("FAKE STOP mode=%s saves=%d fetches=%d\n", type == LIDAR_MODE_SLAM ? "SLAM" : "RAW", save_calls, fetch_calls);
    return 0;
}
int lidar_get_mapping_result(device_handle, const char *dir, const char *name) {
    assert(streaming && mode == LIDAR_MODE_SLAM && map_mode == 1 && activations == 0);
    // 生成完成后才能取图。第一次传输失败或得到空文件时，允许重取同一份设备地图。
    assert(save_flag == 0 && reported_complete == save_calls && !map_fetched);
    ++fetch_calls;
    std::printf("FAKE FETCH save=%d\n", save_calls);
    if (test_case == "fetch-once-fail" && fetch_calls == 1) return -44;
    std::ofstream map(std::filesystem::path(dir) / name, std::ios::binary);
    if (test_case == "empty-once" && fetch_calls == 1) return 0;
    map << "fake .bin " << save_calls;
    map_fetched = true;
    return 0;
}
int lidar_save_map(device_handle, const char *, const char *, uint32_t) {
    // 继续链接旧接口，调用时会明确指出程序还未采用分步流程。
    std::puts("FAKE UNEXPECTED_LEGACY_SAVE_CALL");
    return -45;
}
