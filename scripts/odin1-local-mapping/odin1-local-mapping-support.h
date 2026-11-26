#pragma once

// 本文件声明主流程用到的辅助类型。学习建图顺序时，先阅读主文件即可；
// 命令行、回调统计、资源清理和控制文件的实现位于同名 .cpp 中。
#include "lidar_api.h"

#include <array>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <mutex>
#include <string>

namespace odin_test {
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

// 为原有退出码命名，日志和脚本仍使用相同的整数值。
enum ExitCode : int {
    Success = 0, InvalidArguments = 1, SdkInitFailed = 2, DeviceNotFound = 3,
    DeviceNotIdle = 4, CreateDeviceFailed = 5, ReadVersionFailed = 6,
    EnableLogFailed = 7, ReadCalibrationFailed = 8, RegisterCallbackFailed = 9,
    OpenDeviceFailed = 10, ConfigureDeviceFailed = 11, ConfigureMapModeFailed = 12,
    ClearSaveFlagFailed = 13, ActivateStreamsFailed = 14, StartStreamFailed = 15,
    DeviceDetached = 16, SaveNeedsRetry = 18, TestIncomplete = 19,
    CleanupFailed = 20, SwitchToRawFailed = 21, Interrupted = 130
};

struct Options {
    fs::path directory;
    int seconds = 240;                    // 仅里程计对照使用；建图解析后设为 0。
    int map_mode = 1;                     // 1：设备内部建图；0：里程计对照。
    bool encrypted_logs = false;
    bool calibration_after_open = false; // 历史对照选项，默认按手册先读取标定。
    bool allow_preopened = false;
    bool activate_after_start = false;   // 仅里程计对照使用。

    bool mapping() const { return map_mode == 1; }
};

// 参数错误或目录检查失败时抛出异常，由 main 统一报告。
Options parse_options(int argc, char** argv);
void prepare_run_directory(const Options& options);
void install_stop_handlers();
extern volatile std::sig_atomic_t stop_signal;

void on_device(const lidar_device_info_t* info, bool attached);
int wait_for_device(lidar_device_info_t& device);
bool is_device_present();
bool log_api(const char* name, int rc);
void print_version(const char* label, const lidar_version_t& version);
bool read_mode(device_handle handle, const char* phase, int requested);

enum Stream { rgb, imu, dtof, odom, cloud, status, stream_count };
using Counts = std::array<unsigned long long, stream_count>;

// SDK 回调只更新观测值，建图不会拿点云或里程计数量作为保存前提。
struct Observation {
    Counts valid{};
    Counts invalid{};
    lidar_device_status_t device_status{};
    Clock::time_point status_received{};
    ros_odom_convert_complete_t latest_odom{};
    unsigned long long odom_nonincreasing = 0;
};

struct Receiver {
    std::mutex mutex;
    Observation observation;
};
void on_data(const lidar_data_t* data, void* user_data);

// Session 拥有设备资源；它必须先于回调使用的 Receiver 析构。
// 已完成的初始化步骤才会被清理；显式 close 后析构不会重复释放。
struct Session {
    device_handle handle = nullptr;
    bool system = false;
    bool callback = false;
    bool opened = false;
    int stream_mode = LIDAR_MODE_SLAM;
    bool stream_attempted = false;

    Session() = default;
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    bool start_stream(int mode, uint32_t& dtof_odr);
    bool stop_stream();
    bool close();
    ~Session();
};

bool activate_raw_streams(device_handle handle);
bool activate_odometry_streams(device_handle handle);
void print_map_save_diagnostics(device_handle handle, Receiver& receiver,
                                const char* phase, Clock::time_point save_started);

// 每两秒输出一次统计；只有里程计对照模式使用 odometry_ready 判定结果。
struct RuntimeMonitor {
    Clock::time_point started = Clock::now();
    Clock::time_point last_window = started;
    Observation previous{};
    bool queried_running = false;
    int healthy_windows = 0;
    bool odometry_ready = false;

    void update(const Options& options, const Session& session, Receiver& receiver,
                unsigned save_attempts, unsigned maps_saved);
};

// 操作窗口把 s/c/q 写成控制文件。领取请求时保留收据，便于核对人工操作。
bool has_request(const fs::path& path);
bool has_save_request(const fs::path& directory);
struct SaveRequest {
    fs::path directory;
    std::string suffix;
    std::string map_name;
};
bool claim_save_request(const fs::path& directory, unsigned& attempts, SaveRequest& request);
void finish_save_request(const SaveRequest& request);
bool accept_cancel_request(const fs::path& directory);
bool accept_exit_request(const fs::path& directory);

} // 命名空间 odin_test
