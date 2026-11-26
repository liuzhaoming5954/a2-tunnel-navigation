// ODIN1 内部建图测试：设备执行建图，主机负责请求保存并取回 .bin 文件。
//
// 建议阅读顺序：
//   1. main → run：程序全貌；
//   2. connect_device → start_algorithm：对照厂商初始化与开流顺序；
//   3. run_manual_mapping：操作员的 s / c / q 如何影响程序；
//   4. save_current_map：触发保存 → 等待设备完成 → 下载地图。
//
// 本文件保留建图相关 SDK 调用。回调、统计、命令行和文件收据等辅助代码
// 位于 odin1-local-mapping-support.cpp。本候选使用 v0.14.5 SDK 原生资源清理。
#include "odin1-local-mapping-support.h"

#include <cstdio>
#include <exception>
#include <thread>

namespace odin_test {
namespace {
using namespace std::chrono_literals;

// 与 v0.14.5 官方头文件的生成等待默认值一致。这是每次人工请求的等待上限，
// 不是建图时长；超时后再次按 s 继续查询同一次保存，不重新触发设备生成。
constexpr auto kSaveWaitTimeout = 300s;
constexpr auto kSavePollInterval = 1s;
constexpr auto kDiagnosticInterval = 10s;
// 同版厂商 README 要求连续保存至少间隔 5 秒，只约束新的保存触发命令。
constexpr auto kMinimumSaveInterval = 5s;

// 这是主机记住的处理阶段，与设备的 save_map 参数不是同一变量。
// 超时或下载失败后保留阶段，下一次按 s 才继续，避免重复触发设备保存。
enum class MapSaveState {
    idle,        // 尚未发送保存命令，或上一份地图已经完整取回。
    unconfirmed, // 保存命令返回错误，不能确定设备是否已执行。
    generating,  // 保存命令已被接受，或者读回 save_map=1；需要继续查询。
    ready        // 已成功读到 save_map=0；可以下载，下载失败只重试下载。
};

// 把一次建图会话的保存进度放在一起，避免在主循环维护大量零散变量。
struct MappingProgress {
    MapSaveState save_state = MapSaveState::idle;
    Clock::time_point save_started{};
    Clock::time_point next_save_allowed{}; // 首次无需等待；下发尝试后更新，回执失败也保留。
    unsigned save_attempts = 0;
    unsigned maps_saved = 0;
    bool save_attempted = false;
    bool save_incomplete = false;
    bool stop_deferred_reported = false;
};

// 第一步：初始化 SDK，等待设备，读取版本和标定，注册回调，然后打开设备。
// Session 记录每一步取得的资源；任一步失败，返回时都会按已完成的步骤清理。
int connect_device(const Options& options, Session& session, Receiver& receiver) {
    if (!log_api("lidar_system_init", lidar_system_init(on_device))) return SdkInitFailed;
    session.system = true;
    if (stop_signal) return Interrupted;

    lidar_device_info_t device{};
    const int rc = wait_for_device(device);
    if (rc != Success) return rc;

    std::printf("PROBE DEVICE_ATTACHED model=%.*s initial_state=%u cold_expected=%d\n",
                static_cast<int>(sizeof(device.model)), reinterpret_cast<const char*>(device.model),
                device.initial_state, LIDAR_DEVICE_NOT_INITIALIZED);
    // 默认只接受未初始化设备。--allow-stopped 额外允许已初始化或已停流设备，
    // 复用时跳过打开设备的调用。正在开流的设备会被拒绝。
    const bool cold = device.initial_state == LIDAR_DEVICE_NOT_INITIALIZED;
    const bool inactive = device.initial_state == LIDAR_DEVICE_STREAM_STOPPED ||
                          device.initial_state == LIDAR_DEVICE_INITIALIZED;
    if (!cold && !(options.allow_preopened && inactive)) {
        std::puts("PROBE COLD_OR_STOPPED_REQUIRED");
        return DeviceNotIdle;
    }
    if (!log_api("lidar_create_device", lidar_create_device(&device, &session.handle))) return CreateDeviceFailed;
    if (stop_signal) return Interrupted;

    // 记录内核、MCU、SoC、后台服务和 SLAM 版本，便于定位版本兼容问题。
    lidar_fireware_version_t versions{};
    if (!log_api("lidar_get_version", lidar_get_version(session.handle, &versions))) return ReadVersionFailed;
    std::printf("PROBE FIRMWARE");
    print_version("kernel", versions.kernel_version);
    print_version("mcu", versions.mcu_version);
    print_version("soc", versions.soc_version);
    print_version("daemon", versions.Daemon_proc_version);
    print_version("slam", versions.slam_version);
    std::puts("");
    if (stop_signal) return Interrupted;

    // 开设备前启用加密日志，覆盖初始化阶段。日志目录由 main() 提前创建。
    if (options.encrypted_logs) {
        // 使用 create_device 返回的句柄；device 只保存发现设备时的描述信息。
        const auto path = options.directory / "device-logs";
        if (!log_api("lidar_enable_encrypted_device_log",
                     lidar_enable_encrypted_device_log(session.handle, path.c_str()))) return EnableLogFailed;
    }
    if (stop_signal) return Interrupted;
    // 默认按厂商冷启动顺序读取标定、注册数据回调，再打开设备。
    // --calibration-after-open 将标定读取延后，用于比较调用顺序的影响。
    const auto calib_dir = options.directory / "calibration";
    if (!options.calibration_after_open &&
        !log_api("lidar_get_calib_file(before_open)", lidar_get_calib_file(session.handle, calib_dir.c_str()))) return ReadCalibrationFailed;
    if (stop_signal) return Interrupted;

    lidar_data_callback_info_t callback{on_data, &receiver};
    if (!log_api("lidar_register_stream_callback(before_open)",
                 lidar_register_stream_callback(session.handle, callback))) return RegisterCallbackFailed;
    session.callback = true;
    if (stop_signal) return Interrupted;

    if (cold) {
        if (!log_api("lidar_open_device", lidar_open_device(session.handle))) return OpenDeviceFailed;
    } else {
        std::puts("PROBE USING_ALREADY_INITIALIZED_DEVICE");
    }
    session.opened = true;
    if (stop_signal) return Interrupted;
    if (options.calibration_after_open &&
        !log_api("lidar_get_calib_file(after_open)", lidar_get_calib_file(session.handle, calib_dir.c_str()))) return ReadCalibrationFailed;
    if (stop_signal) return Interrupted;

    return Success;
}

// 第二步：选择设备端算法并开流。
// v0.14.5 建图顺序：配置深度与 SLAM → 确认 map_mode=1 → 开流 → 清零并读回 save_map。
// 写入后读回参数，避免只凭“命令发送成功”就认为模式已经正确设置。
int start_algorithm(const Options& options, Session& session) {
    const bool mapping = options.mapping();
    const int map_mode = options.map_mode;
    // 两种模式均按同版厂商配置先设定 dToF 10 Hz，使重复试验的传感器配置确定。
    lidar_depth_para_t depth{};
    depth.odr = LIDAR_DEPTH_ODR_10HZ;
    if (!log_api("lidar_set_depth_parameter(10Hz)", lidar_set_depth_parameter(session.handle, &depth))) return ConfigureDeviceFailed;
    if (stop_signal) return Interrupted;
    // 两种模式都使用 LIDAR_MODE_SLAM。map_mode=0 对应里程计，1 对应建图。
    if (!log_api("lidar_set_mode(LIDAR_MODE_SLAM)", lidar_set_mode(session.handle, LIDAR_MODE_SLAM))) return ConfigureDeviceFailed;
    if (stop_signal) return Interrupted;
    if (!log_api(mapping ? "lidar_set_custom_parameter(map_mode=1)"
                         : "lidar_set_custom_parameter(map_mode=0)",
                 lidar_set_custom_parameter(session.handle, "map_mode", &map_mode, sizeof(map_mode)))) return ConfigureMapModeFailed;
    if (stop_signal) return Interrupted;
    if (!read_mode(session.handle, "before_start", map_mode)) {
        std::puts("PROBE MODE_INIT_FAILED reason=map_mode_not_confirmed");
        return ConfigureMapModeFailed;
    }
    if (stop_signal) return Interrupted;
    // 里程计测试逐路启用输出。--activate-after-start 用于比较启用时机。
    // 建图模式沿用厂商建图调用顺序，回调只记录设备实际发出的数据。
    if (!mapping && !options.activate_after_start && !activate_odometry_streams(session.handle)) return stop_signal ? Interrupted : ActivateStreamsFailed;
    // lidar_start_stream() 发出开流命令，dtof_odr 保留 SDK 返回的原始值供对照。
    uint32_t dtof_odr = 0;
    if (!session.start_stream(LIDAR_MODE_SLAM, dtof_odr)) return stop_signal ? Interrupted : StartStreamFailed;
    if (stop_signal) return Interrupted;
    std::printf("PROBE STREAM_STARTED dtof_subframe_odr_raw=%u limit_s=%d\n", dtof_odr, options.seconds);
    if (mapping) {
        // 厂商 v0.14.5 示例在开流后初始化保存标志。先确认空闲，再开放人工保存入口。
        const int save_map = 0;
        if (!log_api("lidar_set_custom_parameter(save_map=0)",
                     lidar_set_custom_parameter(session.handle, "save_map", &save_map, sizeof(save_map)))) return ClearSaveFlagFailed;
        if (stop_signal) return Interrupted;
        int initial_value = -1;
        const int rc = lidar_get_custom_parameter(session.handle, "save_map", &initial_value);
        std::printf("PROBE SAVE_MAP_READBACK phase=after_start requested=0 rc=%d value=%d\n", rc, initial_value);
        if (rc != 0 || initial_value != 0) {
            std::puts("PROBE MAPPING_INIT_FAILED reason=save_map_not_cleared");
            return ClearSaveFlagFailed;
        }
    }
    if (!mapping && options.activate_after_start && !activate_odometry_streams(session.handle)) return stop_signal ? Interrupted : ActivateStreamsFailed;

    return Success;
}

// 保存步骤 1：只有新的人工保存，才发送 save_map=1。
// 已经在等待生成、或已经可以下载时，这个函数不重发保存命令。
int request_device_map_save(device_handle handle, MapSaveState& state,
                            Clock::time_point& next_save_allowed) {
    if (state == MapSaveState::unconfirmed) {
        int value = -1;
        const int rc = lidar_get_custom_parameter(handle, "save_map", &value);
        std::printf("PROBE SAVE_TRIGGER_READBACK rc=%d value=%d\n", rc, value);
        if (rc != 0 || (value != 0 && value != 1)) return SaveNeedsRetry;
        // 1 表示设备保存标志仍置位，不能重发触发命令。0 无法证明上次请求成功，
        // 也可能尚未执行，因此只允许本次人工重试重新触发，不能直接取旧地图报成功。
        state = value == 1 ? MapSaveState::generating : MapSaveState::idle;
    }
    if (stop_signal) return Interrupted;
    if (state == MapSaveState::idle) {
        // 仅准备下发新的 save_map=1 时等待，包括回执不明后决定重发的情况。
        // 同一份地图的继续查询和下载重试不会进入此分支。
        const auto now = Clock::now();
        if (now < next_save_allowed) {
            std::printf("PROBE MAP_SAVE_TRIGGER_DELAY wait_s=%.2f\n",
                        std::chrono::duration<double>(next_save_allowed - now).count());
        }
        while (Clock::now() < next_save_allowed) {
            if (stop_signal) return Interrupted;
            if (!is_device_present()) return DeviceDetached;
            // 每 100 毫秒检查中断和拔出，避免一次睡满剩余间隔而无法及时退出。
            std::this_thread::sleep_for(100ms);
        }
        if (stop_signal) return Interrupted;
        if (!is_device_present()) return DeviceDetached;
        const int save_map = 1;
        state = MapSaveState::unconfirmed;
        // 在调用前记录尝试时间；调用报错也可能只是回执丢失，不能立即再次触发。
        next_save_allowed = Clock::now() + kMinimumSaveInterval;
        if (!log_api("lidar_set_custom_parameter(save_map=1)",
                     lidar_set_custom_parameter(handle, "save_map", &save_map, sizeof(save_map)))) return SaveNeedsRetry;
        state = MapSaveState::generating;
    }

    return Success;
}

// 保存步骤 2：每秒读取 save_map，只有查询成功且值为 0 才允许进入下载。
// 返回码 rc=0 表示“查询成功”；读回值 value=0 才表示“设备已完成保存”。
// 若一直 value=1，排查应从此处的查询日志和设备诊断入手。
int wait_for_device_map(device_handle handle, const fs::path& dir, MapSaveState& state,
                        Receiver& receiver, Clock::time_point save_started) {
    const auto waiting_started = Clock::now();
    const auto save_deadline = waiting_started + kSaveWaitTimeout;
    auto next_diagnostics = waiting_started + kDiagnosticInterval;
    int last_save_state = -1;
    bool stop_notice_printed = false;
    while (state == MapSaveState::generating && Clock::now() < save_deadline) {
        std::this_thread::sleep_for(kSavePollInterval);
        if (stop_signal) {
            std::printf("PROBE MAP_SAVE_INTERRUPTED signal=%d\n", static_cast<int>(stop_signal));
            return Interrupted;
        }
        // 普通取消/退出请求排队到取图成功之后。保存期间在日志中明确显示等待原因。
        if (!stop_notice_printed &&
            (has_request(dir / "cancel-map.request") || has_request(dir / "exit.request"))) {
            std::puts("PROBE STOP_DEFERRED waiting_for=map_saved");
            stop_notice_printed = true;
        }
        if (!is_device_present()) {
            std::puts("PROBE DEVICE_DETACHED during=map_generation");
            return DeviceDetached;
        }
        int value = -1;
        const int rc = lidar_get_custom_parameter(handle, "save_map", &value);
        std::printf("PROBE API lidar_get_custom_parameter(save_map) rc=%d value=%d\n", rc, value);
        if (rc == 0) {
            last_save_state = value;
            if (value == 0) state = MapSaveState::ready;
        }
        const auto now = Clock::now();
        if (state == MapSaveState::generating && now >= next_diagnostics && now < save_deadline) {
            print_map_save_diagnostics(handle, receiver, "save_poll", save_started);
            next_diagnostics = now + kDiagnosticInterval;
        }
    }
    if (state != MapSaveState::ready) {
        const char* reason = last_save_state == -1 ? "save_state_unavailable" : "generation_timeout";
        print_map_save_diagnostics(handle, receiver, "save_timeout", save_started);
        std::printf("PROBE MAP_SAVE_FAILED reason=%s timeout_s=%lld last_save_state=%d\n",
                    reason, static_cast<long long>(kSaveWaitTimeout.count()), last_save_state);
        return SaveNeedsRetry;
    }
    if (stop_signal) return Interrupted;
    if (!is_device_present()) return DeviceDetached;

    return Success;
}

// 保存步骤 3：从设备下载地图，确认文件存在且非空，再放到本轮输出目录。
// 此函数的失败不会将主机保存阶段重置；下次按 s 继续下载同一份设备地图。
int download_device_map(device_handle handle, const fs::path& dir,
                        const std::string& map_name, Clock::time_point save_started) {
    // 先下载到临时子目录。SDK 返回成功且文件非空时，才将文件移到输出目录。
    const auto download_dir = dir / (map_name + ".download");
    std::error_code error;
    if (!fs::create_directory(download_dir, error)) {
        std::printf("PROBE MAP_SAVE_FAILED reason=download_dir_unavailable error=%d\n", error.value());
        return SaveNeedsRetry;
    }
    const int fetch_rc = lidar_get_mapping_result(handle, download_dir.c_str(), map_name.c_str());
    if (!log_api("lidar_get_mapping_result", fetch_rc)) return SaveNeedsRetry;
    const auto download = download_dir / map_name;
    const auto size = fs::file_size(download, error);
    if (error || size == 0) {
        std::puts("PROBE MAP_SAVE_FAILED reason=output_missing_or_empty");
        return SaveNeedsRetry;
    }
    fs::rename(download, dir / map_name, error);
    if (error) {
        std::printf("PROBE MAP_SAVE_FAILED reason=output_rename_failed error=%d\n", error.value());
        return SaveNeedsRetry;
    }
    fs::remove(download_dir, error);
    std::printf("PROBE MAP_SAVED file=%s bytes=%ju total_wait_s=%.2f\n",
                map_name.c_str(), static_cast<std::uintmax_t>(size),
                std::chrono::duration<double>(Clock::now() - save_started).count());
    return Success;
}

// 保存的完整顺序集中在这里。某步失败就返回，主循环保留会话供人工重试。
// 注意：这里没有停止建图；停止由操作员另行按 c 触发。
int save_current_map(Session& session, Receiver& receiver, const SaveRequest& request,
                     MappingProgress& progress) {
    print_map_save_diagnostics(session.handle, receiver, "before_save", progress.save_started);
    if (stop_signal) return Interrupted;
    if (!is_device_present()) return DeviceDetached;

    int rc = request_device_map_save(session.handle, progress.save_state, progress.next_save_allowed);
    if (rc != Success) return rc;
    rc = wait_for_device_map(session.handle, request.directory, progress.save_state,
                             receiver, progress.save_started);
    if (rc != Success) return rc;
    return download_device_map(session.handle, request.directory, request.map_name, progress.save_started);
}

// 领取 s 对应的请求文件，并给这一次尝试分配独立文件名和收据。
// 保存失败仍保持 save_incomplete，普通 c/q 必须等地图取回后再执行。
int handle_save_request(const Options& options, Session& session, Receiver& receiver,
                        MappingProgress& progress) {
    SaveRequest request;
    if (!claim_save_request(options.directory, progress.save_attempts, request)) return SaveNeedsRetry;
    progress.save_attempted = true;
    if (!progress.save_incomplete) progress.save_started = Clock::now();
    progress.save_incomplete = true;

    const int rc = save_current_map(session, receiver, request, progress);
    finish_save_request(request);
    if (rc == Interrupted || rc == DeviceDetached) return rc;
    if (rc != Success) {
        std::printf("PROBE MAP_SAVE_RETRY_AVAILABLE rc=%d save_request=%s\n", rc,
                    (options.directory / "save-map.request").c_str());
        std::puts("保存或取图失败，设备建图会话继续保持。再次创建 save-map.request 可重试当前步骤。");
        return rc;
    }

    ++progress.maps_saved;
    progress.save_incomplete = false;
    progress.save_state = MapSaveState::idle;
    return Success;
}

// 人工取消建图：先确认 SLAM 停流成功，再配置并启动 RAW 传感器输出。
// 停流失败时不切模式；Session 的最终清理仍会有界重试一次停流。
int stop_mapping(Session& session, unsigned maps_saved) {
    uint32_t raw_odr = 0;
    if (!session.stop_stream()) return stop_signal ? Interrupted : SwitchToRawFailed;
    if (stop_signal) return Interrupted;
    if (!log_api("lidar_set_mode(LIDAR_MODE_RAW)", lidar_set_mode(session.handle, LIDAR_MODE_RAW)) ||
        !activate_raw_streams(session.handle) || !session.start_stream(LIDAR_MODE_RAW, raw_odr)) {
        return stop_signal ? Interrupted : SwitchToRawFailed;
    }
    std::puts("PROBE MAPPING_CANCELLED mode=raw");
    std::printf("PROBE MAPPING_FINISHED map_saved=%d maps_saved=%u\n", maps_saved > 0, maps_saved);
    std::puts("设备已切换为 RAW 传感器模式，可按 Ctrl+C 关闭连接并退出程序。");
    return Success;
}

// 第三步：保持会话，等待人工指令。操作窗口把 s/c/q 写成下列请求文件。
// 保存期间此循环暂时进入 save_current_map；保存失败返回后仍等待下一次人工操作。
int run_manual_mapping(const Options& options, Session& session, Receiver& receiver,
                        RuntimeMonitor& monitor, MappingProgress& progress) {
    const auto& dir = options.directory;
    const auto cancel_request = dir / "cancel-map.request";
    const auto exit_request = dir / "exit.request";
    std::printf("PROBE MAPPING_STARTED save_request=%s\n", (dir / "save-map.request").c_str());
    std::printf("PROBE MAPPING_CANCEL_PATH request=%s\n", cancel_request.c_str());
    std::puts("PROBE MAPPING_POLICY device_internal=1 wait_for_manual_save=1 cancel_after_fetch=0");
    std::puts("设备内部建图已启动。采集结束并停稳后创建 save-map.request，等待地图取回。");
    std::puts("本轮由人工取消建图，取图后创建 cancel-map.request 切换到 RAW 模式。");

    while (!stop_signal) {
        if (!is_device_present()) {
            std::puts("PROBE DEVICE_DETACHED");
            return DeviceDetached;
        }
        monitor.update(options, session, receiver, progress.save_attempts, progress.maps_saved);

        // s：保存当前地图。取消建图并切到 RAW 后，不再接收保存请求。
        if (session.stream_mode == LIDAR_MODE_SLAM && has_save_request(dir)) {
            const int rc = handle_save_request(options, session, receiver, progress);
            if (rc == Interrupted || rc == DeviceDetached) return rc;
            if (rc != Success) continue;
        }

        const bool cancel_requested = has_request(cancel_request);
        const bool exit_requested = has_request(exit_request);
        // 保存已开始但还没取回：c/q 排队，s 可重试；Ctrl+C/SIGTERM 可明确中断。
        if (progress.save_incomplete && (cancel_requested || exit_requested)) {
            if (!progress.stop_deferred_reported) {
                std::puts("PROBE STOP_DEFERRED waiting_for=map_saved retry_save_request=1");
                progress.stop_deferred_reported = true;
            }
            std::this_thread::sleep_for(100ms);
            continue;
        }

        // q：结束循环，由 run 统一停流并关闭连接。
        if (exit_requested && accept_exit_request(dir)) break;

        // c：领取人工取消请求，再停止设备内部建图；程序继续运行并观察 RAW 输出。
        if (!stop_signal && session.stream_mode == LIDAR_MODE_SLAM && cancel_requested) {
            if (!accept_cancel_request(dir)) continue;
            const int rc = stop_mapping(session, progress.maps_saved);
            if (rc != Success) return rc;
        }
        std::this_thread::sleep_for(100ms);
    }
    return Success;
}

// 里程计对照是独立的辅助路径，只在显式传入 --odometry 时使用。
// 它有运行时限及数据健康判据；内部建图流程不依赖这些条件。
int run_odometry_check(const Options& options, Session& session, Receiver& receiver,
                       RuntimeMonitor& monitor) {
    const auto deadline = monitor.started + std::chrono::seconds(options.seconds);
    while (!stop_signal && Clock::now() < deadline) {
        if (!is_device_present()) {
            std::puts("PROBE DEVICE_DETACHED");
            return DeviceDetached;
        }
        monitor.update(options, session, receiver, 0, 0);
        if (has_request(options.directory / "exit.request") && accept_exit_request(options.directory)) break;
        std::this_thread::sleep_for(100ms);
    }
    return Success;
}

// 总流程：连接 → 开流 → 人工操作（或里程计对照）→ 清理资源。
// C++ 局部变量按声明的逆序析构，因此 Receiver 必须写在 Session 前面。
int run(const Options& options) {
    std::printf("PROBE BUILD sdk=v0.14.5 cleanup=native save_wait_timeout_s=%lld\n",
                static_cast<long long>(kSaveWaitTimeout.count()));
    Receiver receiver;
    Session session;
    int rc = connect_device(options, session, receiver);
    if (rc != Success) return rc;
    rc = start_algorithm(options, session);
    if (rc != Success) return rc;

    RuntimeMonitor monitor;
    MappingProgress progress;
    if (options.mapping()) {
        rc = run_manual_mapping(options, session, receiver, monitor, progress);
    } else {
        rc = run_odometry_check(options, session, receiver, monitor);
    }
    if (rc != Success) return rc;

    // 建图以非空地图取回为完成条件；里程计对照以连续健康窗口为完成条件。
    const bool completed = options.mapping() ? progress.maps_saved > 0 : monitor.odometry_ready;
    std::printf("PROBE END ready=%d save_attempted=%d signal=%d map_saved=%d maps_saved=%u\n",
                completed, progress.save_attempted, static_cast<int>(stop_signal),
                progress.maps_saved > 0, progress.maps_saved);
    const bool cleanup_ok = session.close();
    if (stop_signal) return Interrupted;
    if (!cleanup_ok) return CleanupFailed;
    return completed ? Success : TestIncomplete;
}
} // 匿名命名空间
} // 命名空间 odin_test

// 程序入口：参数、目录及信号准备好后，进入上面的总流程。
int main(int argc, char** argv) {
    using namespace odin_test;
    std::setvbuf(stdout, nullptr, _IOLBF, 0); // 日志按行刷新，终端实时显示。
    try {
        const Options options = parse_options(argc, argv);
        prepare_run_directory(options);
        install_stop_handlers();
        return run(options);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "PROBE ERROR %s\n", error.what());
        return InvalidArguments;
    }
}
