// ODIN1 本地联调的辅助实现。
// 主文件负责“连接、建图、保存、停止”的业务顺序；本文件集中保存这些步骤依赖的
// 参数解析、回调统计、资源清理和控制文件操作。拆分只改变代码位置，不改变设备协议。
#include "odin1-local-mapping-support.h"

#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>

namespace odin_test {
using namespace std::chrono_literals;

// 主线程轮询这个标记；信号处理器不直接调用 SDK，也不操作互斥锁。
volatile std::sig_atomic_t stop_signal = 0;

namespace {
// 热插拔状态由 SDK 回调和主线程共享，所有普通变量都由 device_mutex 保护。
volatile std::sig_atomic_t stop_sender_pid = 0;
volatile std::sig_atomic_t stop_signal_code = 0;
std::mutex device_mutex;
std::condition_variable device_changed;
lidar_device_info_t device_info{};
bool have_device = false;
bool device_present = false;

// 只保存信号来源，实际停流和资源释放仍由主线程的 Session 完成。
void request_stop(int signal, siginfo_t* info, void*) {
    if (info) {
        stop_sender_pid = info->si_pid;
        stop_signal_code = info->si_code;
    }
    stop_signal = signal;
}

// 主线程拿到观测副本后即可释放锁；日志打印和计算不会长时间阻塞 SDK 回调。
Observation read_observation(Receiver& receiver) {
    std::lock_guard<std::mutex> lock(receiver.mutex);
    return receiver.observation;
}

// SDK 状态包的频率单位为毫赫兹，界面统一显示为赫兹。
double hertz(int odr) { return static_cast<double>(odr) / 1000.0; }

// 并列展示主机接收频率与设备自报发送频率，供里程计对照和 RAW 阶段诊断。
void print_stats(const Observation& latest, const Observation& previous,
                 Clock::time_point now, double elapsed, double window) {
    constexpr std::array<const char*, stream_count> names{"rgb", "imu", "dtof", "odom", "cloud", "status"};
    std::printf("PROBE WINDOW elapsed_s=%.2f window_s=%.2f", elapsed, window);
    // 两次累计有效帧数之差除以实际窗口时长得到接收频率，无效包仍显示累计值。
    for (std::size_t i = 0; i < names.size(); ++i) {
        std::printf(" %s_n=%llu %s_hz=%.2f %s_invalid=%llu", names[i], latest.valid[i],
                    names[i], static_cast<double>(latest.valid[i] - previous.valid[i]) / window,
                    names[i], latest.invalid[i]);
    }
    std::printf(" odom_nonincreasing=%llu\n", latest.odom_nonincreasing);

    if (latest.valid[status] > 0) {
        const auto& d = latest.device_status;
        const double status_age = std::chrono::duration<double>(now - latest.status_received).count();
        std::printf("PROBE DEVICE uptime_s=%.2f status_age_s=%.2f rgb_cfg_hz=%.2f rgb_tx_hz=%.2f "
                    "imu_cfg_hz=%.2f imu_tx_hz=%.2f dtof_cfg_hz=%.2f dtof_tx_hz=%.2f "
                    "odom_tx_hz=%.2f cloud_tx_hz=%.2f odom_hf_tx_hz=%.2f\n",
                    d.uptime_seconds, status_age, hertz(d.rgb_sensor.configured_odr), hertz(d.rgb_sensor.tx_odr),
                    hertz(d.imu_sensor.configured_odr), hertz(d.imu_sensor.tx_odr),
                    hertz(d.dtof_sensor.configured_odr), hertz(d.dtof_sensor.tx_odr),
                    hertz(d.slam_odom_tx_odr), hertz(d.slam_cloud_tx_odr), hertz(d.slam_odom_highfreq_tx_odr));
    }
    // 有新里程计时显示最新位姿。位置从微米转成米，四元数按 x、y、z、w 顺序还原，
    // 然后计算以度为单位的航向角。
    if (latest.valid[odom] > previous.valid[odom]) {
        const auto& pose = latest.latest_odom;
        const auto scaled = [](int64_t value) { return static_cast<double>(value) / 1e6; };
        const double x = scaled(pose.orient[0]), y = scaled(pose.orient[1]);
        const double z = scaled(pose.orient[2]), w = scaled(pose.orient[3]);
        const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * (180.0 / std::acos(-1.0));
        std::printf("PROBE POSE timestamp_ns=%llu x_m=%.5f y_m=%.5f z_m=%.5f yaw_deg=%.3f\n",
                    static_cast<unsigned long long>(pose.timestamp_ns),
                    scaled(pose.pos[0]), scaled(pose.pos[1]), scaled(pose.pos[2]), yaw);
    }
}

// 只有里程计对照模式使用这些条件，设备内部建图不依赖主机收到点云或里程计。
bool odometry_tx_healthy(const Observation& observation, Clock::time_point now) {
    const auto& d = observation.device_status;
    return observation.valid[status] > 0 && now - observation.status_received < 5s &&
           d.imu_sensor.tx_odr >= 100000 && d.dtof_sensor.tx_odr >= 3000 &&
           d.slam_odom_tx_odr > 0;
}

// 心跳只是诊断信息，保留厂商状态枚举的原始数值。
void read_state(const char* when) {
    auto state = LIDAR_DEVICE_NONE;
    const int rc = lidar_get_device_state(&state);
    std::printf("PROBE HEARTBEAT phase=%s rc=%d state=%d\n", when, rc, static_cast<int>(state));
}

// 系统资源和传感器状态用于解释设备行为，缺包或低发送率不改变建图保存流程。
void print_mapping_device_status(const Observation& observed, const char* phase, Clock::time_point now) {
    if (observed.valid[status] == 0) {
        std::printf("PROBE MAPPING_DEVICE_STATUS phase=%s status=unavailable status_invalid=%llu\n",
                    phase, observed.invalid[status]);
        return;
    }
    const auto age = now - observed.status_received;
    const auto& d = observed.device_status;
    const auto& cpu = d.cpu_use_rate;
    std::printf("PROBE MAPPING_DEVICE_STATUS phase=%s status=%s status_age_s=%.2f "
                "uptime_s=%.2f soc_temp_c=%d cpu_temp_c=%d center_temp_c=%d gpu_temp_c=%d npu_temp_c=%d "
                "dtof_tx_temp_c=%d dtof_rx_temp_c=%d cpu_pct=%d,%d,%d,%d,%d,%d,%d,%d ram_pct=%d "
                "rgb_cfg_hz=%.2f rgb_tx_hz=%.2f imu_cfg_hz=%.2f imu_tx_hz=%.2f "
                "dtof_cfg_hz=%.2f dtof_tx_hz=%.2f status_invalid=%llu\n",
                phase, age < 5s ? "fresh" : "stale", std::chrono::duration<double>(age).count(),
                d.uptime_seconds, d.soc_thermal.package_temp, d.soc_thermal.cpu_temp,
                d.soc_thermal.center_temp, d.soc_thermal.gpu_temp, d.soc_thermal.npu_temp,
                d.dtof_sensor.tx_temp, d.dtof_sensor.rx_temp,
                cpu[0], cpu[1], cpu[2], cpu[3], cpu[4], cpu[5], cpu[6], cpu[7], d.ram_use_rate,
                hertz(d.rgb_sensor.configured_odr), hertz(d.rgb_sensor.tx_odr),
                hertz(d.imu_sensor.configured_odr), hertz(d.imu_sensor.tx_odr),
                hertz(d.dtof_sensor.configured_odr), hertz(d.dtof_sensor.tx_odr), observed.invalid[status]);
}

// 流启用顺序与原程序一致：先 RGB、IMU、dToF，再按需启用里程计和 SLAM 点云。
const std::array<std::pair<int, const char*>, 3> raw_streams{{
    {LIDAR_DT_RAW_RGB, "lidar_activate_stream_type(LIDAR_DT_RAW_RGB)"},
    {LIDAR_DT_RAW_IMU, "lidar_activate_stream_type(LIDAR_DT_RAW_IMU)"},
    {LIDAR_DT_RAW_DTOF, "lidar_activate_stream_type(LIDAR_DT_RAW_DTOF)"},
}};
const std::array<std::pair<int, const char*>, 2> slam_streams{{
    {LIDAR_DT_SLAM_ODOMETRY, "lidar_activate_stream_type(LIDAR_DT_SLAM_ODOMETRY)"},
    {LIDAR_DT_SLAM_CLOUD, "lidar_activate_stream_type(LIDAR_DT_SLAM_CLOUD)"},
}};

template <std::size_t Count>
bool activate_streams(device_handle handle, const std::array<std::pair<int, const char*>, Count>& streams) {
    for (const auto& [type, name] : streams) {
        if (stop_signal || !log_api(name, lidar_activate_stream_type(handle, type))) return false;
    }
    return true;
}
} // 匿名命名空间

// 参数和目录准备在 SDK 初始化之前完成，输入错误不会触发设备操作。
Options parse_options(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s RUN_DIR [seconds (odometry only)] [--manual-stop] [--encrypted-logs] [--calibration-after-open] [--odometry] [--allow-stopped] [--activate-after-start]\n", argv[0]);
        throw std::invalid_argument("RUN_DIR is required");
    }
    Options options;
    options.directory = fs::absolute(argv[1]);
    bool duration_set = false;
    for (int arg = 2; arg < argc; ++arg) {
        const std::string value = argv[arg];
        if (value == "--encrypted-logs") {
            options.encrypted_logs = true;
        } else if (value == "--manual-stop") {
            // 保留旧启动脚本兼容性。所有建图会话均由人工分别触发保存和停止。
        } else if (value == "--calibration-after-open") {
            options.calibration_after_open = true;
        } else if (value == "--odometry") {
            options.map_mode = 0;
        } else if (value == "--allow-stopped") {
            options.allow_preopened = true;
        } else if (value == "--activate-after-start") {
            options.activate_after_start = true;
        } else if (!duration_set) {
            std::size_t used = 0;
            options.seconds = std::stoi(value, &used);
            if (used != value.size() || options.seconds < 10 || options.seconds > 900) {
                throw std::invalid_argument("seconds must be 10-900");
            }
            duration_set = true;
        } else {
            throw std::invalid_argument("unknown command-line argument");
        }
    }
    if (options.mapping()) {
        // 旧脚本传入的建图时长只提示忽略；建图始终等待人工保存和停止。
        if (duration_set) std::puts("PROBE MAPPING_DURATION_IGNORED wait_for_manual_save=1");
        options.seconds = 0;
    }
    return options;
}

void prepare_run_directory(const Options& options) {
    const auto& dir = options.directory;
    if (!fs::is_directory(dir)) throw std::invalid_argument("RUN_DIR must already exist");
    if (fs::exists(dir / "save-map.request") || fs::exists(dir / "cancel-map.request") ||
        fs::exists(dir / "exit.request") || fs::exists(dir / "save-map.busy")) {
        throw std::invalid_argument("RUN_DIR contains stale control requests or save-map.busy; use a fresh directory");
    }
    fs::create_directory(dir / "calibration");
    if (options.encrypted_logs) fs::create_directory(dir / "device-logs");
    std::printf("PROBE PLAN calibration=%s callback=before_open activation=%s map_mode=%d encrypted_logs=%d\n",
                options.calibration_after_open ? "after_open_AB" : "before_open_official",
                options.mapping() ? "device_mapping" : (options.activate_after_start ? "after_start" : "before_start"),
                options.map_mode, options.encrypted_logs);
}

void install_stop_handlers() {
    struct sigaction action{};
    action.sa_sigaction = request_stop;
    action.sa_flags = SA_SIGINFO;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, nullptr) != 0 || sigaction(SIGTERM, &action, nullptr) != 0) {
        throw std::runtime_error("failed to install stop signal handlers");
    }
}

// 选中第一个接入的设备，后续按序列号跟踪同一设备的插拔。
// 条件变量通知主线程重新检查状态；离开锁的作用域后再通知，减少无效竞争。
void on_device(const lidar_device_info_t* info, bool attached) {
    if (!info) return;
    {
        std::lock_guard<std::mutex> lock(device_mutex);
        if (!have_device && attached) {
            device_info = *info;
            have_device = true;
        }
        if (have_device && std::memcmp(info->serial, device_info.serial, sizeof(info->serial)) == 0) {
            device_present = attached;
        }
    }
    device_changed.notify_all();
}

// 最多等待 15 秒，每 200 毫秒同时检查一次停止标记。
// 条件变量等待期间会暂时释放互斥锁，允许热插拔回调写入设备信息。
int wait_for_device(lidar_device_info_t& device) {
    const auto deadline = Clock::now() + 15s;
    std::unique_lock<std::mutex> lock(device_mutex);
    while ((!have_device || !device_present) && !stop_signal && Clock::now() < deadline) {
        device_changed.wait_for(lock, 200ms);
    }
    if (stop_signal) return Interrupted;
    if (!have_device || !device_present) {
        std::puts("PROBE NO_DEVICE");
        return DeviceNotFound;
    }
    device = device_info;
    return Success;
}

bool is_device_present() {
    std::lock_guard<std::mutex> lock(device_mutex);
    return device_present;
}

// 日志保留 SDK 原始返回码，布尔结果仅用于决定当前步骤是否成功。
bool log_api(const char* name, int rc) {
    std::printf("PROBE API %s rc=%d\n", name, rc);
    return rc == 0;
}

void print_version(const char* label, const lidar_version_t& v) {
    std::printf(" %s=%d.%d.%d", label, v.major, v.minor, v.patch);
}

// 开流前调用者检查返回值；诊断阶段仍只记录，不改变运行状态。
bool read_mode(device_handle handle, const char* when, int requested) {
    int actual = -1;
    const int rc = lidar_get_custom_parameter(handle, "map_mode", &actual);
    std::printf("PROBE MODE_READBACK phase=%s requested=%d rc=%d value=%d\n", when, requested, rc, actual);
    return rc == 0 && actual == requested;
}

// 回调从 SDK 借用载荷，不能在回调结束后继续持有 payload.pAddr。
// 因此在 Receiver 的锁内更新计数，并复制需要保留的状态包与位姿。
void on_data(const lidar_data_t* data, void* user_data) {
    if (!data || !user_data) return;
    Stream stream;
    switch (data->type) {
        case LIDAR_DT_RAW_RGB: stream = rgb; break;
        case LIDAR_DT_RAW_IMU: stream = imu; break;
        case LIDAR_DT_RAW_DTOF: stream = dtof; break;
        case LIDAR_DT_SLAM_ODOMETRY: stream = odom; break;
        case LIDAR_DT_SLAM_CLOUD: stream = cloud; break;
        case LIDAR_DT_DEV_STATUS: stream = status; break;
        default: return;
    }

    // dToF 的 XYZ 点云位于 SDK 数据槽 1，槽 0 是深度图。其余被统计的流读取槽 0。
    // 所有载荷均检查槽位、指针和长度。IMU 与里程计要求大小匹配结构体，
    // 状态包允许包含额外字段。RGB 和点云只做基本的载荷检查。
    const unsigned index = stream == dtof ? 1 : 0;
    const auto& payload = data->stream.imageList[index];
    bool valid = data->stream.imageCount > index && payload.pAddr != nullptr && payload.length > 0;
    if (stream == imu) valid = valid && payload.length == sizeof(imu_convert_data_t);
    if (stream == odom) valid = valid && payload.length == sizeof(ros_odom_convert_complete_t);
    if (stream == status) valid = valid && payload.length >= sizeof(lidar_device_status_t);

    auto& receiver = *static_cast<Receiver*>(user_data);
    std::lock_guard<std::mutex> lock(receiver.mutex);
    auto& observation = receiver.observation;
    if (!valid) {
        ++observation.invalid[stream];
        return;
    }
    ++observation.valid[stream];

    // 回调内复制状态包和位姿，后续统计使用本地副本。状态时间取主机收到包的时刻。
    if (stream == status) {
        std::memcpy(&observation.device_status, payload.pAddr, sizeof(observation.device_status));
        observation.status_received = Clock::now();
    } else if (stream == odom) {
        ros_odom_convert_complete_t pose;
        std::memcpy(&pose, payload.pAddr, sizeof(pose));
        // 时间戳重复或倒退单独累加。该帧仍计入有效载荷数，也会更新最新位姿。
        if (observation.valid[odom] > 1 && pose.timestamp_ns <= observation.latest_odom.timestamp_ns) {
            ++observation.odom_nonincreasing;
        }
        observation.latest_odom = pose;
    }
}

// Session 负责资源生存期；方法移到类外定义后，调用顺序与原程序保持一致。
// 调用者先声明 Receiver、后声明 Session，退出作用域时先清理 SDK，再销毁回调数据。
// 开流调用即使报错，也可能已改变设备状态，因此先记下“尝试过”，供收尾停流。
bool Session::start_stream(int mode, uint32_t& dtof_odr) {
    if (stop_signal) return false;
    // 开流失败时，收尾仍尝试停掉本次请求的流。
    stream_mode = mode;
    stream_attempted = true;
    const char* name = mode == LIDAR_MODE_RAW ? "lidar_start_stream(LIDAR_MODE_RAW)"
                                            : "lidar_start_stream(LIDAR_MODE_SLAM)";
    return log_api(name, lidar_start_stream(handle, mode, dtof_odr));
}

// 只有 SDK 明确返回成功才清除开流标记，失败时允许最终收尾再尝试一次。
bool Session::stop_stream() {
    if (!stream_attempted) return true;
    const char* name = stream_mode == LIDAR_MODE_RAW ? "lidar_stop_stream(LIDAR_MODE_RAW)"
                                                   : "lidar_stop_stream(LIDAR_MODE_SLAM)";
    const bool stopped = log_api(name, lidar_stop_stream(handle, stream_mode));
    // 显式取消时若停流失败，仍视为可能开流；close() 必须再做一次有界停流清理。
    // 只有确认停流成功，才允许调用者切换模式。
    if (stopped) stream_attempted = false;
    return stopped;
}

// 按停流、注销回调、关闭设备、销毁句柄、反初始化的顺序清理。
// 单步失败仍继续后面的步骤；清除资源标记使显式 close 后的析构成为空操作。
bool Session::close() {
    if (system && stop_signal) {
        std::printf("PROBE EXIT_SIGNAL signal=%d sender_pid=%d code=%d\n",
                    static_cast<int>(stop_signal), static_cast<int>(stop_sender_pid),
                    static_cast<int>(stop_signal_code));
    }
    bool ok = stop_stream();
    // 最终清理只尝试一次停流，后续仍注销/关闭/反初始化；析构不再复用已销毁句柄。
    stream_attempted = false;
    if (callback) {
        callback = false;
        ok = log_api("lidar_unregister_stream_callback", lidar_unregister_stream_callback(handle)) && ok;
    }
    if (opened) {
        opened = false;
        ok = log_api("lidar_close_device", lidar_close_device(handle)) && ok;
    }
    if (handle) {
        ok = log_api("lidar_destory_device", lidar_destory_device(handle)) && ok;
        handle = nullptr;
    }
    if (system) {
        system = false;
        // v0.14.5 候选直接使用厂商清理入口，不继承旧版静态库专用的释放包装。
        ok = log_api("lidar_system_deinit", lidar_system_deinit()) && ok;
    }
    return ok;
}

// 即使调用方因初始化失败而提前 return，离开作用域仍会执行这次清理。
Session::~Session() { close(); }

bool activate_raw_streams(device_handle handle) {
    return activate_streams(handle, raw_streams);
}

bool activate_odometry_streams(device_handle handle) {
    return activate_raw_streams(handle) && activate_streams(handle, slam_streams);
}

// 保存时间从第一次领取请求开始计算，人工重试不会重置累计等待时间。
void print_map_save_diagnostics(device_handle handle, Receiver& receiver, const char* phase,
                                Clock::time_point save_started) {
    const auto observed = read_observation(receiver);
    const auto now = Clock::now();
    std::printf("PROBE MAP_SAVE_WAIT phase=%s total_wait_s=%.2f\n",
                phase, std::chrono::duration<double>(now - save_started).count());
    print_mapping_device_status(observed, phase, now);
    read_state(phase);
    read_mode(handle, phase, 1);
}

// 运行监测独立于建图业务状态。每次调用只在至少两秒过去后更新一份观测窗口。
void RuntimeMonitor::update(const Options& options, const Session& session, Receiver& receiver,
                            unsigned save_attempts, unsigned maps_saved) {
    const bool mapping = options.mapping();
    const auto now = Clock::now();
    if (now - last_window >= 2s) {
        const auto observed = read_observation(receiver);
        const double window = std::chrono::duration<double>(now - last_window).count();
        if (mapping && session.stream_mode == LIDAR_MODE_SLAM) {
            // 内部建图只显示会话和保存状态，主机点云/里程计回调频率不参与判定。
            std::printf("PROBE MAPPING_ACTIVE elapsed_s=%.2f save_attempts=%u maps_saved=%u\n",
                        std::chrono::duration<double>(now - started).count(), save_attempts, maps_saved);
        } else {
            print_stats(observed, previous, now, std::chrono::duration<double>(now - started).count(), window);
        }
        if (!mapping && !odometry_ready) {
            // 里程计测试要求主机收到至少 100 Hz 的 IMU 和新的融合里程计，
            // 时间戳异常次数未增加，设备自报的发送率也须达标。
            const bool receiving = observed.valid[imu] - previous.valid[imu] >= 100 * window &&
                                   observed.valid[odom] > previous.valid[odom] &&
                                   observed.odom_nonincreasing == previous.odom_nonincreasing;
            healthy_windows = receiving && odometry_tx_healthy(observed, now) ? healthy_windows + 1 : 0;
            if (healthy_windows >= 3) {
                odometry_ready = true;
                std::puts("PROBE ODOMETRY_STREAMS_READY window_count=3");
            }
        }
        // 开流约 4 秒后查询一次设备状态、读回一次算法模式，供日志对照。
        if (!queried_running && now - started >= 4s) {
            queried_running = true;
            read_state("after_start");
            if (session.stream_mode == LIDAR_MODE_SLAM) {
                read_mode(session.handle, "after_start", options.map_mode);
                if (mapping) print_mapping_device_status(observed, "after_start", Clock::now());
            }
        }
        previous = observed;
        last_window = now;
    }
}

// 控制文件只接受普通文件。状态查询错误按“没有可领取请求”处理，
// 避免目录暂时不可访问时从诊断分支抛出异常、意外结束正在保存的会话。
bool has_request(const fs::path& path) {
    std::error_code error;
    return fs::is_regular_file(fs::symlink_status(path, error));
}

bool has_save_request(const fs::path& directory) {
    std::error_code error;
    const bool busy = fs::exists(directory / "save-map.busy", error);
    return !error && !busy && has_request(directory / "save-map.request");
}

// 领取保存请求后保留 accepted 收据，并用 busy 目录阻止重复按键排成新的保存。
// 失败时的等待时间与原循环一致，返回 false 后主流程直接进入下一轮检查。
bool claim_save_request(const fs::path& directory, unsigned& attempts, SaveRequest& request) {
    ++attempts;
    request.directory = directory;
    request.suffix = attempts == 1 ? "" : "-" + std::to_string(attempts);
    request.map_name = "odin1-local-test" + request.suffix + ".bin";
    const auto accepted = directory / ("save-map" + request.suffix + ".accepted");
    std::error_code error;
    const bool map_exists = fs::exists(directory / request.map_name, error);
    if (error || map_exists) {
        std::puts("PROBE MAP_SAVE_REFUSED reason=existing_output_or_request_receipt");
        std::this_thread::sleep_for(100ms);
        return false;
    }
    const bool receipt_exists = fs::exists(accepted, error);
    if (error || receipt_exists) {
        std::puts("PROBE MAP_SAVE_REFUSED reason=existing_output_or_request_receipt");
        std::this_thread::sleep_for(100ms);
        return false;
    }
    // 先创建忙碌标记，再重命名请求。创建目录是原子操作。
    const auto busy = directory / "save-map.busy";
    if (!fs::create_directory(busy, error)) {
        std::printf("PROBE MAP_SAVE_REFUSED reason=busy_marker_unavailable error=%d\n", error.value());
        std::this_thread::sleep_for(100ms);
        return false;
    }
    fs::rename(directory / "save-map.request", accepted, error);
    if (error) {
        std::printf("PROBE MAP_SAVE_REFUSED reason=request_rename_failed error=%d\n", error.value());
        fs::remove(busy, error);
        std::this_thread::sleep_for(1s);
        return false;
    }
    std::puts("PROBE MAP_SAVE_REQUEST_ACCEPTED");
    return true;
}

// 保存期间若出现越过控制窗口检查的重复 s，只保留 coalesced 收据。
// 合并失败就保留 busy，阻止未处理的重复请求在保存结束后触发另一份地图。
void finish_save_request(const SaveRequest& request) {
    const auto pending = request.directory / "save-map.request";
    std::error_code error;
    bool requests_coalesced = true;
    if (has_request(pending)) {
        fs::rename(pending, request.directory / ("save-map" + request.suffix + ".coalesced"), error);
        requests_coalesced = !error;
        std::printf("PROBE MAP_SAVE_REQUEST_COALESCED ok=%d error=%d\n", requests_coalesced, error.value());
    }
    if (requests_coalesced) {
        fs::remove(request.directory / "save-map.busy", error);
        if (error) std::printf("PROBE MAP_SAVE_CONTROL_ERROR action=remove_busy error=%d\n", error.value());
    }
}

// 主流程确认允许取消后才领取请求；这里只处理控制文件，不发送停止指令。
// 未发起保存时也允许人工放弃本轮采集，是否等待保存由主流程判断。
bool accept_cancel_request(const fs::path& directory) {
    const auto accepted = directory / "cancel-map.accepted";
    std::error_code error;
    const bool receipt_exists = fs::exists(accepted, error);
    if (error || receipt_exists) {
        std::printf("PROBE MAPPING_CANCEL_FAILED reason=request_receipt_unavailable error=%d\n", error.value());
        std::this_thread::sleep_for(1s);
        return false;
    }
    fs::rename(directory / "cancel-map.request", accepted, error);
    if (error) {
        std::printf("PROBE MAPPING_CANCEL_FAILED reason=request_rename_failed error=%d\n", error.value());
        std::this_thread::sleep_for(1s);
        return false;
    }
    return true;
}

// 正常退出只领取请求并打印凭据，随后由主流程离开循环、调用 Session 清理。
bool accept_exit_request(const fs::path& directory) {
    std::error_code error;
    fs::rename(directory / "exit.request", directory / "exit.accepted", error);
    if (error) {
        std::printf("PROBE EXIT_REQUEST_FAILED error=%d\n", error.value());
        return false;
    }
    std::puts("PROBE EXIT_REQUEST_ACCEPTED source=control_file");
    return true;
}

} // 命名空间 odin_test
