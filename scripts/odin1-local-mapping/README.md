# ODIN1 本机内部建图验证

此工具在 x86_64 Linux 开发机上直接调用厂商 SDK。设备在 `LIDAR_MODE_SLAM`、`map_mode=1` 下建图。主机在开流后清零并读回 `save_map`，由操作员触发保存、取图和取消。

## 构建与离线验证

使用 Bash、Git、g++（C++17）、libusb-1.0 开发包和 Python 3。以下命令从项目根目录运行。将指定提交的厂商仓库检出到独立目录，已有对应版本的本地副本也可直接作为 `sdk`。

```bash
sdk="$PWD/third_party/src/odin_ros_driver-0.14.5"
git clone https://github.com/manifoldsdk/odin_ros_driver.git "$sdk"
git -C "$sdk" switch --detach 9487890acd648f1c7647576d1ad2b04dd7fd3677

tool="$PWD/scripts/odin1-local-mapping"
bundle=$(bash "$tool/build.sh" "$sdk")
python3 "$tool/test/run_tests.py" --sdk-include "$sdk/include"
python3 "$tool/test/test_entrypoints.py" --sdk-include "$sdk/include" \
    --candidate-binary "$bundle/odin1-local-mapping"
```

构建时校验提交及两个头文件、AMD64 静态库的 SHA-256。每次构建默认在仓库已忽略的 `artifacts/odin1-local-mapping/` 下创建独立目录。要临时指定构建位置，可执行 `bundle=$(bash "$tool/build.sh" "$sdk" "$(mktemp -d)/build")`，第二个参数为尚不存在的构建目录。模拟测试使用合成 SDK 和临时目录，入口检查通过 `nm` 检查真实二进制的清理符号。

## 人工建图

在有图形桌面的开发机连接一台 ODIN1，执行 `bash "$bundle/start-manual.sh"` 打开日志和控制终端。运行需有 `gnome-terminal`、`lsusb`、`flock` 和当前 USB 节点的读写权限。窗口显示本轮快照目录，地图、标定和设备日志均写在该目录中。

1. 日志出现 `MAPPING_STARTED` 后开始采集。采集结束并停稳，在控制终端输入 `s` 并回车。
2. 保存标志回到 0 后程序请求下载。核对 `MAP_SAVED` 中的文件名、实际字节数和传输日志。300 秒生成等待超时后可按 `s` 继续查询；下载失败后可按 `s` 重试下载。
3. 按 `c` 并回车取消建图，看到 `MAPPING_CANCELLED mode=raw` 后按 `q` 并回车退出。查看本轮 `exit-code.txt` 和 `probe.log`；退出码 21 表示切换 RAW 失败。地图能否用于重定位须另行实测。
