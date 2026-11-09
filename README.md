# spacemit-vslam

面向 SpacemiT K3 的 ROS 2 双目视觉惯性定位与三维建图项目。输入双目图像和 IMU，
输出位姿、轨迹、双目深度及 TSDF/ESDF，在正常桌面窗口显示相机和建图结果。

VSLAM、Mapping 核心以已 strip 的 RISC-V 共享库和公共接口交付；相机接入、
ROS 2 封装及启动配置保留源码。

## 功能与目录

```text
middleware/ros2/slam/spacemit-vslam/
├── modules/
│   ├── camera/          # 原生 C++ 相机、元数据与缓冲区接口
│   ├── vslam/           # VSLAM 二进制 SDK
│   └── mapping/         # TSDF/ESDF 与渲染二进制 SDK
├── src/
│   ├── camera_io/       # 相机/IMU ROS 接入、录制和回放
│   ├── vio/             # 位姿、轨迹与特征发布
│   ├── las2/            # SDK LAS2 深度推理封装与四向预览
│   ├── esdf_drm/        # 异步建图、桌面显示与录屏
│   ├── common/          # 消息与就绪工具
│   └── bringup/         # 启动、配置及会话控制
├── dependencies/       # SDK 依赖路径和查找配置
├── tools/              # 构建、二进制校验和构建工具测试
├── LICENSES/           # 第三方许可补充材料
├── LICENSE
└── README.md
```

桌面主链路：

```text
camera_mpp_publisher ──双目图像/IMU──► vio_node ──位姿──┐
                    └──────────────► las2_node ─深度──┤
                                                     ▼
                                               esdf_drm_node
                                                     │
                                                     ▼
                                              esdf_desktop_viewer
```

图像和深度通过 DMA-BUF 描述符传递，各阶段独立调度，使用有界队列。
项目还提供录制回放、四向相机深度预览，以及可选的 RTSP、Foxglove 和视频传输接入。

## 环境与依赖

- K3 / X100，Linux `riscv64`，配套 Bianbu 4.0.1 和 ROS 2 Humble 环境。
- C++17、CMake 3.22 或以上，以及 SDK 配套编译工具和各包声明的依赖。
- 相机、MPP/V2D、DMA-BUF、PowerVR EGL/GLES/OpenCL 及桌面显示所需驱动和权限。
- 与 LAS2 ExternalStereo 匹配的 ONNX Runtime、SpacemiT EP 和 FP16 模型工件。

按 Robot SDK 的安装流程准备系统环境。GPU 用户态库须与板端镜像配套；应用启动
日志中的 renderer 应为 PowerVR。首次安装图形驱动后需要重新登录桌面，日常运行
直接在桌面终端启动。

项目复用 SDK 已有内容，`dependencies/` 不包含依赖源码：

| 依赖 | SDK 路径 |
| --- | --- |
| MPP、MPP 消息 | `components/multimedia/mpp` |
| LAS2 ExternalStereo | `components/perception/stereo_matching` |
| IMU DDS | `middleware/ros2/peripherals/imu` |

两个二进制 SDK 只支持配套 Linux/RISC-V64 环境，具体 ISA、ABI、动态依赖和文件
SHA256 见 `modules/{vslam,mapping}/sdk/riscv64/manifest.json`。
VSLAM 的 LMDB 开启；Mapping 包含 rendering，map_io/SQLite 关闭。
消费端 CMake 开关不会重新编译核心算法或改变二进制功能。

## 使用 SDK 构建

在 K3 上执行，将工件目录替换为实际路径：

```bash
cd /path/to/spacemit_robot
source build/envsetup.sh
export ARTIFACT_ROOT=/path/to/matched-artifacts
cd middleware/ros2/slam/spacemit-vslam
python3 tools/build.py --check
AUTO_INSTALL_DEPS=no mm -j4 -- \
  -DONNXRUNTIME_ROOT="$ARTIFACT_ROOT/onnxruntime" \
  -DSTEREO_MATCHING_EXTERNAL_INPUT_MODEL="$ARTIFACT_ROOT/model/LAS2_M_288x384.fp16.onnx"
```

`mm` 经 SDK 的项目入口调用本项目 `build.sh`，编排原生依赖和 ROS 包。
VSLAM/Mapping 在此步骤校验、安装预编译库；相机、LAS2 和 ROS 封装正常编译。
默认安装前缀为 SDK 的 `output/staging`，无需创建产品 target。

## 桌面启动与标定

在已登录的桌面终端加载 SDK 和构建产物，再运行：

```bash
cd /path/to/spacemit_robot
source build/envsetup.sh
sros2_setup
ros2 run bringup run_vslam_desktop.sh
```

默认显示 1280×720 窗口，使用包内双目输入配置和 `camera_io/config/0.yaml`。
按 `Ctrl+C` 停止。使用 `--help` 查看入口说明，可通过 launch 参数调整：

```bash
ros2 run bringup run_vslam_desktop.sh \
  camera_fps:=30 display_width:=1920 display_height:=1080 desktop_fullscreen:=true
```

若硬件访问需要管理身份，可用 `SPACEMIT_DESKTOP_UID` 指定已登录桌面用户的数值 UID，
并保留该桌面的 `DISPLAY`、`XAUTHORITY`。后台和查看器使用相同运行 UID 及
`ROS_DOMAIN_ID`；入口的 `--backend-only`、`--viewer-only` 支持分开启动。

仓库只提供一份标定：`src/camera_io/config/0.yaml`，包含 `cam0`、`cam1` 和 `imu0`。
它对应特定实机，其他相机应提供自己的 Kalibr 文件：

```bash
ros2 run bringup run_vslam_desktop.sh \
  camera_config_file:=/path/to/camera.json \
  calibration_file:=/path/to/calibration.yaml \
  imu_calibration_file:=/path/to/calibration.yaml
```

相机 JSON 控制设备、输入格式和图像布局；Kalibr 文件提供内参、畸变、刚体外参和
IMU 噪声。更换标定时，同步更新相机 JSON 的 `calibration.file`，使相机消息与定位、
深度使用同一标定。保持物理左右目对应关系；缩放图像时按比例调整内参，不缩放畸变系数和
刚体变换。图像与嵌入式 IMU 使用同一硬件时钟，不另加人为时间偏移。
惯性初始化需要在有纹理的场景中适当平移、转动相机。

## 其他入口

| 功能 | 入口与条件 |
| --- | --- |
| 录制 | `ros2 launch bringup vio_record_launch.py --show-args` |
| 会话控制 | `ros2 run bringup run_vio_session.sh --help`；配置示例为 `bringup/config/vio_session.example.json` |
| 回放 | `ros2 run bringup run_vio_replay.sh RECORDING_DIRECTORY NEW_OUTPUT_DIRECTORY [name:=value ...]` |
| 四向相机深度预览 | `ros2 launch bringup quad_las2_launch.py --show-args`；须提供真实四目标定和模型/库路径，不能使用双目 `0.yaml` |
| RTSP | `run_stereo_las2_rtsp.sh`、`run_headless_esdf_rtsp.sh`；启用推流须另行提供 `mpp_rtsp` 包 |
| Foxglove | `run_live_vio_foxglove.sh`；须提供 `FOXGLOVE_RUNNER` 和相应外部服务，脚本同时使用 `mpp_rtsp` |
| 视频传输 | `run_reconstruction_video_transmission.sh --help`；须另行提供 `video_transmission` 包 |

表中脚本通过 `ros2 run bringup <脚本名>` 调用。额外服务不属于默认桌面链路。
录制、回放和会话脚本使用以下工作区和安装前缀配置：

```bash
export SPACEMIT_ROBOT_WORKSPACE_ROOT="$SROBOTIS_ROOT/middleware/ros2/slam/spacemit-vslam"
export SPACEMIT_ROBOT_SDK_PREFIX="${PREFIX:-$SROBOTIS_ROOT/output/staging}"
```

记录、输出和模型使用仓库外的目录；回放会话优先读取录制时保存的标定。

## 常见问题与工具检查

- 找不到 ROS 包：重新加载 SDK 环境和 `sros2_setup`，确认使用当前安装前缀。
- 找不到 ORT/EP 或模型：检查工件路径和 `output/staging`；可用 `LAS2_ORT_LIBRARY_DIR`、
  `LAS2_MODEL_PATH` 指定匹配运行库和模型。
- 软件渲染或图形初始化失败：检查 PowerVR、EGL/OpenCL 配套关系及桌面环境。
- 标定加载失败：检查路径、Kalibr 键名、左右目映射和 IMU 参数；四向输入需要四目标定。
- 二进制校验失败：使用完整、匹配的 SDK 文件，核对清单中的架构和 SHA256。

构建工具的主机检查入口：

```bash
python3 tools/test_build.py
python3 tools/test_binary_sdk.py
```

## 许可

集成源码的许可证见 [LICENSE](LICENSE)。
各二进制 SDK 及第三方材料保留随包提供的许可文件。
