# 28com_ws — 无人车自主导航与视觉任务研究工作空间

[![ROS](https://img.shields.io/badge/ROS-1%20(catkin)-22314E?logo=ros&logoColor=white)](http://wiki.ros.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-11%2F14-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-NVIDIA%20Jetson-76B900?logo=nvidia&logoColor=white)](https://developer.nvidia.com/embedded-computing)
[![PX4](https://img.shields.io/badge/Controller-PX4%20%2B%20MAVROS-3C3C3D)](https://px4.io/)

一个基于 **ROS 1 (catkin)** 的**无人车（地面无人平台）**自主导航与视觉任务研究工作空间，围绕 **PX4 + MAVROS Offboard 控制** 构建，以**偏航角作为转向、油门作为前进速度**驱动底盘，集成了定位（Cartographer / VINS-Fusion / T265）、建图导航（A\* 全局规划、move_base）、视觉感知（YOLOE 目标检测、ArUco 检测）以及视觉穿门、精准停靠等任务脚本。

---

## 目录

- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [技术栈](#技术栈)
- [目录结构](#目录结构)
- [环境要求](#环境要求)
- [编译与安装](#编译与安装)
- [快速开始](#快速开始)
- [ROS 接口](#ros-接口)
- [关键参数配置](#关键参数配置)
- [相机标定](#相机标定)
- [视觉检测模型](#视觉检测模型)
- [第三方组件与致谢](#第三方组件与致谢)
- [注意事项](#注意事项)
- [许可证](#许可证)

---

## 功能特性

- **Offboard 控制**：基于 MAVROS 的 PX4 Offboard 姿态 + 油门控制，偏航角用于转向、油门用于前进，包含解锁、旋转对准、直行、到达停车与停车停留的状态机，输出到 `/mavros/setpoint_raw/attitude`。
- **全局路径规划**：`A_star` 栅格搜索 + `global_planner` 节点，结合前瞻转向（lookahead）与偏航（yaw）对齐控制，按航点类型下发姿态 + 油门指令。
- **导航栈**：`move_base` + costmap 局部/全局代价地图参数，支持局部避障与目标点导航。
- **多源定位**：
  - Cartographer 2D/3D SLAM（LiDAR + IMU）；
  - VINS-Fusion（双目/单目 + IMU）视觉惯性里程计；
  - Intel RealSense T265 里程计，提供车体坐标系到世界坐标系的位姿变换。
- **视觉感知**：
  - YOLOE（Ultralytics）开放词表目标检测，输出目标 bbox 与三维估计坐标；
  - ArUco 码检测、跟踪与动态/静态精准停靠（对准降落/停靠标板）；
  - 椭圆/圆环目标检测与视觉穿门（color circle cross）。
- **任务脚本**：一键拉起多终端多节点的任务流程（定位 + 建图 + 感知 + 规划 + 控制）与 ArUco 精准停靠、视觉穿门、方形航点等任务。
- **相机标定**：基于 OpenCV 的针孔相机内参与偏航角标定工具。

## 系统架构

```
                ┌──────────────────────────────────────────────┐
   传感器层      │  2D LiDAR (RPLIDAR S1/S2) │ RealSense T265/D435i │ usb_cam │ IMU │
                └──────────────────────────────────────────────┘
                                     │
                ┌────────────────────┴─────────────────────┐
   感知/定位层   │ Cartographer / VINS-Fusion / T265 里程计  │  YOLOE / ArUco 检测
                └────────────────────┬─────────────────────┘
                                     │  (tf, odom, detections)
                ┌────────────────────┴─────────────────────┐
   规划层        │        global_planner (A*) + move_base   │
                └────────────────────┬─────────────────────┘
                                     │  AttitudeTarget (偏航 + 油门) / setpoints
                ┌────────────────────┴─────────────────────┐
   控制/执行层   │        offboard_pkg  →  MAVROS  →  PX4    │
                └──────────────────────────────────────────────┘
```

## 技术栈

| 类别 | 技术 / 组件 |
| --- | --- |
| 机器人中间件 | ROS 1（catkin / roscpp / rospy / tf / tf2 / rviz） |
| 平台类型 | 无人车（地面无人平台，偏航转向 + 油门驱动） |
| 底盘控制与通讯 | PX4（Offboard 姿态/油门）、MAVROS、`mavros_msgs` |
| 编程语言 | C++11/14、Python 3、Bash |
| 构建系统 | CMake（catkin_make）、catkin 消息生成 |
| 数学 / 几何库 | Eigen3、Ceres Solver、Boost（thread / date_time） |
| 感知与图像 | OpenCV、`cv_bridge`、`image_transport`、PCL（`pcl_ros`）、ArUco |
| SLAM 与定位 | Google Cartographer（`cartographer_ros`）、VINS-Fusion、RealSense T265 |
| 规划 | A\* 全局规划器、`move_base`、`quadrotor_msgs`、ego-planner（外部） |
| 深度学习 | PyTorch、Ultralytics YOLOE、ONNX Runtime、NVIDIA TensorRT、MobileCLIP / CLIP、SAM2 |
| 硬件平台 | NVIDIA Jetson（NX / TX2）、PX4 系列控制器、RPLIDAR S1/S2、RealSense T265 & D435i、USB 相机 |
| 开发环境 | Ubuntu 18.04 / 20.04、VS Code（CMake Tools）、JetPack |

## 目录结构

```
28com_ws/
├── src/                          # ROS 功能包源码
│   ├── offboard_pkg/             # 核心包：Offboard 控制 + A* 全局规划 + 消息定义
│   │   ├── include/              # math_utils.h, message_utils.h
│   │   ├── src/planning/         # A_star.h / global_planner.{h,cpp} / node
│   │   ├── msg/                  # 自定义 ROS 消息
│   │   ├── launch/               # offboard.launch（参数）、goal.xml（航点）
│   │   ├── config/               # 相机参数、move_base 代价地图、rviz 配置
│   │   ├── dataset/              # 停靠目标数据集（landing_patches.txt）
│   │   └── sh/                   # 任务启动脚本（t265_* / aruco_* / ego_* …）
│   ├── Calibrate/                # 相机内参 / 偏航角标定（Python + OpenCV）
│   ├── model_location/           # 视觉检测与目标定位（YOLOE / MobileCLIP / ONNX / TensorRT）
│   ├── cartographer_ros-master/  # Cartographer ROS 集成（上游）
│   ├── rplidar_ros-master/       # RPLIDAR 驱动（上游）
│   ├── rviz_plugins/             # RViz 可视化插件
│   ├── map_server/               # 地图服务
│   └── pdf_images/               # 文档 / 图片资料
├── third_party/                  # 第三方上游开源库（见“第三方组件与致谢”）
│   ├── CLIP/
│   ├── lvis-api/
│   ├── ml-mobileclip/
│   └── sam2/
├── planner_sh/                   # 顶层任务编排脚本
│   ├── planner.sh                # 一键拉起 mavros / rplidar / cartographer / 感知 / offboard
│   ├── calibrate.sh              # 相机标定流程
│   └── kill_ros_pid.sh           # 清理 ROS 相关进程
├── build/                        # catkin 编译产物（不纳入版本控制）
├── devel/                        # catkin 开发空间（不纳入版本控制）
├── .catkin_workspace             # catkin 工作空间标记文件
└── README.md
```

## 环境要求

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 18.04 / 20.04（x86_64 或 NVIDIA Jetson aarch64） |
| ROS | ROS 1（catkin，Melodic 或 Noetic） |
| 编译工具 | CMake ≥ 3.0.2、g++（支持 C++11/14）、`catkin_make` |
| 依赖 | `roscpp`、`std_msgs`、`geometry_msgs`、`sensor_msgs`、`image_transport`、`cv_bridge`、`tf`、`tf2_eigen`、`visualization_msgs`、`pcl_ros`、`mavros_msgs`、`quadrotor_msgs`、`message_generation` |
| 系统库 | OpenCV、Eigen3、Ceres Solver、Boost（thread、date_time）、PCL |
| Python | Python 3.x、`rospy`、`numpy`、`pyyaml`、`opencv-python`、`torch`、`ultralytics`（含 YOLOE）、`onnxruntime` / TensorRT |

> 说明：`quadrotor_msgs` 与 ego-planner / VINS-Fusion 属于外部依赖，需按各自项目说明单独安装；本仓库的 `third_party/` 仅包含视觉相关上游库。

## 编译与安装

```bash
# 1. 安装 ROS 与系统依赖（示例：Noetic）
sudo apt install -y \
  libopencv-dev libeigen3-dev libceres-dev libboost-all-dev libpcl-dev

# 2. 编译工作空间
cd ~/28com_ws
source /opt/ros/<distro>/setup.bash
catkin_make            # 或 catkin_make -DCMAKE_BUILD_TYPE=Release

# 3. 生效环境
source devel/setup.bash
echo "source ~/28com_ws/devel/setup.bash" >> ~/.bashrc
```

编译产物说明：

- `offboard_pkg` 生成可执行文件 `global_planner_main` 以及全部自定义消息（`offboard_pkg_generate_messages_cpp`）。
- `build/`、`devel/` 为本地生成目录，已通过 `.gitignore` 排除。

## 快速开始

推荐直接使用 `planner_sh/planner.sh`，它会按顺序在多个 `gnome-terminal` 标签页中启动完整链路：

```bash
cd ~/28com_ws
chmod +x planner_sh/*.sh
./planner_sh/planner.sh
```

`planner.sh` 依次启动：

1. **mavros** — 连接底层控制器（`fcu_url`、`gcs_url` 需按实际串口/IP 修改）；
2. **rplidar** — 2D 激光雷达驱动；
3. **usb_cam** — USB 相机；
4. **cartographer** — 建图/定位（启动前会通过 `mavcmd` 校准传感器）；
5. **yoloE** — `src/model_location/auto_pose_ros1.py` 视觉目标检测与定位；
6. **offboard** — `offboard_pkg/offboard.launch` 全局规划与控制节点。

其他常用入口：

| 脚本 | 用途 |
| --- | --- |
| `src/offboard_pkg/sh/t265_circle_cross*.sh` | T265 定位 + 视觉穿门 / A\* / ArUco 停靠任务 |
| `src/offboard_pkg/sh/aruco_land_static.sh` / `aruco_land_dynamic_circle2.sh` | 静态 / 动态 ArUco 精准停靠 |
| `src/offboard_pkg/sh/cartographer_t265*.sh` | Cartographer + T265 定位与 move_base 导航 |
| `src/offboard_pkg/sh/d435i_color_*.sh` | D435i 视觉圆环检测与穿越/跟踪 |
| `src/offboard_pkg/sh/for_ego-planner/*.sh` | VINS-Fusion + ego-planner 自主导航 |
| `planner_sh/calibrate.sh` | 相机标定流程 |
| `planner_sh/kill_ros_pid.sh` | 结束所有 ROS 相关进程 |

> ⚠️ 脚本中包含 `sudo` 密码与固定串口/设备路径（如 `/dev/ttyTHS0`、`/dev/ttyUSB0`），请按自己的设备与安全策略修改后再运行。

## ROS 接口

### 自定义消息（`src/offboard_pkg/msg/`）

| 消息 | 说明 |
| --- | --- |
| `DroneState.msg` | 平台状态（连接/解锁/降落标志、模式、位置、速度、姿态、电量） |
| `PositionReference.msg` | 位置参考 / 目标点 |
| `PositionCommand.msg` | 位置指令输出（供规划层下发） |
| `ControlCommand.msg` | 控制指令 |
| `DetectionInfo.msg` | 通用检测结果 |
| `ArucoInfo.msg` / `MultiArucoInfo.msg` | 单个 / 多个 ArUco 码检测信息 |
| `Message.msg` | 通用调试/状态消息 |

> 说明：部分消息名（如 `DroneState`、`PositionReference`）以及 `config/camera_param.yaml` 中的“降落板”参数沿用自早期无人机项目；在本工作空间中分别对应车体状态与停靠/标记板。

### 主要话题（脚本中涉及）

| 话题 | 说明 |
| --- | --- |
| `/mavros/state`、`/mavros/local_position/pose` | 控制器状态与本地位置 |
| `/mavros/setpoint_raw/attitude` | Offboard 姿态 + 油门设定点（偏航转向 + 油门） |
| `/mavros/local_position/velocity_body` | 车体坐标系速度反馈 |
| `/drone/object_detection/ellipse_det` | 椭圆/圆环目标检测 |
| `/drone/object_detection/aruco_det` | ArUco 检测 |
| `/det/vis_image`、`/det/xyz` | YOLOE 可视化图像与目标三维坐标 |

## 关键参数配置

主要参数集中在 [`src/offboard_pkg/launch/offboard.launch`](src/offboard_pkg/launch/offboard.launch)：

| 参数 | 含义 |
| --- | --- |
| `ctrl_interval` / `precision` / `arrival_delay` | 控制周期、到达判定半径、到达后的停留时间 |
| `yaw_p` / `yaw_i` / `yaw_max` / `yaw_precision` | 偏航角 PID 与限幅、对准精度阈值 |
| `yaw_overshoot_left/right` / `yaw_overshoot_threshold` | 左右方向的偏航过冲补偿 |
| `thrust_set_min` | 行进基础油门（0.0 ~ 1.0） |
| `lookahead_angle` / `lookahead_distance` | 前瞻转向的触发角度与距离 |
| `feedback_check_interval_min/max`、`alignment_yaw_threshold` | 动态反馈修正与对齐偏差阈值 |
| `camera_offset_x/y`、`extra_offset_factor_left/right` | 相机安装偏移与视觉补偿系数 |
| `goal_file_path` | 航点文件，默认 `launch/goal.xml` |

航点文件 [`src/offboard_pkg/launch/goal.xml`](src/offboard_pkg/launch/goal.xml) 中，`type` 字段定义航点行为：

| type | 行为 |
| --- | --- |
| `0` | 普通航点：先原地旋转对准，再直行至到达精度范围内 |
| `1` | 视觉航点：依据视觉检测目标计算实际坐标（用于视觉穿门） |
| `2` / `3` | 原地左转 / 右转 90°（保留，暂未使用） |
| `4` | 前瞻转向航点：距离小于 `lookahead_distance` 时边走边转向下一个目标点，完成条件为**车头对准**下一目标点（而非到达该点） |

## 相机标定

`src/Calibrate/` 提供基于 ROS + OpenCV 的标定工具：

```bash
# 标定流程（会自动启动 usb_cam 与标定节点）
./planner_sh/calibrate.sh
```

- `Calibrate.py`：采集棋盘格图像并计算内参，输出 `cali.yaml`；
- `calibrate_yaw.py`：标定相机偏航安装角；
- `cali.yaml`：相机内参矩阵，被 `model_location/auto_pose_ros1.py` 读取用于单目测距。

## 视觉检测模型

`src/model_location/` 提供检测与定位脚本：

| 文件 | 说明 |
| --- | --- |
| `auto_pose_ros1.py` | ROS 1 节点：YOLOE 推理 + 单目测距，发布检测可视化与三维坐标 |
| `auto_pose_ros2.py` | ROS 2 版本 |
| `test_engine.py` / `test_onnx.py` | TensorRT / ONNX 推理测试 |
| `model/best.pt`、`best.onnx`、`best.engine` | 训练好的检测模型（PyTorch / ONNX / TensorRT） |
| `mobileclip_blt.ts` | MobileCLIP 文本编码器 TorchScript 模型 |

> 训练产物目录（`runs/`、`train622/`、`train662/`）体积较大，已通过 `.gitignore` 排除，如需分享请使用 Git LFS、GitHub Release 或网盘。

## 第三方组件与致谢

`third_party/` 目录包含以下上游开源项目，版权归各自作者所有，遵循其各自的许可证：

| 组件 | 说明 | 链接 |
| --- | --- | --- |
| **CLIP** | OpenAI 对比语言-图像预训练模型 | <https://github.com/openai/CLIP> |
| **ml-mobileclip** | Apple MobileCLIP（高效图文模型） | <https://github.com/apple/ml-mobileclip> |
| **SAM 2** | Meta Segment Anything Model 2 | <https://github.com/facebookresearch/sam2> |
| **lvis-api** | LVIS 数据集标注 API | <https://github.com/lvis-dataset/lvis-api> |

`src/` 下的 `cartographer_ros-master`、`rplidar_ros-master` 同样来自上游项目：

- Cartographer ROS — <https://github.com/cartographer-project/cartographer_ros>
- RPLIDAR ROS — <https://github.com/Slamtec/rplidar_ros>

此外本项目思路与接口参考了 ego-planner、VINS-Fusion、PX4、MAVROS 等开源项目，特此致谢。

## 注意事项

- 本项目为**研究用途**，脚本中包含硬编码的绝对路径（`~/28com_ws`、`~/multifunc_ws`）、设备节点、IP 地址与明文 `sudo` 密码，上传/共享前请务必清理敏感信息。
- 实车运行前请确认：串口权限、底层控制器连接、传感器标定、`gcs_url` 地面站地址、航点文件均已正确配置，并在封闭安全场地测试。
- 首次运行建议先在 SITL（PX4 软件仿真）或离地架空、低速的状态下验证话题与规划输出，再进行实车测试。

## 许可证

本仓库自身代码暂未指定开源许可证（`offboard_pkg` 中为 `TODO`）。`third_party/` 与 `src/*-master/` 下的各上游项目遵循其各自许可证，请在再分发时保留相应许可证与版权声明。
