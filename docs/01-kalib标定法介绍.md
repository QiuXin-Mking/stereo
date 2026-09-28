# Kalibr 标定法介绍

> 本文介绍本项目中 `Kalibr Camera+IMU` 模式的录制输入、输出和 Kalibr 基本原理。本工程只负责录制，Kalibr 求解在外部进行。
> Kalibr 的官方名称是 **Kalibr**，项目文件名沿用用户侧的“kalib”叫法。

## 1. Kalibr 是什么

Kalibr 是 ETH Zurich Autonomous Systems Lab 开源的多传感器标定工具箱，主要用于相机、IMU 等视觉惯性传感器的内参、外参和时间关系标定。

本项目使用 Kalibr 完成：

- 双目相机的左右相机内参和畸变标定；
- 左右相机之间的相对位姿标定；
- 双目相机与内置 IMU 之间的旋转、平移外参标定；
- 根据图像和 IMU 的时间戳估计传感器之间的时间关系（具体是否启用由 Kalibr 命令参数决定）。

本项目的 Kalibr 模式只适用于能够同时提供左右图像和内置 IMU 数据的 `world intelligent` 相机。本工程只负责录制双目视频与 IMU 数据，Kalibr 求解（打 rosbag、相机/IMU 联合优化）在外部流程完成，RK3588 主机不运行 Kalibr 容器。

官方参考：[Kalibr Camera-IMU Calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)、[Kalibr YAML 格式](https://github.com/ethz-asl/kalibr/wiki/yaml-formats)。

## 2. 标定输入

### 2.1 录制产物

一次录制会保存到：

```text
sessions/<会话时间>/kalibr/
├── cam0.avi              # 左相机灰度视频（1920×1200，不含码带）
├── cam1.avi              # 右相机灰度视频（1920×1200，不含码带）
├── imu.json              # IMU 样本（时间戳、三轴陀螺、三轴加速度）
├── capture.json          # 采集统计信息
├── decoder_stats.json    # 码带/IMU 解码统计信息
├── dataset_manifest.json # 录制清单
└── integrity.json        # 完整性报告
```

| 产物 | 内容 | 作用 |
|---|---|---|
| `cam0.avi` / `cam1.avi` | 左右眼灰度视频（MJPEG 编码，不含码带） | 供外部 Kalibr 解帧提取 AprilGrid 角点 |
| `imu.json` | IMU 时间戳、角速度、线加速度 | 建立视觉运动与惯性运动的约束 |
| `capture.json` | 录制时长、帧数、IMU 样本数等 | 判断采集是否完整 |
| `decoder_stats.json` | 解码率、重复时间戳等 | 判断 IMU 数据质量 |

`imu.json` 的样本字段（兼容 `imu_decoder.ImuSample`）：

```text
timestamp_ns, frame_idx, t_us, gyro_rps[x,y,z], accel_mps2[x,y,z]
```

- `timestamp_ns`：纳秒时间戳；
- `gyro_rps`：陀螺仪角速度，单位为 `rad/s`；
- `accel_mps2`：加速度计测量值，单位为 `m/s²`；
- `frame_idx`、`t_us`：设备帧号和原始设备时间，便于追溯。

### 2.2 外部 Kalibr 流程所需的其余输入

AprilGrid 标定板配置（`target.yaml`）、IMU 噪声配置（`imu.yaml`）、相机链（`camchain.yaml`）以及 rosbag（`dataset.bag`）由外部 Kalibr 流程基于本工程录制的视频与 `imu.json` 准备，不在本工程内生成。

## 3. 本工程的录制流程

```text
开始录制
    ↓
移动并转动相机 60～90 秒（页面抽稀显示 IMU 数据）
    ↓
停止录制
    ↓
生成 cam0.avi、cam1.avi、imu.json
```

后续 Kalibr 求解（打 rosbag、相机内参/双目标定、相机-IMU 联合标定、生成报告）由外部流程完成。

## 4. Kalibr 的基本原理

### 4.1 相机观测约束

AprilGrid 上每一个 Tag 的角点在标定板坐标系中的三维位置是已知的。相机拍摄标定板后，Kalibr 检测这些角点在图像中的二维位置。

给定标定板角点三维坐标、相机内参和畸变模型，以及标定板相对于相机的位姿，可以将三维角点投影到图像中。投影点与实际检测点之间的差称为**重投影误差**：

```text
重投影误差 = 检测到的图像角点 - 模型投影得到的图像角点
```

Kalibr 会调整相机内参、畸变、标定板位姿和相机外参，使所有图像的重投影误差整体尽可能小。

### 4.2 IMU 观测约束

IMU 提供两类测量：

- 陀螺仪：测量角速度；
- 加速度计：测量比力/线加速度，并受到重力影响。

相机连续观察标定板，可以得到随时间变化的相机位姿。Kalibr 使用连续时间样条（spline）表示这段运动轨迹，再对轨迹求导得到角速度和加速度，与 IMU 的实际测量进行比较。

IMU 侧的误差主要包括：

- 预测角速度与陀螺仪测量之间的误差；
- 预测加速度与加速度计测量之间的误差；
- IMU 零偏、噪声和随机游走造成的误差。

### 4.3 相机与 IMU 外参

相机和 IMU 固定安装在同一个设备上，但两者的坐标系原点和轴方向不同。Kalibr 求解的外参通常包含：

- 旋转矩阵 `R`：IMU 坐标系与相机坐标系之间的轴向关系；
- 平移向量 `T`：IMU 原点与相机原点之间的空间距离；
- 必要时的时间偏移：图像时间戳与 IMU 时间戳之间的延迟。

同一段真实运动在相机和 IMU 中应该是相互一致的。Kalibr 通过同时最小化视觉重投影误差和 IMU 测量误差，找到一组能够解释全部观测的参数。

```text
相机看到标定板
        ↓
估计设备运动轨迹
        ↓ 与 IMU 测量对齐
联合优化内参、外参、偏置和时间关系
        ↓
输出相机链与相机-IMU 标定结果
```

官方文档将该过程描述为基于样条运动轨迹的全批量优化，而不是逐帧独立求解。这样可以利用整段采集数据的约束，提高外参和时间关系估计的稳定性。

### 4.4 为什么必须移动相机

Kalibr 标定的是相机和 IMU 的相对关系，因此必须让相机本体运动：

- 绕 X、Y、Z 三轴转动，激励陀螺仪；
- 前后、左右移动，激励加速度计；
- 让标定板出现在画面中央、边缘和不同距离，提升视觉约束；
- 保证左右相机能够同时看到标定板。

如果只移动标定板而相机保持静止，IMU 几乎没有有效运动信息，无法可靠估计相机-IMU 外参。推荐连续、缓慢地运动相机 60～90 秒，避免长时间停顿。

## 5. 输出结果

本工程录制产物位于：

```text
sessions/<会话时间>/kalibr/
```

| 文件 | 说明 |
|---|---|
| `cam0.avi` / `cam1.avi` | 左右眼灰度视频（不含码带） |
| `imu.json` | IMU 样本 |
| `capture.json` / `decoder_stats.json` | 采集与解码统计 |
| `dataset_manifest.json` / `integrity.json` | 录制清单与完整性报告 |

外部 Kalibr 求解的最终结果（`camchain.yaml`、`camchain-imucam.yaml`、PDF 报告等）由外部流程生成，不在本工程内。

## 6. 录制建议目标

本工程已移除内置求解与「检查数据」步骤，录制质量由操作者结合页面抽稀 IMU 显示自行判断。建议尽量满足：

| 项目 | 建议目标 |
|---|---:|
| 录制时长 | 至少 60 秒，推荐 90 秒 |
| IMU 采样率 | 250～400 Hz |
| IMU 码带解码成功率 | 至少 90% |
| 陀螺仪三轴激励 | 每轴峰峰值至少 0.35 rad/s |
| 加速度计三轴激励 | 每轴峰峰值至少 1.5 m/s² |
| 左右视频帧 | 数量一致、时间戳单调 |
| 双眼同时检测到标定板 | 至少 60 帧 |
| 视场覆盖 | 3×3 区域至少覆盖 5 个区域 |

## 7. 与 OpenCV Chessboard 标定的区别

| 项目 | OpenCV Chessboard | Kalibr Camera+IMU |
|---|---|---|
| 标定对象 | 左右双目相机 | 左右相机 + 内置 IMU |
| 标定板 | 普通棋盘格 | AprilGrid |
| 主要输入 | 左右图像对 | 左右图像 + IMU 时间序列 |
| 操作方式 | 相机可固定，移动棋盘 | 必须移动/转动相机本体 |
| 主要输出 | K、D、R、T、校正映射 | camchain、camchain-imucam、IMU 外参和报告 |
| 求解方式 | OpenCV 双目几何标定 | Kalibr 全批量视觉惯性联合优化 |

两种模式使用不同标定板，不能把 OpenCV 模式的普通棋盘格直接用于本项目 Kalibr 流程。

## 8. 结果使用注意事项

1. `camchain-imucam.yaml` 中的坐标变换方向必须结合字段名和 Kalibr 版本确认，不能只根据文件名猜测矩阵方向。
2. 标定完成后不要改变左右镜头或 IMU 的机械安装关系，否则外参会失效。
3. 标定板尺寸、相机分辨率、图像 topic 和 IMU 轴方向必须与实际设备一致。
4. `imu.yaml` 中标记为 provisional 的噪声参数只能作为当前工程的运行参数，不能直接当作高精度 IMU 噪声标定结论。
5. 交付结果时应同时保存 `camchain-imucam.yaml`、`summary.json`、两个 PDF 报告以及对应的 `pipeline.json`，便于复现和追溯。
