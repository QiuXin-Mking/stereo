# Kalibr 标定法介绍

> 本文介绍本项目中 `Kalibr Camera+IMU` 联合标定模式的输入、输出和基本原理。
> Kalibr 的官方名称是 **Kalibr**，项目文件名沿用用户侧的“kalib”叫法。

## 1. Kalibr 是什么

Kalibr 是 ETH Zurich Autonomous Systems Lab 开源的多传感器标定工具箱，主要用于相机、IMU 等视觉惯性传感器的内参、外参和时间关系标定。

本项目使用 Kalibr 完成：

- 双目相机的左右相机内参和畸变标定；
- 左右相机之间的相对位姿标定；
- 双目相机与内置 IMU 之间的旋转、平移外参标定；
- 根据图像和 IMU 的时间戳估计传感器之间的时间关系（具体是否启用由 Kalibr 命令参数决定）。

本项目的 Kalibr 模式只适用于能够同时提供左右图像和内置 IMU 数据的 `world intelligent` 相机。Kalibr 在 ARM64 Docker 容器内运行，RK3588 主机不需要安装完整 ROS 环境。

官方参考：[Kalibr Camera-IMU Calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)、[Kalibr YAML 格式](https://github.com/ethz-asl/kalibr/wiki/yaml-formats)。

## 2. 标定输入

### 2.1 原始采集数据

一次采集会保存到：

```text
sessions/<会话时间>/kalibr/
├── cam0/                 # 左相机灰度图
├── cam1/                 # 右相机灰度图
├── imu0.csv              # IMU 原始测量值
├── frames.csv            # 图像帧、时间戳和文件对应关系
├── capture.json          # 采集统计信息
└── decoder_stats.json    # 码带/IMU 解码统计信息
```

| 输入 | 内容 | 作用 |
|---|---|---|
| `cam0/*.png` | 左相机灰度图 | 提取 AprilGrid 角点，估计左相机参数 |
| `cam1/*.png` | 右相机灰度图 | 提取 AprilGrid 角点，估计右相机参数 |
| `imu0.csv` | IMU 时间戳、角速度、线加速度 | 建立视觉运动与惯性运动的约束 |
| `frames.csv` | 图像时间戳、曝光时间、左右图路径 | 将图像和 IMU 数据按时间关联 |
| `capture.json` | 录制时长、图像数、IMU 样本数等 | 判断采集是否完整 |
| `decoder_stats.json` | 解码率、重复时间戳等 | 判断 IMU 数据质量 |

`imu0.csv` 的主要字段如下：

```text
timestamp_ns,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z,frame_idx,t_us
```

- `timestamp_ns`：纳秒时间戳；
- `omega_x/y/z`：陀螺仪角速度，单位为 `rad/s`；
- `alpha_x/y/z`：加速度计测量值，单位为 `m/s²`；
- `frame_idx`、`t_us`：设备帧号和原始设备时间，便于追溯。

### 2.2 标定板配置 `target.yaml`

本项目使用 AprilGrid，不使用 OpenCV 模式的普通棋盘格。当前配置为：

```yaml
target_type: aprilgrid
tagCols: 8
tagRows: 6
tagSize: 0.020
tagSpacing: 0.30
```

| 参数 | 含义 |
|---|---|
| `target_type` | 标定板类型，此处为 AprilGrid |
| `tagCols` / `tagRows` | AprilTag 列数和行数，当前为 8×6 |
| `tagSize` | 单个 Tag 边长，单位 m，当前为 20 mm |
| `tagSpacing` | Tag 间距与 Tag 边长的比例，当前为 0.30 |

标定板物理尺寸必须与配置一致，否则会影响平移外参和尺度结果。

### 2.3 IMU 配置 `imu.yaml`

项目会根据采集数据生成 IMU 配置，典型内容为：

```yaml
rostopic: /imu0
update_rate: 300.0
accelerometer_noise_density: 0.02
accelerometer_random_walk: 0.002
gyroscope_noise_density: 0.002
gyroscope_random_walk: 0.0002
provisional: true
```

实际 `update_rate` 由数据检查阶段估计。噪声密度和随机游走参数目前是项目的临时配置，`provisional: true` 表示它们不是通过专门的 IMU 静态噪声实验得到的最终参数。

### 2.4 相机配置 `camchain.yaml`

相机内参标定完成后，Kalibr 先生成双目相机链文件。该文件记录每个相机的：

- 相机模型，项目当前为 `pinhole-radtan`；
- 图像分辨率；
- 相机内参 `fx、fy、cx、cy`；
- 畸变参数；
- 左右相机之间的相对外参。

在相机-IMU 联合标定阶段，Kalibr 使用该文件作为相机参数输入。

### 2.5 ROS bag 输入

项目不会要求操作员手动准备 ROS bag。系统将上述目录中的文件转换为：

```text
results/dataset.bag
```

其中包含三个 ROS topic：

```text
/cam0/image_raw    # 左相机图像
/cam1/image_raw    # 右相机图像
/imu0              # IMU 数据
```

图像和 IMU 消息按纳秒时间戳排序后写入 bag，左右图像数量必须一致。

## 3. 本项目的标定流程

```text
开始录制
    ↓
移动并转动相机 60～90 秒
    ↓
停止录制
    ↓
检查数据
    ↓
生成 target.yaml、imu.yaml、pipeline.json
    ↓
生成 dataset.bag
    ↓
Kalibr 相机内参/双目标定
    ↓
Kalibr 相机-IMU 联合标定
    ↓
生成报告和最终结果
```

当前项目使用的核心命令等价于：

```bash
kalibr_calibrate_cameras \
  --bag results/dataset.bag \
  --target target.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw

kalibr_calibrate_imu_camera \
  --bag results/dataset.bag \
  --cams results/camchain.yaml \
  --imu imu.yaml \
  --target target.yaml
```

第一步先估计左右相机参数和双目关系，第二步在此基础上加入 IMU 数据，估计相机与 IMU 的空间关系。

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

标定数据位于：

```text
sessions/<会话时间>/kalibr/
```

### 5.1 配置与中间结果

| 文件 | 说明 |
|---|---|
| `target.yaml` | AprilGrid 标定板配置 |
| `imu.yaml` | IMU topic、频率、噪声密度和随机游走参数 |
| `pipeline.json` | 本次求解使用的 topic、模型、标定板和质量指标 |
| `results/dataset.bag` | 转换后的 ROS bag，包含左右图像和 IMU |
| `results/bag_info.json` | bag 中各 topic 的消息数量和时间跨度 |

### 5.2 主要标定结果

| 文件 | 内容 | 用途 |
|---|---|---|
| `results/camchain.yaml` | 左右相机内参、畸变和双目外参 | 相机模型和双目几何关系 |
| `results/camchain-imucam.yaml` | 相机-IMU 联合标定结果 | 读取相机与 IMU 的旋转、平移及相关参数 |
| `results/summary.json` | 结果摘要和关键统计量 | 程序读取、交付记录和快速检查 |

`camchain-imucam.yaml` 是相机-IMU 联合标定的核心结果，通常重点查看：

- 相机内参和畸变参数；
- 左右相机之间的旋转和平移；
- 相机相对于 IMU 的旋转和平移；
- 重投影误差、样本数和优化状态；
- 若启用时间标定，对应的时间偏移量。

### 5.3 报告和日志

| 文件 | 说明 |
|---|---|
| `results/dataset-report-cam.pdf` | 相机/双目标定报告，包含角点检测和重投影误差等信息 |
| `results/dataset-report-imucam.pdf` | 相机-IMU 联合标定报告 |
| `results/cameras.log` | 相机标定阶段的完整日志 |
| `results/imu-camera.log` | 相机-IMU 标定阶段的完整日志 |
| `validation.json` | 进入求解前的数据质量检查结果 |
| `stage.json` | 当前求解阶段，如 `bagging`、`camera_calibrating`、`imu_calibrating`、`pass` 或 `error` |

## 6. 求解前的数据质量要求

本项目在开始 Kalibr 求解前会检查：

| 检查项 | 当前要求 |
|---|---:|
| 录制时长 | 至少 60 秒，推荐 90 秒 |
| IMU 采样率 | 250～400 Hz |
| IMU 码带解码成功率 | 至少 90% |
| 陀螺仪三轴激励 | 每轴峰峰值至少 0.35 rad/s |
| 加速度计三轴激励 | 每轴峰峰值至少 1.5 m/s² |
| 左右图像 | 数量一致、时间戳单调 |
| 双眼同时检测到标定板 | 至少 60 帧 |
| 视场覆盖 | 3×3 区域至少覆盖 5 个区域 |

质量检查不通过时，不应直接运行求解。应根据提示重新录制，例如补充某一轴的转动、增加平移、改善 AprilGrid 可见性或检查 IMU 解码。

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
