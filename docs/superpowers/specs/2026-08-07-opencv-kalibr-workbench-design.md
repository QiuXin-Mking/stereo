# OpenCV / Kalibr 双模式标定工作台设计

## 目标

将现有 RK3588 双目棋盘标定网页升级为两种可选工作流：

1. **OpenCV Chessboard**：保留现有引导式/手动拍摄、双目内外参求解和多格式导出。
2. **Kalibr Camera+IMU**：连续采集 UVC SBS 图像，从左侧码带解码 IMU，在 Mac Docker 中完成双目相机标定和相机–IMU联合标定。

最终用户只通过 `http://127.0.0.1:18765/` 一个入口启动采集、查看数据质量、运行求解和获取产物。

## 系统边界

### RK3588

- 打开 `/dev/video0`，请求 `MJPG 4000×1200@30`。
- 验证设备名称、分辨率和左侧 160px 竖向码带，标记为 `world intelligent`。
- 从每帧左侧码带解码 ICM42688 和 AK09940 数据。
- 将原始帧裁分为左右 `1920×1200` 图像。
- 保存连续图像、IMU、曝光时间和会话元数据；向 Mac 提供状态和控制 API。
- 不在板端安装 ROS 或 Kalibr。

### Mac

- 运行本地编排器，监听 `127.0.0.1:18765`。
- 使用隐藏 SSH 转发端口访问 RK3588 Web/API/MJPEG。
- 代理现有 RK 界面和 API，并提供 Mac 本地 Kalibr API。
- 通过 `rsync` 或 `scp` 拉取 Kalibr 数据集。
- 启动 Docker 完成 ROS bag 构建、Kalibr 求解和结果解析。
- 将结果摘要和文件链接提供给同一网页。

## Mac 本地编排器

`deploy/start.sh` 从直接将 `18765` 转发到 RK，改为：

- SSH 隐藏转发：`127.0.0.1:18766 -> RK3588:127.0.0.1:8765`。
- Mac 编排器：`127.0.0.1:18765`。
- `/api/kalibr/*` 由 Mac 处理。
- 其他 HTTP 、动作 API 和 MJPEG 流代理到 `127.0.0.1:18766`。

编排器必须有独立 PID、日志和健康检查，重复执行 `deploy/start.sh` 不会产生重复进程。

## UI 和状态机

### 公共区域

- 顶部模式选择：`OpenCV Chessboard` / `Kalibr Camera+IMU`。
- 设备标签、UVC 模式、单眼分辨率、码带状态和实时预览。
- 采集期间禁止切换模式。

### OpenCV 模式

保持现有行为：32 姿态槽位、自动采集开关、手动一按必保存、5帧择优、CLAHE 角点重试和 OpenCV 求解产物。

### Kalibr 模式

状态依次为：

`ready -> recording -> recorded -> syncing -> bagging -> camera_calibrating -> imu_calibrating -> pass/retake/error`

界面显示：

- 录制时长、左/右图像帧数和实时 FPS。
- IMU 码带解码成功率、加速度/陀螺仪样本率、时间戳回退和丢包数。
- 时间同步状态、数据集大小、Docker/Kalibr 阶段、最新日志。
- 按钮：开始录制、结束录制、同步到 Mac、开始 Kalibr 求解。
- 产物：camchain、IMU–相机外参、时间偏移、PDF 报告、ROS bag 统计和完整日志。

## IMU 码带解码

实现以 `unified_capture/hardware/imu/imu_decode.h` 为协议基准，不改变位序、阈值或单位定义：

- 灰度阈值：`IMU_VAL0=50`、`IMU_VAL1=220`。
- 单位像素宽度：`IMU_USIZE=8`。
- 分组长度：`IMU_GROUP=16`，目标数据长度 `272` 字节。
- world intelligent 使用竖向扫描，从左边向内每 8px 取列。
- 同步头之后以两个灰度值之和判定 0/1，比特按 LSB-first 组字节。
- Header 提供设备类型、样本数和 `exp_start_us/exp_end_us`。
- ICM42688 数据为大端 `t_us + 3×acc int16 + 3×gyro int16`。
- AK09940 解码保留供诊断，Kalibr bag 仅写加速度和陀螺仪。

单位转换：

- `ax/ay/az_mg * 9.80665 / 1000 -> m/s²`。
- `gx/gy/gz_mdps * pi / 180000 -> rad/s`。

验证必须包括合成比特码带、C++ 参考实现对比和 RK3588 实帧解码。

## 时间戳与同步

- IMU `t_us`、`exp_start_us`、`exp_end_us` 视为相同设备时钟域。
- 图像时间戳使用 `(exp_start_us + exp_end_us) / 2`。
- 对 32 位微秒计数器做回绕展开，转换为 64 位单调时间。
- 左右图像使用同一曝光中点。
- 写 bag 前检查图像和 IMU 时间严格单调，记录重复、回退、断层和采样率。
- Kalibr 继续估计最终相机–IMU 时间偏移。

## 数据集格式

RK session 下新增：

```text
kalibr/
  cam0/<timestamp_ns>.png
  cam1/<timestamp_ns>.png
  imu0.csv
  frames.csv
  capture.json
  decoder_stats.json
```

`imu0.csv` 字段：

```text
timestamp_ns,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z,frame_idx,t_us
```

`frames.csv` 保存帧索引、图像时间戳、曝光边界、解码字节数和该帧 IMU 样本数。

## Kalibr Docker

- Mac 为 Apple Silicon，容器显式固定平台和 Kalibr 提交，保证可重现。
- 首选 Docker Desktop；可以在不改变命令接口的前提下使用 Colima。
- Docker 中完成数据集到 ROS bag 的转换，不要求 macOS 安装 ROS。
- bag topics 固定为 `/cam0/image_raw`、`/cam1/image_raw`、`/imu0`。
- 目标板使用 Kalibr checkerboard：8×5 内角点，方格 0.020m。
- IMU 先使用 ICM42688 保守默认噪声参数，产物标记“工程验证参数”。
- 顺序运行相机链标定和 IMU–相机联合标定；第一步产物作为第二步输入。

## 快速默认 IMU 参数

首个端到端版本使用保守默认值，并在配置和报告中明示标记 provisional：

- update rate 从实测 IMU 时间戳中估计，期望约 330Hz。
- accelerometer noise density: `0.02 m/s²/sqrt(Hz)`。
- accelerometer random walk: `0.002 m/s³/sqrt(Hz)`。
- gyroscope noise density: `0.002 rad/s/sqrt(Hz)`。
- gyroscope random walk: `0.0002 rad/s²/sqrt(Hz)`。

后续可通过长时间静置数据替换，不改变整体流程。

## 质量门槛

开始 Kalibr 前必须满足：

- 录制时长至少 60 秒，推荐 90–120 秒。
- 左右图像帧数完全一致。
- 码带帧解码成功率不低于 90%。
- IMU 中位样本率在 250–400Hz。
- 时间戳无未解释回退，无左右图像时间差。
- 至少包含三轴旋转和明显的加速/减速；动态激励检查不足时提示补录。
- 目标板在数据集中有足够的视场边缘、距离和倾斜覆盖。

## 产物

### OpenCV

保持 YAML、JSON、NPZ、校正映射和校正预览。

### Kalibr

- ROS bag 和 topic 统计。
- `target.yaml`、初始 `imu.yaml`、相机链配置。
- 双目 camera chain YAML 和 PDF 报告。
- IMU–相机联合 YAML、PDF 和时间偏移。
- 标准化 `summary.json`，包含 K/D/R/T、IMU 外参、时间偏移、重投影统计、数据集质量和 provisional 声明。
- 完整 stdout/stderr 日志和容器版本元数据。

## 错误处理

- 码带协议不匹配时保留原始帧和诊断图，不产生伪 IMU 样本。
- 相机或 SSH 断开后停止录制并保留已写入数据，会话标记 incomplete。
- Docker 服务未启动、镜像不存在或磁盘不足时，在开始求解前报错。
- Kalibr 任一阶段失败时保留 bag、配置和日志，不覆盖上一次成功结果。
- 模式切换、录制、同步和求解 API 必须幂等。

## 端到端验收

1. 现有 OpenCV 全套测试继续通过，网页可切换模式且 OpenCV 功能不回归。
2. 合成码带单元测试覆盖同步头、阈值、LSB-first、大端整数、旧协议和 32 位时间回绕。
3. 同一原始帧的 Python 解码字节与 C++ 参考实现一致。
4. RK3588 实录至少 10 秒，验证图像数、IMU 样本率、解码成功率、单调性和数据文件完整性。
5. Mac Docker 生成 bag，`rosbag info` 证明三个 topic 及正确频率。
6. 进行 60–120 秒实际标定录制，顺序完成 camera calibration 和 camera-IMU calibration。
7. 网页显示通过/需补录结论，并能下载产物。

## 非目标

- 首版不在 RK3588 安装 ROS/Kalibr。
- 首版不实现完整 Allan variance 长时间噪声标定。
- 首版不使用磁力计参与 Kalibr 求解。
- 不删除或改写现有 OpenCV 标定产物。

