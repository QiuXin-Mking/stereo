# RK3588 无界面双目标定与 Git 同步设计

日期：2026-08-05  
状态：设计已批准，待实现计划

## 1. 目标

把现有 macOS SBS 棋盘格标定 MVP 扩展到 RK3588。代码由本地 Git 仓库管理，通过 SSH 推送到 RK3588 裸仓库，再由远端运行目录拉取。RK3588 不依赖显示器，通过局域网浏览器实时预览和控制标定。

## 2. 已确认硬件环境

- RK3588：`root@192.168.100.200`，Debian 11，aarch64，Linux 5.10.160。
- Python 3.9.2、OpenCV Python 5.0.0、NumPy 2.0.2。
- OpenCV C++ 4.5.1、CMake 3.18.4、G++ 10.2.1。
- 相机采集节点：`/dev/video0`。
- 相机元数据节点：`/dev/video1`，不用于图像采集。
- 相机名称：`ZXCZ SC233HGS Dual: UVC Camera`。
- 最高 SBS 模式：MJPEG `3840×1080 @ 30 FPS`，单眼 `1920×1080`。

## 3. Git 与部署结构

本地目录 `stereo_chessboard_calibrator/` 初始化为 `main` 分支 Git 仓库。RK3588 使用两个目录：

```text
/root/git/stereo_chessboard_calibrator.git   # 裸仓库，接收 SSH push
/root/stereo_chessboard_calibrator           # 工作克隆，运行测试与服务
```

本地配置名为 `rk3588` 的 remote。首次部署创建裸仓库、推送 `main` 并克隆。后续部署执行 `git push rk3588 main`，再在远端运行目录执行仅快进拉取。`.venv/`、`sessions/`、缓存和浏览器设计临时文件不进入 Git。

## 4. 跨平台采集边界

设备打开逻辑从现有 macOS 专用实现中分离：

- macOS：保留 AVFoundation 名称解析和索引覆盖。
- Linux：使用 V4L2 设备路径，默认 `/dev/video0`。
- Linux 启动前通过 `v4l2-ctl` 验证设备、MJPEG、3840×1080 和 30 FPS。
- OpenCV 采集设置 `CAP_V4L2`、`MJPG`、3840×1080、30 FPS，并读取实际返回的宽、高、FPS 和 FOURCC。
- 实际模式与请求不一致时停止，不允许静默用较低分辨率标定。
- 求解和保存继续使用未缩放的 3840×1080 SBS 原始帧；浏览器只接收缩小后的预览 JPEG。

平台后端只负责打开和读取帧。SBS 拆分、棋盘检测、质量门控、求解和导出保持平台无关。

## 5. 无界面 Web 模式

新增入口：

```bash
./calibrate --web --device /dev/video0 --host 0.0.0.0 --port 8765 --square-mm 20.00
```

浏览器访问 `http://192.168.100.200:8765`。第一版使用 Python 标准库 `ThreadingHTTPServer`，不引入 Flask、FastAPI 或 WebSocket。

### HTTP 接口

- `GET /`：单页中文控制台。
- `GET /stream.mjpg`：缩小后的 MJPEG 实时预览。
- `GET /api/status`：JSON 状态，包括设备模式、角点状态、拒绝原因、稳定进度、已接受数量、当前引导、求解指标和最终状态。
- `POST /api/action`：JSON 动作，只允许 `pause`、`resume`、`undo`、`solve` 和 `stop`。

服务启动后由一个后台采集线程独占 `/dev/video0`。HTTP 线程只读取最新预览 JPEG 和状态快照，不直接访问相机。共享数据用锁保护，原始帧不通过网络发送。

### 浏览器交互

页面显示：

- 左右眼并排预览和棋盘角点。
- 完整 SBS 模式与单眼分辨率。
- 当前最主要拒绝原因。
- 稳定倒计时、下一姿态提示、已接受/目标数量。
- 暂停、继续、撤销、开始求解和停止服务按钮。
- PASS/RETAKE、左右 RMS、极线误差 P95 和结果目录。

达到目标数量后自动求解。操作者也可在不少于 20 对时点击“开始求解”。低于 20 对时 API 拒绝求解并返回明确错误。

## 6. 状态与错误处理

服务状态为 `starting`、`capturing`、`paused`、`solving`、`pass`、`retake`、`error`、`stopped`。相机打开失败、模式不一致、连续读帧失败、写盘失败或求解异常都会进入 `error`，并在浏览器和服务日志显示原因。

`undo` 只在非求解状态删除最近一对。`stop` 释放相机、保存 session 状态并关闭服务。第一版局域网 Web 服务不做认证，因此只在可信的 `192.168.100.0/24` 网络临时运行，完成后停止。

## 7. 远端依赖

优先使用 RK3588 系统已有 OpenCV/NumPy。创建带 `--system-site-packages` 的项目虚拟环境，只安装缺失的 PyYAML 和 pytest。不得通过 pip 覆盖系统 OpenCV，以免改变硬件视频支持。

## 8. 测试顺序

1. 本地运行全部 Python 测试。
2. 推送 Git 并在 RK3588 仅快进拉取。
3. RK3588 创建虚拟环境并安装 PyYAML/pytest。
4. RK3588 运行全部离线 Python 测试。
5. RK3588 构建 C++ 加载示例。
6. 用 V4L2/OpenCV 连续读取 `/dev/video0`，确认每帧 3840×1080、MJPEG、目标帧率和无连续失败。
7. 启动 Web 服务，使用 `curl` 验证 `/`、`/api/status`、`/stream.mjpg` 和非法动作处理。
8. 用户浏览器访问 `192.168.100.200:8765` 验证实时预览和控制。
9. 棋盘进入双眼画面后完成至少 20 对真实采集、求解和导出；验证 YAML、JSON、NPZ、maps 和 C++ remap。

真实标定必须有打印棋盘和人工移动姿态才能完成。自动化阶段以设备模式、连续采帧、Web 流、离线求解和 C++ 构建通过为边界；浏览器人工验证和真实标定在服务启动后连续进行。

## 9. 成功标准

- 本地 Git 可以重复推送，RK3588 工作克隆可以仅快进更新。
- RK3588 严格以 `/dev/video0` 的 MJPEG 3840×1080@30 模式采集。
- 浏览器能看到低延迟左右预览、质量状态和采集进度，并能控制暂停、撤销、求解和停止。
- 离线测试和 C++ 示例在 RK3588 通过。
- 棋盘具备后，可在 RK3588 完成真实 PASS/RETAKE 标定并生成现有工程输出格式。
