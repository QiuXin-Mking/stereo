# SBS 双目棋盘格标定工具

面向 macOS `UVC Camera 1` 的单设备左右拼接（SBS）双目标定。采集过程自动检测 9×6 棋盘、拒绝模糊/曝光异常/重复样本，并导出 OpenCV Python/C++ 可用的标定与校正文件。

## 最快操作路径

```bash
cd stereo_chessboard_calibrator
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install pytest PyYAML
```

1. 打开 `boards/chessboard_A4_9x6_20mm.svg`，使用 A4 横向、实际大小/100% 打印，禁止适应页面。
2. 用尺检查页面上的 100 mm 标尺，并测量一个实际方格边长。
3. 确认设备索引：

   ```bash
   ffmpeg -f avfoundation -list_devices true -i "" 2>&1 | grep -E "\[[0-9]+\]"
   ```

4. 启动标定，把 `20.05` 替换成实测毫米值：

   ```bash
   ./calibrate --square-mm 20.05
   ```

5. 若设备名解析异常，可显式传入当前索引；若知道最高原生 SBS 尺寸，也可一并锁定：

   ```bash
   ./calibrate --device-index 0 --capture-width 3840 --capture-height 1080 --square-mm 20.05
   ```

采集窗口按键：`P` 暂停/继续，`U` 撤销上一对，`Q` 保存退出。默认自动采集 32 对。标定分辨率必须与产品运行分辨率一致；求解使用未缩放原图，只有预览会缩放。

## 输出

通过后写入 `sessions/<时间>/results/final/`：

- `stereo_calibration.yaml`：OpenCV `FileStorage` 直接读取。
- `stereo_calibration.json`：带矩阵形状和单位的交换格式。
- `stereo_calibration.npz`：NumPy 参数包。
- `rectify_maps.yml.gz`：C++ 可直接加载的定点 remap。
- `rectify_maps.npz`：Python remap。
- `rectification_preview.png`：带水平极线的校正预览。
- `quality_report.json`：RMS、极线误差和离群样本。

若结果未通过门槛，写入 `results/diagnostic/` 并显示 `RETAKE`，不会把它冒充正式结果。

## C++ 验证

```bash
cmake -S examples/cpp -B examples/cpp/build
cmake --build examples/cpp/build
examples/cpp/build/load_calibration \
  sessions/<时间>/results/final/stereo_calibration.yaml \
  sessions/<时间>/results/final/rectify_maps.yml.gz \
  sessions/<时间>/raw_sbs/0000_sbs.png
```

输出 `rectified_cpp.png`。

## macOS 常见问题

- 找不到/打不开相机：到“系统设置 → 隐私与安全性 → 相机”，允许 Terminal 或 Codex 访问。
- 索引变化：优先按名称解析；必要时重新运行 FFmpeg 枚举并传 `--device-index`。
- 左右反了：添加 `--swap-eyes` 重新采集。
- 长期检测不到：确认是 9×6 内角点、棋盘完整出现在左右眼、无反光且打印纸保持平整。

