# RK3588 最小 Kalibr 运行环境

该镜像只为离线双目和相机–IMU标定提供 Kalibr、rosbag、ROS 消息类型及数值依赖。它不包含机器人桌面、仿真、导航或可视化组件，运行时不需要 ROS Master。

构建固定提交：

```sh
KALIBR_COMMIT=1f60227442d25e36365ef5f72cd80b9666d73467 \
  ./kalibr_runtime/build-arm64.sh
```

脚本会验证 ARM64 架构、两个 Kalibr 命令及 2GiB 大小上限。设置 `EXPORT_PATH=/path/image.tar.zst` 可导出供 RK3588 执行 `docker load`。

