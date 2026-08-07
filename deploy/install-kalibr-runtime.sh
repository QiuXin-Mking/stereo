#!/bin/sh
set -eu

KALIBR_COMMIT=1f60227442d25e36365ef5f72cd80b9666d73467
IMAGE="stereo-kalibr-rk3588:$KALIBR_COMMIT"
ARCHIVE=${1:-}
CHECKSUM=${2:-"${ARCHIVE}.sha256"}
MINIMUM_FREE_KIB=4194304

if [ -z "$ARCHIVE" ]; then
  echo "用法: $0 <stereo-kalibr-arm64.tar.zst> [sha256 文件]" >&2
  exit 2
fi
if [ "$(uname -m)" != "aarch64" ]; then
  echo "错误：只能在 RK3588 aarch64 主机上安装" >&2
  exit 1
fi
if [ ! -f "$ARCHIVE" ] || [ ! -f "$CHECKSUM" ]; then
  echo "错误：缺少镜像归档或 SHA-256 文件" >&2
  exit 1
fi

available_kib=$(df -Pk /var/lib/docker 2>/dev/null | awk 'NR==2 {print $4}')
if [ -z "$available_kib" ]; then
  available_kib=$(df -Pk / | awk 'NR==2 {print $4}')
fi
if [ "$available_kib" -lt "$MINIMUM_FREE_KIB" ]; then
  echo "错误：Docker 所在磁盘可用空间少于 4 GiB" >&2
  exit 1
fi

checksum_dir=$(CDPATH= cd -- "$(dirname -- "$CHECKSUM")" && pwd)
checksum_name=$(basename -- "$CHECKSUM")
(cd "$checksum_dir" && sha256sum -c "$checksum_name")

zstd -dc "$ARCHIVE" | docker load

docker image inspect "$IMAGE" --format '{{.Architecture}}' | grep -qx arm64
verify_help() {
  command_name=$1
  set +e
  docker run --rm --platform linux/arm64 --entrypoint /bin/bash "$IMAGE" \
    -lc ". /opt/ros/noetic/setup.sh && . /opt/kalibr/install/setup.sh && $command_name --help >/dev/null"
  status=$?
  set -e
  if [ "$status" -ne 0 ] && [ "$status" -ne 2 ]; then
    echo "错误：${command_name} --help 失败（${status}）" >&2
    exit "$status"
  fi
}

verify_help kalibr_calibrate_cameras
verify_help kalibr_calibrate_imu_camera

echo "Kalibr ARM64 运行时安装并验证通过：$IMAGE"
