#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
KALIBR_COMMIT=${KALIBR_COMMIT:-1f60227442d25e36365ef5f72cd80b9666d73467}
BUILD_JOBS=${BUILD_JOBS:-2}
IMAGE="stereo-kalibr-rk3588:$KALIBR_COMMIT"
SIZE_LIMIT=2147483648

docker info >/dev/null
docker build \
  --platform linux/arm64 \
  --build-arg "KALIBR_COMMIT=$KALIBR_COMMIT" \
  --build-arg "BUILD_JOBS=$BUILD_JOBS" \
  -f "$SCRIPT_DIR/Dockerfile.arm64" \
  -t "$IMAGE" \
  "$PROJECT_DIR"

set -- $(docker image inspect "$IMAGE" --format '{{.Architecture}} {{.Size}}')
architecture=$1
image_size=$2
if [ "$architecture" != "arm64" ]; then
  echo "错误：镜像架构是 $architecture，不是 arm64" >&2
  exit 1
fi
docker history --no-trunc "$IMAGE"
if [ "$image_size" -gt "$SIZE_LIMIT" ]; then
  echo "错误：运行镜像 $image_size bytes，超过 2GiB" >&2
  exit 1
fi

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

echo "镜像验证通过：$IMAGE ($image_size bytes)"
if [ -n "${EXPORT_PATH:-}" ]; then
  docker save "$IMAGE" | zstd -T0 -o "$EXPORT_PATH"
  export_dir=$(CDPATH= cd -- "$(dirname -- "$EXPORT_PATH")" && pwd)
  export_name=$(basename -- "$EXPORT_PATH")
  (cd "$export_dir" && sha256sum "$export_name" >"$export_name.sha256")
  echo "已导出：$EXPORT_PATH"
fi
