#!/bin/sh
set -eu

. /opt/ros/noetic/setup.sh
. /opt/kalibr/install/setup.sh
export MPLBACKEND=Agg

DATASET=/data/kalibr
RESULTS="$DATASET/results"
STAGE_FILE="$DATASET/stage.json"
BAG="$RESULTS/dataset.bag"
IMU_TOPIC=/imu0

mkdir -p "$RESULTS"
grep -Eq "^rostopic:[[:space:]]*$IMU_TOPIC$" "$DATASET/imu.yaml"

write_stage() {
  python3 -c 'import json,sys,time; print(json.dumps({"stage":sys.argv[1],"updated_unix":time.time()}))' "$1" >"$STAGE_FILE.tmp"
  mv "$STAGE_FILE.tmp" "$STAGE_FILE"
}

on_exit() {
  code=$?
  trap - EXIT
  if [ "$code" -ne 0 ]; then
    write_stage error
  fi
  exit "$code"
}
trap on_exit EXIT

write_stage bagging
python3 /opt/kalibr-tools/build_bag.py --dataset "$DATASET" --output "$BAG"

write_stage camera_calibrating
kalibr_calibrate_cameras \
  --bag "$BAG" \
  --target "$DATASET/target.yaml" \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw \
  --dont-show-report >"$RESULTS/cameras.log" 2>&1
cp "$RESULTS/dataset-camchain.yaml" "$RESULTS/camchain.yaml"

write_stage imu_calibrating
kalibr_calibrate_imu_camera \
  --bag "$BAG" \
  --cams "$RESULTS/camchain.yaml" \
  --imu "$DATASET/imu.yaml" \
  --target "$DATASET/target.yaml" \
  --dont-show-report >"$RESULTS/imu-camera.log" 2>&1
cp "$RESULTS/dataset-camchain-imucam.yaml" "$RESULTS/camchain-imucam.yaml"

python3 /opt/kalibr-tools/summarize.py \
  --results "$RESULTS" --output "$RESULTS/summary.json"
write_stage pass
trap - EXIT
