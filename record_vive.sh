#!/bin/bash

TARGET_DIR="$HOME/Desktop/HRCLAB/vive"
mkdir -p "$TARGET_DIR"
cd "$TARGET_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)


if [ -n "$1" ]; then
  CUSTOM_NAME="$1"
else
  read -p "please insrt the name of the bag: (例如 exp1_walk，直接按 Enter 則預設為 test): " CUSTOM_NAME
fi


if [ -z "$CUSTOM_NAME" ]; then
  CUSTOM_NAME="test"
fi


BAG_NAME="${CUSTOM_NAME}_${TIMESTAMP}"

echo "=================================================="
echo "準備開始錄製 Vive Tracker 及 Locobot 實驗數據..."
echo "儲存資料夾: $TARGET_DIR/$BAG_NAME"
echo "提示: 想要結束錄製時，請按下 Ctrl + C"
echo "=================================================="

ros2 bag record -o "$BAG_NAME" \
  /tracker1/vive_pose_abs \
  /tracker2/vive_pose_abs \
  /tracker3/vive_pose_abs \
  /tracker4/vive_pose_abs \
  /tracker5/vive_pose_abs \
  /tracker6/vive_pose_abs \
  /vive_pose_abs \
  /vive_pose_rel \
  /tf \
  /tf_static \
  /locobot/camera/camera/color/image_raw/compressed \
  /locobot/camera/camera/color/camera_info \
  /locobot/odom \
  /locobot/commands/velocity \
  /locobot/tf \
  /locobot/tf_static