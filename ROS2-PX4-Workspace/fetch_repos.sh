#!/bin/bash

TARGET_DIR="$(pwd)/src"

if [ ! -d "$TARGET_DIR" ]; then
  mkdir "$TARGET_DIR"
fi

# remove folders


if [ ! -d "${TARGET_DIR}/px4_msgs" ]; then
    rm -rf "${TARGET_DIR}/px4_msgs"
fi


if [ ! -d "${TARGET_DIR}/px4_ros_com" ]; then
    rm -rf "${TARGET_DIR}/px4_ros_com"
fi


git clone https://github.com/PX4/px4_msgs.git "${TARGET_DIR}/px4_msgs"
git clone https://github.com/PX4/px4_ros_com.git "${TARGET_DIR}/px4_ros_com"