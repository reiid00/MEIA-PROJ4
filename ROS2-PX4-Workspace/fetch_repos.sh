#!/bin/bash

TARGET_DIR="$(pwd)/src"

if [ ! -d "$TARGET_DIR" ]; then
  mkdir "$TARGET_DIR"
fi

if [ ! -d "${TARGET_DIR}/px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git "${TARGET_DIR}/px4_msgs"
fi

if [ ! -d "${TARGET_DIR}/px4_ros_com" ]; then
    git clone https://github.com/PX4/px4_ros_com.git "${TARGET_DIR}/px4_ros_com"
fi