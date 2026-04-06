#!/bin/bash
set -e

echo "=== [1/5] Source ROS 2 Jazzy ==="
source /opt/ros/jazzy/setup.bash

echo "=== [2/5] Cai dependencies ==="
sudo apt update && sudo apt install -y \
  libyaml-cpp-dev libfmt-dev libboost-all-dev libtbb-dev \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
  ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-eigen \
  python3-colcon-common-extensions

echo "=== [3/5] Tao workspace ==="
WS=~/ros2_ws_limo
mkdir -p $WS/src

echo "=== [4/5] Clone & build ==="
cd $WS/src
if [ -d "LIMOncello" ]; then
  echo "LIMOncello da ton tai, pull..."
  cd LIMOncello
  git pull
  git submodule update --init --recursive
  cd ..
else
  git clone --recurse-submodules https://github.com/cooothaa-dot/LIMOncello.git
fi

# Copy livox stub packages ra src level (colcon khong recurse vao trong package)
cp -r LIMOncello/livox_interfaces . 2>/dev/null || true
cp -r LIMOncello/livox_ros_driver . 2>/dev/null || true
cp -r LIMOncello/livox_ros_driver2 . 2>/dev/null || true

cd $WS
source /opt/ros/jazzy/setup.bash

# Build livox stubs truoc (limoncello phu thuoc vao chung)
colcon build --symlink-install \
  --allow-overriding livox_interfaces livox_ros_driver livox_ros_driver2 \
  --packages-select livox_interfaces livox_ros_driver livox_ros_driver2

# Source stubs roi moi build limoncello
source $WS/install/setup.bash
colcon build --symlink-install --packages-select limoncello

echo "=== [5/5] Verify ==="
source $WS/install/setup.bash
ros2 launch limoncello limoncello.launch.py --show-args

echo ""
echo "=== Setup hoan tat! ==="
echo "Chay dataset:"
echo "  source ~/ros2_ws_limo/install/setup.bash"
echo "  ros2 launch limoncello limoncello.launch.py config_name:=<config>"
