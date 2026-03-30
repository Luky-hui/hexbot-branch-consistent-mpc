#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /etc/os-release ]] || ! grep -q 'VERSION_CODENAME=noble' /etc/os-release; then
  echo "[error] container_setup_env.sh must run inside the Ubuntu 24.04 / ROS 2 Jazzy container." >&2
  echo "[hint] start osrf/ros:jazzy-desktop first, then rerun this script." >&2
  exit 1
fi

if [[ ! -d /opt/ros/jazzy ]]; then
  echo "[error] /opt/ros/jazzy is missing. This shell is not a Jazzy runtime." >&2
  exit 1
fi

if [[ ${EUID} -ne 0 ]]; then
  echo "[error] apt install requires root inside the container. Re-enter the container as root and rerun." >&2
  exit 1
fi

apt update
apt install -y \
  build-essential \
  cmake \
  git \
  python3-dev \
  ros-jazzy-pinocchio \
  ros-jazzy-eigen3-cmake-module \
  ros-jazzy-hpp-fcl \
  ros-jazzy-eigenpy \
  ros-jazzy-grid-map \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  pybind11-dev \
  libeigen3-dev \
  libboost-all-dev \
  libglpk-dev \
  libgmp-dev \
  libmpfr-dev \
  libcgal-dev \
  libopencv-dev \
  libpcl-dev \
  liboctomap-dev \
  liburdfdom-dev \
  python3-colcon-common-extensions \
  python3-rosdep

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "[done] Jazzy container dependencies installed and ROS environment sourced."
echo "[hint] next: bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_build_hexbot_stack.sh"
