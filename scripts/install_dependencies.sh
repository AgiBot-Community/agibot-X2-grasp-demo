#!/usr/bin/env bash
set -euo pipefail

ros_distro="${ROS_DISTRO:-humble}"

if [[ ! "${ros_distro}" =~ ^[a-z0-9]+$ ]]; then
  printf 'Invalid ROS_DISTRO: %s\n' "${ros_distro}" >&2
  exit 2
fi

install_system_dependencies() {
  local -a packages=(
    "ros-${ros_distro}-pinocchio"
    libopencv-dev
    python3-numpy
    python3-opencv
  )
  if ! command -v apt-get >/dev/null 2>&1; then
    printf 'apt-get is required to install system dependencies.\n' >&2
    exit 1
  fi

  local -a apt=(apt-get)
  if [[ "${EUID}" -ne 0 ]]; then
    if ! command -v sudo >/dev/null 2>&1; then
      printf 'sudo is required to install system dependencies.\n' >&2
      exit 1
    fi
    apt=(sudo apt-get)
  fi

  printf 'Installing ROS %s and system dependencies...\n' "${ros_distro}"
  "${apt[@]}" update
  "${apt[@]}" install -y "${packages[@]}"
}

install_system_dependencies
