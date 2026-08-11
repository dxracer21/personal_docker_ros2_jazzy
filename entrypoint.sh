#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f "${OPEN_MANIPULATOR_WS:-/root/ros2_ws}/install/setup.bash" ]; then
    source "${OPEN_MANIPULATOR_WS:-/root/ros2_ws}/install/setup.bash"
fi

PERSONAL_WS="${PERSONAL_WS:-/home/jinsoo/ros2_ws}"
cd "${PERSONAL_WS}"

if [ "${AUTO_COLCON_BUILD:-1}" = "1" ] && find src -mindepth 2 -name package.xml -print -quit | grep -q .; then
    echo "[entrypoint] Building ${PERSONAL_WS}"
    colcon build --symlink-install
fi

if [ -f "${PERSONAL_WS}/install/setup.bash" ]; then
    source "${PERSONAL_WS}/install/setup.bash"
fi

exec "$@"
