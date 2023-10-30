#!/bin/bash
source /home/lithcado/workspace/tros_ws/install/setup.bash
# source install/setup.bash
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/opt/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu/bin/$TARGET_TRIPLE-

# colcon build --packages-select mipi_camera_board --merge-install --cmake-force-configure --cmake-args \
# --no-warn-unused-cli -DCMAKE_TOOLCHAIN_FILE=\
# `pwd`/aarch64_toolchainfile.cmake

# colcon build --merge-install --cmake-force-configure --cmake-args \
# --no-warn-unused-cli -DCMAKE_TOOLCHAIN_FILE=\
# `pwd`/aarch64_toolchainfile.cmake 

colcon build --packages-select rgbd_sensor_test  --cmake-args \
--no-warn-unused-cli -DCMAKE_TOOLCHAIN_FILE=\
`pwd`/aarch64_toolchainfile.cmake