#!/bin/bash
source /home/lza/cc_ws/tros_ws/install/setup.bash

export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/opt/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu/bin/$TARGET_TRIPLE-

colcon build --packages-select qt_image --merge-install --cmake-force-configure --cmake-args \
--no-warn-unused-cli -DCMAKE_TOOLCHAIN_FILE=\
`pwd`/aarch64_toolchainfile.cmake

source install/setup.bash

colcon build --packages-select cpp_pubsub --merge-install --cmake-force-configure --cmake-args \
--no-warn-unused-cli -DCMAKE_TOOLCHAIN_FILE=\
`pwd`/aarch64_toolchainfile.cmake

