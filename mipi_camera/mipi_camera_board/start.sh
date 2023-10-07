#!/bin/sh
cd /app/code_li/mipi_camera/mipi_camera_board/
source /opt/tros/setup.bash
source install/local_setup.bash

export LD_LIBRARY_PATH=./src/mipi_camera_board/lib:$LD_LIBRARY_PATH

echo 19 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio19/direction
echo 0 > /sys/class/gpio/gpio19/value
sleep 0.2
echo 1 > /sys/class/gpio/gpio19/value
echo 19 > /sys/class/gpio/unexport

# enable sensor mclk
echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en
echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq

echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart
echo 1 > /sys/class/vps/mipi_host2/param/stop_check_instart
# echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart

i2cdetect -r -y 0
i2cdetect -r -y 3

ros2 run mipi_camera_board mipi_camera