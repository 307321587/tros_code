//
// Created by lza on 6/16/23.
//

#include <unistd.h>
#include "sensor.h"

int main(int argc, char *argv[])
{
    // camera init
    SensorSetting sensor_setting;
    setF37(sensor_setting);
    SensorProcess sensor(sensor_setting);
    sensor.sensorSifDevInit();
    sleep(5);
    cv::Mat img=sensor.getImage();
    cv::imwrite("test.jpg",img);
    return 0;
}