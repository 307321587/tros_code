//
// Created by lza on 6/14/23.
//

#ifndef BUILD_SENSOR_H
#define BUILD_SENSOR_H



extern "C" {
#include <stdint.h>
#include <hb_sys.h>
#include <hb_mipi_api.h>
#include <hb_vin_api.h>
#include <hb_vio_interface.h>
#include <hb_mode.h>
}

#include <iostream>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>

using namespace cv;

typedef struct
{
    /*定义 sensor   初始化的属性信息 */
    MIPI_SENSOR_INFO_S snsinfo;
    /*定义 mipi 初始化参数信息 */
    MIPI_ATTR_S mipi_attr;
    /*定义 dev 初始化的属性信息 */
    VIN_DEV_ATTR_S devinfo;
    /*定义 pipe 属性信息 */
    VIN_PIPE_ATTR_S pipeinfo;
    /*定义 dis 属性信息 */
    VIN_DIS_ATTR_S disinfo;
    /*定义 ldc 属性信息 */
    VIN_LDC_ATTR_S ldcinfo;
}SensorSetting;

extern SensorSetting multi_sensors[10];
// extern std::shared_ptr<dsr_msgs2::msg::QtImage> msg;
void setF37(SensorSetting& sensor_setting);

class SensorProcess {
public:
    SensorProcess(SensorSetting sensor);
    ~SensorProcess();
    int sensorSifDevInit();
    cv::Mat getImage();
    void print_sensor_info(MIPI_SENSOR_INFO_S *snsinfo);
    cv::Mat transRgbToYuv(cv::Mat rgb_img);


private:
    SensorSetting sensor_;
    bool sensor_init_flag = false;
};

#endif //BUILD_SENSOR_H
