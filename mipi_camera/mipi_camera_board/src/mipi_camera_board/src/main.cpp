//
// Created by lza on 6/16/23.
//

#include <unistd.h>
#include "sensor.h"
#include "encoder.h"
#include "image_publish.h"

int main(int argc, char *argv[])
{
    // 初始化相机
    SensorSetting sensor_setting;
    // setF37(sensor_setting);
    setImx219(sensor_setting);
    SensorProcess sensor(sensor_setting);
    sensor.sensorSifDevInit();

    // 初始化编码模块
    Encoder jpeg_encoder(sensor.getSensorWidth(),sensor.getSensorHeight());

    // 初始化rcl
    std::shared_ptr<MinimalPublisher> pub;
    std::shared_ptr<mipi_camera_board::msg::QtImage> msg;
    rclcpp::init(argc,argv);
    pub = std::make_shared<MinimalPublisher>();
    msg = std::make_shared<mipi_camera_board::msg::QtImage>();

    rclcpp::WallRate loop_rate(20); //频率，单位Hz
    cv::Mat img,yuv_img;
    while (rclcpp::ok()){

        printf("try to get img...\n");
        img=sensor.getImage();
        cv::imwrite("test.jpg",img);
        printf("try to trans to yuv...\n");
        yuv_img=sensor.transRgbToYuv(img);
        printf("try to encode...\n");
        jpeg_encoder.encode(yuv_img,msg);
        printf("try to send msg...\n");
        pub->send_image(msg);
        loop_rate.sleep();
    }
    return 0;
}