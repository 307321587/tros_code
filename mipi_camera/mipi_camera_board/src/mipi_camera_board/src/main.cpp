//
// Created by lza on 6/16/23.
//

// #define STEREO

#include <unistd.h>
#include "sensor.h"
#include "encoder.h"
#include "image_publish.h"

int main(int argc, char *argv[])
{
    // 初始化相机 imx 219
    SensorSetting sensor_setting_imx219;
    setImx219(sensor_setting_imx219);
    SensorProcess sensor_imx219(sensor_setting_imx219);
    sensor_imx219.sensorSifDevInit();
#ifdef STEREO
    // 初始化相继 imx2192
    SensorSetting sensor_setting_imx219_2;
    setImx219_2(sensor_setting_imx219_2);
    SensorProcess sensor_imx219_2(sensor_setting_imx219_2);
    sensor_imx219_2.sensorSifDevInit();
#endif

    // 初始化编码模块        
    Encoder jpeg_encoder_imx219(1920,1080);
#ifdef STEREO
    Encoder jpeg_encoder_imx219_2(sensor_imx219_2.getSensorWidth(),sensor_imx219_2.getSensorHeight());
#endif
    // 初始化rcl
    std::shared_ptr<MinimalPublisher> pub;
    std::shared_ptr<qt_image::msg::QtImage> msg;
    rclcpp::init(argc,argv);
    pub = std::make_shared<MinimalPublisher>();
    msg = std::make_shared<qt_image::msg::QtImage>();

    rclcpp::WallRate loop_rate(30); //频率，单位Hz
    cv::Mat img_imx219_2,img_imx219,yuv_img_imx219_2,yuv_img_imx219;
    std::vector<uint8_t> encode_img_1;
    std::vector<uint8_t> encode_img_2;
    while (rclcpp::ok()){

        printf("try to get img...\n");
        img_imx219=sensor_imx219.getImage();

   
        cv::imwrite("test.jpg",img_imx219);

#ifdef STEREO
        img_imx219_2=sensor_imx219_2.getImage();    
        cv::Mat ir_image;
        cv::cvtColor(img_imx219,ir_image,cv::COLOR_BGR2GRAY);
        cv::imwrite("ir_image.jpg",ir_image);

        
        yuv_img_imx219_2=sensor_imx219_2.transRgbToYuv(img_imx219_2);
        jpeg_encoder_imx219_2.encode(yuv_img_imx219_2,encode_img_2);
#endif
        yuv_img_imx219=sensor_imx219.transRgbToYuv(img_imx219);
        jpeg_encoder_imx219.encode(yuv_img_imx219,encode_img_1);

        msg->serialize_image_1=encode_img_1;
#ifdef STEREO
        msg->serialize_image_2=encode_img_2;
#endif


        pub->send_image(msg);
        loop_rate.sleep();
    }
    return 0;
}