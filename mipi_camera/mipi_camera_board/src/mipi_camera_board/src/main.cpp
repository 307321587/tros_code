//
// Created by lza on 6/16/23.
//

#define STEREO

#include <unistd.h>
#include "sensor.h"
#include "encoder.h"
#include "image_publish.h"
#include <time.h>
#include <atomic>  
#include <thread> 
std::atomic<bool> running(false);
std::atomic<bool> start(true);


int main(int argc, char *argv[])
{
    struct timeval time_now = { 0 };
    struct timeval time_next = { 0 };
    int time_cost,time_ms;
    // 初始化相机 imx 219
    SensorSetting sensor_setting_imx219;
    setImx219(sensor_setting_imx219);
    SensorProcess sensor_imx219(sensor_setting_imx219);
    sensor_imx219.sensorSifDevInit();

    // 初始化相继 imx2192
    SensorSetting sensor_setting_imx219_2;
    setImx219_2(sensor_setting_imx219_2);
    SensorProcess sensor_imx219_2(sensor_setting_imx219_2);
    sensor_imx219_2.sensorSifDevInit();


    // 初始化编码模块        
    Encoder jpeg_encoder_imx219(1280,720);
    Encoder jpeg_encoder_imx219_2(1280,720);
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
    cv::Mat resized_image1,resized_image2;

    while (rclcpp::ok()){

        printf("try to get img...\n");
        img_imx219=sensor_imx219.getImage();

        // timestamp
        struct timespec current_time;  
        clock_gettime(CLOCK_REALTIME, &current_time);  
        long long timestamp = current_time.tv_sec * 1000LL + current_time.tv_nsec / 1000000LL;  
        char filename[100];  
        char filename_l[100];  
        char filename_r[100];  
        // sprintf(filename, "output_image_%lld.jpg", timestamp); 
        printf("timestamp: %lld \n",timestamp);
        snprintf(filename_l, sizeof(filename), "/app/code_li/stereo/save/left_%lld.jpg", timestamp);  
        snprintf(filename_r, sizeof(filename), "/app/code_li/stereo/save/right_%lld.jpg", timestamp);  



        img_imx219_2=sensor_imx219_2.getImage();  

        int new_height = static_cast<int>(img_imx219.cols / (static_cast<double>(1280) / 720));  
        cv::resize(img_imx219, resized_image1, cv::Size(1280, new_height), cv::INTER_LINEAR);  
        resized_image1 = resized_image1(cv::Rect(0, (new_height - 720) / 2, 1280, 720));  
        cv::resize(img_imx219_2, resized_image2, cv::Size(1920, new_height), cv::INTER_LINEAR);  
        resized_image2 = resized_image2(cv::Rect(0, (new_height - 720) / 2, 1280, 720));  


        // cv::imwrite(filename_l,resized_image1);
        // cv::imwrite(filename_r,resized_image2);

        
        yuv_img_imx219_2=sensor_imx219_2.transRgbToYuv(resized_image2);
        jpeg_encoder_imx219_2.encode(yuv_img_imx219_2,encode_img_2);
        yuv_img_imx219=sensor_imx219.transRgbToYuv(resized_image1);
        jpeg_encoder_imx219.encode(yuv_img_imx219,encode_img_1);

        msg->serialize_image_1=encode_img_1;
        msg->serialize_image_2=encode_img_2;

        pub->send_image(msg);
        // loop_rate.sleep();
    }
    return 0;
}