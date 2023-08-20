#ifndef __IMAGE_PUBLISH_H__
#define __IMAGE_PUBLISH_H__

#include <iostream>
#include <chrono>
#include <stdio.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <qt_image/msg/qt_image.hpp>
using namespace std::chrono_literals;
/*----------------------------------
*   msg:QtImage
*   ------layout------
*   uint32 height,width,channel
*   ----image_data----
*   uint8[] serialize_image
*   ------boxnum------
*   uint32 boxn
*   -----box_data-----
*   uint32[] box_point1,box_point2
*   float32[] box_depth
*   uint8[] box_index
*/
class MinimalPublisher : public rclcpp::Node
{
    public:
    
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0) {
        publisher_ = this->create_publisher<qt_image::msg::QtImage>("qt_image", 10); // CHANGE
    }
    
    int64_t count=0;

    void send_image(std::shared_ptr<qt_image::msg::QtImage> message){
        publisher_->publish(*message);
        printf("msg send completed!\n");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<qt_image::msg::QtImage>::SharedPtr publisher_; // CHANGE
    size_t count_;
    };

#endif