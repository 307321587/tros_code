// #ifndef __IMAGE_PUBLISH_H__
// #define __IMAGE_PUBLISH_H__

// #include <iostream>
// #include <chrono>
// #include <stdio.h>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <mipi_camera_board/msg/qt_image.hpp>
// using namespace std::chrono_literals;
// /*----------------------------------
// *   msg:QtImage
// *   ------layout------
// *   uint32 height,width,channel
// *   ----image_data----
// *   uint8[] serialize_image
// *   ------boxnum------
// *   uint32 boxn
// *   -----box_data-----
// *   uint32[] box_point1,box_point2
// *   float32[] box_depth
// *   uint8[] box_index
// */
// class MinimalPublisher : public rclcpp::Node
// {
//     public:
    
//     MinimalPublisher()
//     : Node("minimal_publisher"), count_(0) {
//         publisher_ = this->create_publisher<dsr_msgs2::msg::QtImage>("qt_image", 10); // CHANGE
//     }
    
//     int64_t count=0;

//     void send_image(std::shared_ptr<dsr_msgs2::msg::QtImage> message){
//         publisher_->publish(*message);
//         printf("msg send completed!\n");
//     }

//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<dsr_msgs2::msg::QtImage>::SharedPtr publisher_; // CHANGE
//     size_t count_;
//     };

// #endif