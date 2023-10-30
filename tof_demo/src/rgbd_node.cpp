// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#define SAVE

#include "rgbd_node/rgbd_node.hpp"

#include <sstream>
// #include <std_srvs/srv/Empty.h>

#include <string>
#include <memory>
#include <stdarg.h>
#include <time.h>
#ifdef SAVE
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

extern "C" int ROS_printf(int nLev, char *fmt, ...)
{
  char buf[512] = { 0 };
  va_list args;
  va_start(args, fmt);
  vsprintf(buf, fmt, args);
  switch (nLev)
  {
  case 0:
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", buf);
    break;
  case 1:
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "%s", buf);
    break;
  case 2:
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", buf);
    break;
  default:
    break;
  }

  va_end(args);
  return 0;
}
#define BUF_PUB_NUM 5

namespace rgbd_node
{
RgbdNode::RgbdNode(const rclcpp::NodeOptions & node_options, std::string node_name)
:m_bIsInit(0) ,
  Node(node_name, node_options),
  img_dep_(new sensor_msgs::msg::Image()),
  img_infra_(new sensor_msgs::msg::Image()),
  img_clr_(new sensor_msgs::msg::Image()),
  camera_calibration_info_(new sensor_msgs::msg::CameraInfo())
{
  stop_ = false;
  get_params();
  init();  //外部可能会调用了
  RCLCPP_WARN(rclcpp::get_logger("rgbd_node"),
    "[%s]->mipinode init sucess.\n", __func__);
}

RgbdNode::~RgbdNode()
{
  stop_ = true;
  RCLCPP_WARN(rclcpp::get_logger("rgbd_node"), "shutting down");
  // rgbdCam_.shutdown();
  if (m_spThrdPub) {
    m_spThrdPub->join();
    m_spThrdPub = nullptr;
  }
}

void RgbdNode::get_params()
{
  declare_parameter("sensor_type", "CP3AM");
  declare_parameter("io_method", "ros");
  declare_parameter("enable_color", _enable_clr);
  declare_parameter("enable_depth", _enable_dep);
  declare_parameter("enable_pointcloud", _enable_pcl);
  declare_parameter("enable_aligned_pointcloud", _enable_rgb_pcl);
  declare_parameter("enable_infra", _enable_infra);
  declare_parameter("camera_calibration_file_path", camera_calibration_file_path_);

  this->get_parameter("sensor_type", _sensor_type);
  this->get_parameter("io_method", _io_mode);
  this->get_parameter("camera_calibration_file_path", camera_calibration_file_path_);

  this->get_parameter_or("color_width", clr_w_, 1920);
  this->get_parameter_or("color_height", clr_h_, 1080);
  this->get_parameter_or("color_fps", clr_fps_, 10);
  this->get_parameter_or("enable_color", _enable_clr, false);

  this->get_parameter_or("depth_width", dep_w_, 224);
  this->get_parameter_or("depth_height", dep_h_, 108);
  this->get_parameter_or("depth_fps", dep_fps_, 10);
  this->get_parameter_or("enable_depth", _enable_dep, true);
  this->get_parameter_or("enable_pointcloud", _enable_pcl, true);
  this->get_parameter_or("enable_aligned_pointcloud", _enable_rgb_pcl, false);

  this->get_parameter_or("infra_width", infra_w_, 224);
  this->get_parameter_or("infra_height", infra_h_, 108);
  this->get_parameter_or("infra_fps", infra_fps_, 10);
  this->get_parameter_or("enable_infra", _enable_infra, true);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "color %d, enable_depth %d, enable_pointcloud %d, enable_aligned_pointcloud %d, enable_infra %d",
    _enable_clr, _enable_dep, _enable_pcl, _enable_rgb_pcl, _enable_infra);
}

void RgbdNode::service_capture(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void) request_header;
  if (request->data) {
    // rgbdCam_.stop_capturing();
    ShyCam::GetInstance()->StopStream();
    response->message = "Stop Capturing";
  }
}
void RgbdNode::GetCaptureHdl(struct TCapFrame *frame, void *user_args)
{
  RgbdNode* pThis = reinterpret_cast<RgbdNode*>(user_args);
}

void RgbdNode::init()
{
  if (m_bIsInit)
    return;
  int nRet = -1;
  // img_->header.frame_id = frame_id_;
  img_dep_->header.frame_id = "depth";
  img_infra_->header.frame_id = "infra";
  img_clr_->header.frame_id = "color";
  camera_calibration_info_->header.frame_id = "color";
  // 启动cam 读取并且计算
  nRet = ShyCam::GetInstance()->InitVideo();
  nRet = ShyCam::GetInstance()->StartStream(GetCaptureHdl, this);

  if (!ShyCam::GetInstance()->ReadCalibrationFile(*camera_calibration_info_, camera_calibration_file_path_))
  {
    _enabled_read_cam_calibration = false;
    RCLCPP_WARN(rclcpp::get_logger("rgbd_node"), "get camera calibration parameters failed");
  }
  // 创建线程，读取rgbd 数据，准备发送
  m_spThrdPub = std::make_shared<std::thread>(std::bind(&RgbdNode::exec_loopPub, this));
  m_bIsInit = 1;
}
void RgbdNode::exec_loopPub()
{
  // 创建各个publish
  bool bSharedMem = false;
  char tsTopicName[128] = { 0 };
  if (0 != _io_mode.compare("shared_mem")) {
    if (_enable_dep) {
      snprintf(tsTopicName, sizeof(tsTopicName), "/rgbd_%s/depth/image_rect_raw", _sensor_type.c_str());
      imgDep_pub_ = this->create_publisher<sensor_msgs::msg::Image>(tsTopicName, BUF_PUB_NUM);
    }
    snprintf(tsTopicName, sizeof(tsTopicName), "/rgbd_%s/depth/camera_info", _sensor_type.c_str());
    depCam_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(tsTopicName, 1);
    if (_enable_pcl) {
      snprintf(tsTopicName, sizeof(tsTopicName), "/rgbd_%s/depth/color/points", _sensor_type.c_str());
      img_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(tsTopicName, 1);
    }
    if (_enable_rgb_pcl) {
      snprintf(tsTopicName, sizeof(tsTopicName), "/rgbd_%s/aligned_depth_to_color/color/points", _sensor_type.c_str());
      img_pcl_align_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(tsTopicName, 1);
    }
    if (_enable_infra) {
      snprintf(tsTopicName, sizeof(tsTopicName), "/rgbd_%s/infra/image_rect_raw", _sensor_type.c_str());
      imgInfra_pub_ = this->create_publisher<sensor_msgs::msg::Image>(tsTopicName, BUF_PUB_NUM);
    }
    if (_enable_clr) {
      snprintf(tsTopicName, sizeof(tsTopicName), "/rgbd_%s/color/image_rect_raw", _sensor_type.c_str());
      imgClr_pub_ = this->create_publisher<sensor_msgs::msg::Image>(tsTopicName, BUF_PUB_NUM);
    }
  } else {
#ifdef USING_HBMEM
    // 创建hbmempub
    if (_enable_clr) {
      pub_hbmem1080P_ = this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
        "hbmem_img", BUF_PUB_NUM);
    }
    if (_enable_dep) {
      pub_hbmemdepth_ = this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg480P>(
        "hbmem_depth", BUF_PUB_NUM);
    }
    if (_enable_infra) {
      pub_hbmeminfra_ = this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg480P>(
        "hbmem_infra", BUF_PUB_NUM);
    }
#endif
    bSharedMem = true;
  }
  if (_enabled_read_cam_calibration) {
    snprintf(tsTopicName, sizeof(tsTopicName), "/rgbd_%s/color/camera_info", _sensor_type.c_str());
    imgCam_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(tsTopicName, BUF_PUB_NUM);
  }
  while (!stop_) {
    if (bSharedMem) {
      timer_hbmem_pub();
    } else {
      timer_ros_pub();
    }
    usleep(10000);
  } 
}

void RgbdNode::pub_align_pcl(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPublish,
  TTofRgb_PCDClr &pclRgbDepth, struct timespec time_start)
{
  sensor_msgs::msg::PointCloud2 msg_pointcloud;
  msg_pointcloud.header.stamp.sec = time_start.tv_sec;
  msg_pointcloud.header.stamp.nanosec = time_start.tv_nsec;
  msg_pointcloud.header.frame_id = img_dep_->header.frame_id;  // _optical_frame_id[DEPTH];
  msg_pointcloud.width = pclRgbDepth.nWidth;
  msg_pointcloud.height = pclRgbDepth.nHeight;
  msg_pointcloud.is_dense = true;
  int nPtSz = msg_pointcloud.width * msg_pointcloud.height;

  sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

  modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pointcloud, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg_pointcloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg_pointcloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg_pointcloud, "b");

  for (int nIdx = 0; nIdx < nPtSz; ++nIdx) {
      *iter_x = pclRgbDepth.pData[nIdx].x;
      *iter_y = pclRgbDepth.pData[nIdx].y;
      *iter_z = pclRgbDepth.pData[nIdx].z;

      *iter_r = pclRgbDepth.pData[nIdx].r;
      *iter_g = pclRgbDepth.pData[nIdx].g;
      *iter_b = pclRgbDepth.pData[nIdx].b;
      ++iter_x; ++iter_y; ++iter_z;
      ++iter_r; ++iter_g; ++iter_b;
  }
  pclPublish->publish(msg_pointcloud);
}

void RgbdNode::pub_ori_pcl(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPublish,
  TofDepth_Info &pclDepth, struct timespec time_start)
{
  sensor_msgs::msg::PointCloud2 msg_pointcloud;
  msg_pointcloud.header.stamp.sec = time_start.tv_sec;
  msg_pointcloud.header.stamp.nanosec = time_start.tv_nsec;
  msg_pointcloud.header.frame_id = img_dep_->header.frame_id;  // _optical_frame_id[DEPTH];
  msg_pointcloud.width = pclDepth.frameWidth * pclDepth.frameHeight;
  msg_pointcloud.height = 1;  // pclDepth.frameHeight;
  // msg_pointcloud.is_bigendian = false;
  msg_pointcloud.is_dense = true;
  int nPtSz = msg_pointcloud.width * msg_pointcloud.height;

  sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

  modifier.setPointCloud2Fields(3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pointcloud, "z");
  int nIdx = 0;
  for (; nIdx < nPtSz; ++nIdx) {
      *iter_x = pclDepth.pfPointData[nIdx].x;
      *iter_y = pclDepth.pfPointData[nIdx].y;
      *iter_z = pclDepth.pfPointData[nIdx].z;
      /*if (nIdx < 1000) {
        std::cout <<" " << nIdx 
          <<" x:" << *iter_x
          <<" y:" << *iter_y
          <<" z:" << *iter_z
          << std::endl;
      }*/
      ++iter_x; ++iter_y; ++iter_z;
  }
  pclPublish->publish(msg_pointcloud);
  RCLCPP_INFO(rclcpp::get_logger("rgbd_node"), "[%s]->pub pcl w:h=%d:%d,nIdx-%d:sz=%d.",
    __func__, msg_pointcloud.width, msg_pointcloud.height, nIdx, nPtSz);
}

void RgbdNode::timer_ros_pub()
{
  if (ShyCam::GetInstance()->is_capturing()) {
    TShyFrame ImgDepth;
    if (ShyCam::GetInstance()->GetDepthFrame(ImgDepth)) {
      TTofRgbResult oResTofPCL = {0};
      // 改成取rgb 的数据
      if (0 == ShyCam::GetInstance()->CalcTofSync(&oResTofPCL)) {
        // 获取rgb 图，灰度图，点云，深度图
        struct timespec time_start;
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        int nDepthHeight = 129;
        int nDepthSz = ImgDepth.width * nDepthHeight *2;
        if (_enable_dep) {
          img_dep_->header.stamp.sec = time_start.tv_sec;
          img_dep_->header.stamp.nanosec = time_start.tv_nsec;
          img_dep_->width = oResTofPCL.mOriRes.frameWidth;
          img_dep_->height = oResTofPCL.mOriRes.frameHeight;
          img_dep_->step = oResTofPCL.mOriRes.frameWidth;
          img_dep_->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
          img_dep_->data.resize(oResTofPCL.mOriRes.frameWidth *
                                oResTofPCL.mOriRes.frameHeight * 2);
          std::vector<uint16_t> depth_data;
          //auto float_data = oResTofPCL.mPclRgb.pData;
          auto float_data = oResTofPCL.mOriRes.pfPointData;
          for (int i = 0; i < oResTofPCL.mOriRes.frameWidth *
                                  oResTofPCL.mOriRes.frameHeight;
               i++) {
            //depth_data.push_back(
            //    static_cast<uint16_t>(std::round(float_data->z * 1000.0f)));
            depth_data.push_back(
                static_cast<uint16_t>(std::round(float_data->z * 1000.0f)));
            float_data++;
          }
          memcpy(&img_dep_->data[0], depth_data.data(),
                 oResTofPCL.mOriRes.frameWidth *
                     oResTofPCL.mOriRes.frameHeight * 2);
#ifdef SAVE
        char filename_2[100];  
        char filename_depth[100];  
        printf("gray_timestamp: %ld\n",oResTofPCL.mOriRes.timeStamp/1000L);
        snprintf(filename_depth, sizeof(filename_2), "/app/code_li/tof/save/depth_%ld.png", oResTofPCL.mOriRes.timeStamp/1000L);  
        cv::Mat depthImg = cv::Mat(img_dep_->height, img_dep_->width, CV_16UC1);  
        std::memcpy(depthImg.data, depth_data.data(), img_dep_->height*img_dep_->width * sizeof(uint16_t));  
        cv::imwrite(filename_depth,depthImg);
#endif
          imgDep_pub_->publish(*img_dep_);
        }
        if (_enable_infra) {
          img_infra_->header.stamp = img_dep_->header.stamp;
          img_infra_->width = oResTofPCL.mOriRes.frameWidth;  // ImgDepth.width;
          img_infra_->height = oResTofPCL.mOriRes.frameHeight;
          img_infra_->step = oResTofPCL.mOriRes.frameWidth;
          img_infra_->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
          img_infra_->data.resize(oResTofPCL.mOriRes.frameWidth * oResTofPCL.mOriRes.frameHeight);
          memcpy(&img_infra_->data[0], oResTofPCL.mOriRes.pU8Graydata, img_infra_->data.size());

#ifdef SAVE
        char filename[100];  
        char filename_gray[100];  
        printf("gray_timestamp: %ld\n",oResTofPCL.mOriRes.timeStamp/1000L);
        snprintf(filename_gray, sizeof(filename), "/app/code_li/tof/save/gray_%ld.png", oResTofPCL.mOriRes.timeStamp/1000L);  
        cv::Mat depthGray = cv::Mat(img_infra_->height, img_infra_->width, CV_8UC1, (void *)(oResTofPCL.mOriRes.pU8Graydata), cv::Mat::AUTO_STEP);
        cv::imwrite(filename_gray,depthGray);
#endif
          imgInfra_pub_->publish(*img_infra_);
        }

        if (_enable_clr) {
          TShyFrame ImgYuv;
          ShyCam::GetInstance()->GetClrFrame(ImgYuv, 0);
          img_clr_->encoding = sensor_msgs::image_encodings::BGR8;  // "bgr8";
          img_clr_->header.stamp = img_dep_->header.stamp;
          img_clr_->width = ImgYuv.width;
          img_clr_->height = ImgYuv.height;
          img_clr_->step = ImgYuv.width;
          img_clr_->data.resize(ImgYuv.size * 2);
          memcpy(&img_clr_->data[0], oResTofPCL.mOutRgb, img_clr_->data.size());
          imgClr_pub_->publish(*img_clr_);
        }
        if (_enabled_read_cam_calibration) {
          camera_calibration_info_->header.stamp = img_dep_->header.stamp;
          imgCam_pub_->publish(*camera_calibration_info_);
          RCLCPP_INFO(rclcpp::get_logger("rgbd_node"),"publish camera info.\n");
        }
        // pub_CamInfo(depCam_pub_,);
        if (_enable_pcl)
          pub_ori_pcl(img_pcl_pub_, oResTofPCL.mOriRes, time_start);
        if (_enable_rgb_pcl)
          pub_align_pcl(img_pcl_align_pub_, oResTofPCL.mPclRgb, time_start);
        // pub_pcl(img_pcl_align_pub_);

        // m_oTofPcl->CalcShyTofSync(oResTofPCL, m_oResTofPCL.pnt_cloud, m_oResTofPCL.ori_pnt_cloud);
        // 直接获取 rgb，pt，等各个数据，然后 pub
        RCLCPP_INFO(rclcpp::get_logger("rgbd_node"), "[%s]->pub dep w:h=%d:%d,sz=%d, infra w:h=%d:%d, sz=%d.",
          __func__, img_dep_->width, img_dep_->height, ImgDepth.size,
          img_infra_->width, img_infra_->height, img_infra_->data.size());
        ReleaseTofResult(&oResTofPCL);
      }
      ShyCam::GetInstance()->ReleaseDepthFrame();
      ShyCam::GetInstance()->ReleaseClrFrame();
      ShyCam::GetInstance()->ReleasePair();
    }
  }
}
void RgbdNode::timer_hbmem_pub()
{
#ifdef USING_HBMEM
  if (ShyCam::GetInstance()->is_capturing()) {
    TShyFrame ImgDepth;
    if (ShyCam::GetInstance()->GetDepthFrame(ImgDepth)) {
      TTofRgbResult oResTofPCL = {0};
      // 改成取rgb 的数据
      if (0 == ShyCam::GetInstance()->CalcTofSync(&oResTofPCL)) {
        // 获取rgb 图，灰度图，点云，深度图
        ++mSendIdx;
        struct timespec time_start;
        clock_gettime(CLOCK_MONOTONIC, &time_start);

        int nDepthHeight = 129;
        int nDepthSz = ImgDepth.width * nDepthHeight *2;
        if (_enable_dep) {
          auto loanedepthMsg = pub_hbmemdepth_->borrow_loaned_message();
          if (loanedepthMsg.is_valid()) {
            auto& msg = loanedepthMsg.get();
            msg.index = mSendIdx;
            memcpy(msg.encoding.data(), sensor_msgs::image_encodings::TYPE_16UC1,
              strlen(sensor_msgs::image_encodings::TYPE_16UC1));
            msg.time_stamp.sec = time_start.tv_sec;
            msg.time_stamp.nanosec = time_start.tv_nsec;
            msg.width = ImgDepth.width;
            msg.height = nDepthHeight;
            msg.step = ImgDepth.width;
            msg.data_size = nDepthSz;
            memcpy(msg.data.data(), ImgDepth.pucImageData + nDepthSz *9, nDepthSz);
            pub_hbmemdepth_->publish(std::move(loanedepthMsg));
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rgbd_node"), "depth borrow_loaned_message failed");
          }
        }

        if (_enable_infra) {
          auto loaned480Msg = pub_hbmeminfra_->borrow_loaned_message();
          if (loaned480Msg.is_valid()) {
            auto& msg = loaned480Msg.get();
            msg.index = mSendIdx;
            memcpy(msg.encoding.data(), sensor_msgs::image_encodings::TYPE_8UC1,
              strlen(sensor_msgs::image_encodings::TYPE_8UC1));
            msg.time_stamp.sec = time_start.tv_sec;
            msg.time_stamp.nanosec = time_start.tv_nsec;
            msg.width = oResTofPCL.mOriRes.frameWidth;
            msg.height = oResTofPCL.mOriRes.frameHeight;
            msg.step = oResTofPCL.mOriRes.frameWidth;
            msg.data_size = msg.width * msg.height;
            memcpy(msg.data.data(), oResTofPCL.mOriRes.pU8Graydata, msg.data_size);
            pub_hbmeminfra_->publish(std::move(loaned480Msg));
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rgbd_node"), "480p borrow_loaned_message failed");
          }
        }

        if (_enable_clr) {
          auto loaned1080Msg = pub_hbmem1080P_->borrow_loaned_message();
          if (loaned1080Msg.is_valid()) {
            auto& msg = loaned1080Msg.get();
            TShyFrame ImgYuv;
            ShyCam::GetInstance()->GetClrFrame(ImgYuv, 0);
            msg.index = mSendIdx;
            memcpy(msg.encoding.data(), sensor_msgs::image_encodings::BGR8,
              strlen(sensor_msgs::image_encodings::BGR8));
            msg.time_stamp.sec = time_start.tv_sec;
            msg.time_stamp.nanosec = time_start.tv_nsec;
            msg.width = ImgYuv.width;
            msg.height = ImgYuv.height;
            msg.step = ImgYuv.width;
            msg.data_size = ImgYuv.size * 2;
            memcpy(msg.data.data(), oResTofPCL.mOutRgb, msg.data_size);
            pub_hbmem1080P_->publish(std::move(loaned1080Msg));
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rgbd_node"), "1080p borrow_loaned_message failed");
          }
        }
        if (_enabled_read_cam_calibration ) {
          camera_calibration_info_->header.stamp.sec = time_start.tv_sec;
          camera_calibration_info_->header.stamp.nanosec = time_start.tv_nsec;
          imgCam_pub_->publish(*camera_calibration_info_);
          RCLCPP_INFO(rclcpp::get_logger("rgbd_node"),"publish camera info.\n");
        }
        // pub_ori_pcl(img_pcl_pub_, oResTofPCL.mOriRes, time_start);
        // pub_align_pcl(img_pcl_align_pub_, oResTofPCL.mPclRgb, time_start);
        // pub_pcl(img_pcl_align_pub_);

        // m_oTofPcl->CalcShyTofSync(oResTofPCL, m_oResTofPCL.pnt_cloud, m_oResTofPCL.ori_pnt_cloud);
        // 直接获取 rgb，pt，等各个数据，然后 pub
        RCLCPP_INFO(rclcpp::get_logger("rgbd_node"), "[%s]->pub dep w:h=%d:%d,sz=%d, infra w:h=%d:%d, sz=%d.",
          __func__, img_dep_->width, img_dep_->height, ImgDepth.size,
          img_infra_->width, img_infra_->height, img_infra_->data.size());
        ReleaseTofResult(&oResTofPCL);
      }
      ShyCam::GetInstance()->ReleaseDepthFrame();
      ShyCam::GetInstance()->ReleaseClrFrame();
      ShyCam::GetInstance()->ReleasePair();
    }
  }
#endif
}
}  // namespace rgbd_node
