cmake_minimum_required(VERSION 3.5)
project(mipi_camera_board)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# #设置自定义消息
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/QtImage.msg"
# )
ament_export_dependencies(rosidl_default_runtime)

find_package(rclcpp REQUIRED)
find_package(qt_image REQUIRED)
# 设置包根目录
set(HOST_PACKAGE_DIR /home/lithcado/workspace/sysroot_docker/)
link_directories(
  ${HOST_PACKAGE_DIR}/lib
  ${HOST_PACKAGE_DIR}/usr_x3/lib
  ${HOST_PACKAGE_DIR}/lib/hbmedia
  ${HOST_PACKAGE_DIR}/lib/hbbpu
  ${HOST_PACKAGE_DIR}/lib/modules
  ${HOST_PACKAGE_DIR}/lib/firmware
  ${HOST_PACKAGE_DIR}/lib/pkgconfig
  ${HOST_PACKAGE_DIR}/lib/sensorlib
  ${HOST_PACKAGE_DIR}/lib/udev
  ${PROJECT_SOURCE_DIR}/lib
)

set(X3_HAPI_LIB cam vio hbmedia gdcbin isp iar isp_algo tinyalsa multimedia z avformat avcodec avutil dl ion alog diag cjson hbmem rt stdc++ m pthread opencv_world cnn_intf hbrt_bernoulli_aarch64 dnn hbspdev spcdev)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${HOST_PACKAGE_DIR}/usr_x3/include
  ${HOST_PACKAGE_DIR}/usr_x3/include/vio
  ${HOST_PACKAGE_DIR}/usr_x3/include/hbmedia
)

file(GLOB SRC_LIST "src/*.cpp")
file(GLOB Config_LIST "sensor/*.cpp")

add_executable(mipi_camera ${Config_LIST} ${SRC_LIST})
target_link_libraries(mipi_camera ${X3_HAPI_LIB})
ament_target_dependencies(mipi_camera rclcpp qt_image)
# rosidl_target_interfaces(mipi_camera ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
mipi_camera
  DESTINATION lib/${PROJECT_NAME})

ament_package()