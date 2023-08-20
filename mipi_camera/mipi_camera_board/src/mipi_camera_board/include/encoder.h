//
// Created by lza on 6/17/23.
//

#ifndef BUILD_ENCODER_H
#define BUILD_ENCODER_H

#include <sp_codec.h>
#include <opencv2/opencv.hpp>
#include <qt_image/msg/qt_image.hpp>

#define STREAM_FRAME_SIZE 2097152
class Encoder
{
public:
    Encoder(int width,int height);
    ~Encoder();

    void encode(cv::Mat yuv_img,std::vector<uint8_t>& encode_img);


private:
    void *sp_encoder_;
    char *stream_buffer;
    int data_size;
};

#endif //BUILD_ENCODER_H
