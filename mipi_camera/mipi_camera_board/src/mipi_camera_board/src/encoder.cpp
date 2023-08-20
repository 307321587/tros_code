//
// Created by lza on 6/17/23.
//
#include "encoder.h"

Encoder::Encoder(int width, int height)
{
    sp_encoder_ = sp_init_encoder_module();
    stream_buffer = (char*)std::malloc(sizeof(char) * STREAM_FRAME_SIZE);
    int ret = sp_start_encode(sp_encoder_, 0, SP_ENCODER_MJPEG, width, height, 8000);
    data_size=width*height*3/2;
}

Encoder::~Encoder()
{
    sp_release_encoder_module(sp_encoder_);
    sp_stop_encode(sp_encoder_);
    free(stream_buffer);
}

void Encoder::encode(cv::Mat yuv_img,std::vector<uint8_t>& encode_img)
{

    sp_encoder_set_frame(sp_encoder_,(char *)yuv_img.data,data_size);
    memset(stream_buffer, 0, STREAM_FRAME_SIZE);
    int stream_frame_size = sp_encoder_get_stream(sp_encoder_, stream_buffer);//get
    if (stream_frame_size == -1)
    {
        printf("encoder_get_image error! \n");
    }
    encode_img=std::vector<uint8_t>(stream_buffer, stream_buffer+stream_frame_size);
}

