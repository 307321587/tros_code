#include "sensor.h"



SensorProcess::SensorProcess(SensorSetting sensor_setting)
{
    sensor_setting_=sensor_setting;
    height_=sensor_setting_.mipi_attr.mipi_host_cfg.height;
    width_=sensor_setting_.mipi_attr.mipi_host_cfg.width;
}

SensorProcess::~SensorProcess()
{

}

int SensorProcess::sensorSifDevInit()
{
    // system("echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart");
    int ret = 0;
    int devId = sensor_setting_.snsinfo.sensorInfo.dev_port;
    int pipeId=sensor_setting_.snsinfo.sensorInfo.dev_port;
    MIPI_SENSOR_INFO_S snsinfo=sensor_setting_.snsinfo;
    VIN_PIPE_ATTR_S pipeinfo=sensor_setting_.pipeinfo;
    MIPI_ATTR_S  mipi_attr=sensor_setting_.mipi_attr;
    VIN_DEV_ATTR_S devinfo=sensor_setting_.devinfo;
    VIN_DIS_ATTR_S disinfo=sensor_setting_.disinfo;
    VIN_LDC_ATTR_S ldcinfo=sensor_setting_.ldcinfo;

    printf("seridesInx:%d\nseridesPort:%d\n",snsinfo.sensorInfo.deserial_index,snsinfo.sensorInfo.deserial_port);
    HB_MIPI_SensorBindSerdes(&snsinfo, snsinfo.sensorInfo.deserial_index, snsinfo.sensorInfo.deserial_port);
    printf("MipiInx:%d\n",snsinfo.sensorInfo.entry_index);
    HB_MIPI_SensorBindMipi(&snsinfo, snsinfo.sensorInfo.entry_index);
    print_sensor_info(&snsinfo);
    ret = HB_MIPI_InitSensor(devId, &snsinfo);
    if (ret < 0) {
        printf("hb mipi init sensor error!\n");
        return ret;
    }
    printf("hb sensor init success...\n");

    // 初始化mipi，设置mipi host的mipiclk、linelength、framelength、settle等参数
    ret = HB_MIPI_SetMipiAttr(snsinfo.sensorInfo.entry_index, &mipi_attr);
    if (ret < 0) {
        printf("hb mipi set mipi attr error!\n");
        return ret;
    }
    printf("hb mipi init success...\n");

    // 需要设置成offline模式，否则获取不到raw图
    ret = HB_SYS_SetVINVPSMode(pipeId, VIN_OFFLINE_VPS_OFFINE);
    if (ret < 0) {
        printf("HB_SYS_SetVINVPSMode error!\n");
        return ret;
    }

    // 必须要初始化isp来贯通pipeline，即使isp不工作也需要，否则 HB_MIPI_SetMipiAttr 会失败
    ret = HB_VIN_CreatePipe(pipeId, &pipeinfo);  // isp init
    if (ret < 0) {
        printf("HB_VIN_CreatePipe error!\n");
        return ret;
    }

    printf("devId: %d snsinfo.sensorInfo.entry_index:%d\n", devId, snsinfo.sensorInfo.entry_index);
    ret = HB_VIN_SetMipiBindDev(devId, snsinfo.sensorInfo.entry_index); /* mipi和vin(sif) dev 绑定 */
    if (ret < 0) {
        printf("HB_VIN_SetMipiBindDev error!\n");
        return ret;
    }
    printf("camera_info->mipi_attr.mipi_host_cfg.channel_num: %d\n", mipi_attr.mipi_host_cfg.channel_num);
    ret = HB_VIN_SetDevVCNumber(devId, mipi_attr.mipi_host_cfg.channel_sel[0]); /* 确定使用哪几个虚拟通道作为mipi的输入 */
    if (ret < 0) {
        printf("HB_VIN_SetDevVCNumber error!\n");
        return ret;
    }

    ret = HB_VIN_SetDevAttr(devId, &devinfo);  // sif init
    if (ret < 0) {
        printf("HB_VIN_SetDevAttr error!\n");
        return ret;
    }
    ret = HB_VIN_SetPipeAttr(pipeId, &pipeinfo);  // isp init
    if (ret < 0) {
        printf("HB_VIN_SetPipeAttr error!\n");
        return ret;
    }
    ret = HB_VIN_SetChnDISAttr(pipeId, 1, &disinfo);  //  dis init
    if (ret < 0) {
        printf("HB_VIN_SetChnDISAttr error!\n");
        return ret;
    }
    ret = HB_VIN_SetChnLDCAttr(pipeId, 1, &ldcinfo);  //  ldc init
    if (ret < 0) {
        printf("HB_VIN_SetChnLDCAttr error!\n");
        return ret;
    }
    ret = HB_VIN_SetChnAttr(pipeId, 1);  //  dwe init
    if (ret < 0) {
        printf("HB_VIN_SetChnAttr error!\n");
        return ret;
    }
    ret = HB_VIN_SetDevBindPipe(pipeId, pipeId);  //  bind init
    if (ret < 0) {
        printf("HB_VIN_SetDevBindPipe error!\n");
        return ret;
    }


    ret = HB_VIN_EnableChn(pipeId, 0); // dwe start
    if (ret < 0)
    {
        printf("HB_VIN_EnableChn error!\n");
        return ret;
    }
    ret = HB_VIN_StartPipe(pipeId); // isp start
    if (ret < 0)
    {
        printf("HB_VIN_StartPipe error!\n");
        return ret;
    }
    ret = HB_VIN_EnableDev(devId); // sif start && start thread
    if (ret < 0)
    {
        printf("HB_VIN_EnableDev error!\n");
        return ret;
    }

    /* sensor出流，streamon*/
    ret = HB_MIPI_ResetSensor(devId);
    if (ret < 0)
    {
        printf("HB_MIPI_ResetSensor error!\n");
        return ret;
    }
    ret = HB_MIPI_ResetMipi(snsinfo.sensorInfo.entry_index);
    if (ret < 0)
    {
        printf("HB_MIPI_ResetMipi error, ret= %d\n", ret);
        return ret;
    }
    return ret;
}

void SensorProcess::print_sensor_info(MIPI_SENSOR_INFO_S *snsinfo)
{

    printf("bus_num %d\n", snsinfo->sensorInfo.bus_num);
    printf("bus_type %d\n", snsinfo->sensorInfo.bus_type);
    printf("sensor_name %s\n", snsinfo->sensorInfo.sensor_name);
    printf("reg_width %d\n", snsinfo->sensorInfo.reg_width);
    printf("sensor_mode %d\n", snsinfo->sensorInfo.sensor_mode);
    printf("sensor_addr 0x%x\n", snsinfo->sensorInfo.sensor_addr);
    printf("serial_addr 0x%x\n", snsinfo->sensorInfo.serial_addr);
    printf("resolution %d\n", snsinfo->sensorInfo.resolution);
    printf("devid %d\n", snsinfo->sensorInfo.dev_port);


}

void x3_buf_info_print(hb_vio_buffer_t * buf)
{
    printf("normal pipe_id (%d)type(%d)frame_id(%d)buf_index(%d)w x h(%dx%d) data_type %d img_format %d\n",
           buf->img_info.pipeline_id,
           buf->img_info.data_type,
           buf->img_info.frame_id,
           buf->img_info.buf_index,
           buf->img_addr.width,
           buf->img_addr.height,
           buf->img_info.data_type,
           buf->img_info.img_format);
}
int time_cost_(struct timeval *start, struct timeval *end)
{
    int time_ms = -1;
    time_ms = ((end->tv_sec * 1000 + end->tv_usec /1000) -
               (start->tv_sec * 1000 + start->tv_usec /1000));
    // printf("time cost %d ms \n", time_ms);
    return time_ms;
}

cv::Mat SensorProcess::getImage()
{
    struct timeval time_now = { 0 };
    struct timeval time_next = { 0 };
    int size = -1;
    int ret = 0, i;
    int pipeId = sensor_setting_.snsinfo.sensorInfo.dev_port;
    auto yuv_buf=new hb_vio_buffer_t();
    uint8_t *img_addr;
    cv::Mat yuv_image,rgb_image,resize_model_img;
    gettimeofday(&time_now, NULL);

    /* 从VIN模块获取图像帧 */
    ret = HB_VIN_GetChnFrame(pipeId, 0, yuv_buf, 2000);
    x3_buf_info_print(yuv_buf);
    if (ret < 0)
    {
        printf("HB_VIN_GetChnFrame error!!!\n");
    }
    else {
        // 获取图像的宽高
        int width = yuv_buf->img_addr.width;
        int stride = yuv_buf->img_addr.stride_size;
        int height = yuv_buf->img_addr.height;

        // 获取图像y,uv地址
        char *y_addr = reinterpret_cast<char *>(yuv_buf->img_addr.addr[0]);
        char *c_addr = reinterpret_cast<char *>(yuv_buf->img_addr.addr[1]);
        auto y_img_len = height * width;
        auto uv_img_len = height * width / 2;
        auto img_size = y_img_len + uv_img_len;    //yuv图像大小
        printf("mat success\n");
        img_addr = reinterpret_cast<uint8_t *>(std::calloc(1, img_size));
        // 进行转换
        if (width == stride) {
            std::memcpy(img_addr, y_addr, y_img_len);
            std::memcpy(img_addr+ y_img_len, c_addr, uv_img_len);
        } else {
            // copy y data jump over stride
            for (int i = 0; i < height; i++) {
                auto src_y_addr = y_addr + i * stride;
                auto dst_y_addr = img_addr + i * width;
                memcpy(dst_y_addr, src_y_addr, width);
                printf("circle2 success\n");
            }
            // copy uv data jump over stride
            auto dst_y_size = width * height;
            for (int i = 0; i < height / 2; i++) {
                auto src_c_addr = c_addr + i * stride;
                auto dst_c_addr = img_addr + dst_y_size + i * width;
                memcpy(dst_c_addr, src_c_addr, width);
                printf("circle3 success\n");
            }
        }

        ret = HB_VIN_ReleaseChnFrame(pipeId, 0, yuv_buf);
        if (ret < 0) {
            printf("HB_VIN_ReleaseChnFrame error!!!\n");
        }

        yuv_image = cv::Mat(height * 3 / 2, width, CV_8UC1, img_addr);
        cv::cvtColor(yuv_image, rgb_image, CV_YUV2BGR_NV12);
        gettimeofday(&time_next, NULL);
        int time_cost = time_cost_(&time_now, &time_next);
        printf("get img cost time %d ms\n", time_cost);
    }

    delete(yuv_buf);
    free(img_addr);
    return rgb_image;


    // int new_height = static_cast<int>(rgb_image.cols / (static_cast<double>(1920) / 1080));  
    // cv::resize(rgb_image, resized_image, cv::Size(1920, new_height), cv::INTER_LINEAR);  
    // resized_image = resized_image(cv::Rect(0, (new_height - 1080) / 2, 1920, 1080));  
    // gettimeofday(&time_next, NULL);


    // return resized_image;
}

cv::Mat SensorProcess::transRgbToYuv(cv::Mat rgb_img)
{


    cv::Mat yuv_img;
    cvtColor(rgb_img,yuv_img,CV_BGR2YUV_I420);

    auto src_h = rgb_img.rows;
    auto src_w = rgb_img.cols;

    auto n_y = src_h * src_w;
    auto n_uv = n_y / 2;
    auto n_u = n_y / 4;
    std::vector<uint8_t> uv(n_uv);

    std::copy(yuv_img.data+n_y, yuv_img.data+n_y+n_uv, uv.data());
    for (auto i = 0; i < n_u; i++) {
        yuv_img.data[n_y + 2*i] = uv[i];            // U
        yuv_img.data[n_y + 2*i + 1] = uv[n_u + i];  // V
    }

    return yuv_img;
}

