#include "DataCapturer.h"
#include "ld_log.h"
#include "date_time.h"

#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp>

#include<chrono>

namespace VSLAM 
{

    #ifndef STEREO_ORIGIN_WIDTH
    #define STEREO_ORIGIN_WIDTH 1280
    #endif

    #ifndef STEREO_ORIGIN_HEIGHT
    #define STEREO_ORIGIN_HEIGHT 1072
    #endif

    #ifndef STEREO_HALF_WIDTH
    #define STEREO_HALF_WIDTH 640
    #endif

    #ifndef STEREO_HALF_HEIGHT
    #define STEREO_HALF_HEIGHT 544
    #endif

    // COUNT计数值归零的周期，单位是秒
    #define COUNT_BE_0_PERIOD 2    // 单位s
    // 该变量用于当imu_queue中没找到合适count的数据帧时 对同一图像帧 再次查询CHECK_I_TH帧IMU数据
    #define CHECK_I_TH      15
    // 当找不到同步count的IMU数据帧时，通过时间差寻找IMU数据帧时，允许的最大时间差，单位是us
    #define DELTA_T_TH      10*1000      // 单位us
    // 相机图像数据处理线程的Sleep周期
    #define SLEEP_PERIOD    5000

    // 相机图像帧数据队列的长度
    #define STEREO_QUEUE_SIZE 3
    // 用于控制图像的读取频率
    #define POP_COUNT_TH    15


    // #define MTEST    // for test
    // #define CAPDATA     // for capture imu&camera data
    // #define ONLYCAPIMU

    #define FIXED_IMU_DEQUE

    DataCapturer::DataCapturer() 
    {

    }

    void DataCapturer::IMUReceiveLoop() {
    // #ifdef CAPDATA
        std::string strPath("/mnt/UDISK/calib_save/");
        std::ofstream of(strPath + std::string("imu.txt"), std::ios::out);
        if (!of.is_open()) {
            std::cout << "imu file can not open to write" << std::endl;
            return;
        }
        of << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z" << std::endl;

        std::ofstream of1(strPath + std::string("imu1.csv"), std::ios::out);
        of1 << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z" << std::endl;
    // #endif

        int usleep_time = 2500; // 5ms

        // get imu data thread
        int imu_frame_rate = 200;                                     // 200hz
        uint64_t imu_frame_rate_ustime = 0.5 * 1.0 / (float)imu_frame_rate * 1e6; // 单位是us，注意1e6表示的是10^6
        FLOGD("IMUReceiveLoop - imu_frame_rate_ustime:%llu", imu_frame_rate_ustime);

        // 共享内存
        ShmSensorQueue imu_ssq;

        uint64_t last_time_stamp = 0;              // 上一帧时间戳
        double g = 9.81;                           // 重力加速度
        double acc_unit = 1.0 / 2048 * g;          // 加速度单位

    // #ifndef ONLYCAPIMU
    // #ifdef FIXED_IMU_DEQUE
    //     uint16_t imu_data_qsize = imu_frame_rate * 10; // imu队列大小：200hz * 10秒
    // #endif
    // #endif

        bool allow_successfully_print = false;
        while (true) {
            // 读取共享内存
            ModSyncImuFb imudata;
            int32_t res = imu_ssq.PopImuData(&imudata);
            if (res <= 0) {
                // std::cout << "Pop imu data failed, id: " << res << std::endl;
                // FLOGE("IMUReceiveLoop - IMU data res < 0! Failed to get!");
                allow_successfully_print = true;
                usleep(usleep_time);
                continue;
            }

            // 去重
            if (imudata.time_stamp == last_time_stamp) {
                // FLOGE("IMUReceiveLoop - IMU data not update!");
                allow_successfully_print = true;
                usleep(imu_frame_rate_ustime);
                continue;
            }

            allow_successfully_print = true;
            if (allow_successfully_print) {
                allow_successfully_print = false;
                // FLOGW("IMUReceiveLoop - IMU data update successfully! imudata.sync_count:%d", imudata.sync_count);
            }

            last_time_stamp = imudata.time_stamp;
            

            // 输入imu数据
            ImuData imumessage;
            // 将时间戳转化成以秒为单位
            imumessage.timestamp = us2s(imudata.time_stamp);
            imumessage.wm << imudata.imu_data.gyro_x * 1e-3, imudata.imu_data.gyro_y * 1e-3, imudata.imu_data.gyro_z * 1e-3;             // mrad/s to rad/s
            imumessage.am << imudata.imu_data.accel_x * acc_unit, imudata.imu_data.accel_y * acc_unit, imudata.imu_data.accel_z * acc_unit; // 1/2048 g to m/s2

            of << std::setiosflags(std::ios::fixed) << std::setprecision(10);
            of <<imudata.time_stamp * 1000 << ","
            <<imudata.sync_count<<","
            << imumessage.wm(0) << "," << imumessage.wm(1) << "," << imumessage.wm(2) << ","
            << imumessage.am(0) << "," << imumessage.am(1) << "," << imumessage.am(2) << std::endl;
            of1 << std::setiosflags(std::ios::fixed) << std::setprecision(10);
            of1 <<imudata.time_stamp * 1000 << ","
                << imumessage.wm(0) << "," << imumessage.wm(1) << "," << imumessage.wm(2) << ","
                << imumessage.am(0) << "," << imumessage.am(1) << "," << imumessage.am(2) << std::endl;
            
            FLOGD("imu,%llu,%lu,%f,%f,%f,%f,%f,%f\n", imudata.time_stamp, imudata.sync_count, 
                imumessage.wm(0), imumessage.wm(1),imumessage.wm(2),
                imumessage.am(0), imumessage.am(1),imumessage.am(2));
    /*   
    #ifdef MTEST
            FLOGD("imu,%llu,%lu,%f,%f,%f,%f,%f,%f\n", imudata.time_stamp, imudata.sync_count, imudata.imu_data.gyro_x * 1e-3, imudata.imu_data.gyro_y * 1e-3, imudata.imu_data.gyro_z * 1e-3,
                imudata.imu_data.accel_x * acc_unit, imudata.imu_data.accel_y * acc_unit, imudata.imu_data.accel_z * acc_unit);
    #elif defined(CAPDATA)
            of << std::setiosflags(std::ios::fixed) << std::setprecision(10);
            of << (uint64_t)s2ns(imumessage.timestamp) << ","
            << imumessage.wm(0) << "," << imumessage.wm(1) << "," << imumessage.wm(2) << ","
            << imumessage.am(0) << "," << imumessage.am(1) << "," << imumessage.am(2) << std::endl;
    #else
            m_sys_->feed_measurement_imu(imumessage);
    #endif

            // 保存imu队列
            {
    #ifndef ONLYCAPIMU
                std::lock_guard<std::mutex> lock(imu_data_qmutex_);
                imu_data_deque_.push_back(imudata);
                
    #ifdef FIXED_IMU_DEQUE
                if (imu_data_deque_.size() > imu_data_qsize) {
                    imu_data_deque_.pop_front();
                }
    #endif
    #endif
            }
    */
            // 成功更新IMU数据之后需要sleep一个周期
            usleep(imu_frame_rate_ustime);
        }
    }

    void DataCapturer::Run() 
    {
        std::string strPath("/mnt/UDISK/calib_save/");

        // 申请IMU数据的处理线程
        get_imu_thread_ = std::thread([](DataCapturer *p_this) { p_this->IMUReceiveLoop(); }, this);
        this->mtSaveStereoImageThread = std::thread([](DataCapturer *p_this) { p_this->SaveStereoImage(); }, this);

    #ifdef ONLYCAPIMU
        // 在C++中，线程的join操作会导致主线程阻塞等待被join的线程执行完毕
        get_imu_thread_.join();
    #endif

        // get camera data thread
        // 共享内存
        ShmSensorQueue cam_ssq;

        // buff
        uint8_t *read_buf = new uint8_t[FRAME_MAX_LEN];
        CameraFrame frame;
        frame.buf = read_buf;
        frame.max_len = FRAME_MAX_LEN;

        // 最新一帧相机图像帧的时间戳
        uint64_t t_c_newest = 0;

        // 该标志量用于表示上一帧双目图像是否已经更新
        int nLastCount = -1;
        while (true) 
        {
            // 读取图像buf
            int ret_len = cam_ssq.PopAllCameraData(IMAGE_RESIZE_HALF, frame);
            LOGD("ret len:%d", ret_len);

            if(ret_len >= 0) 
            {
            
                int nCurCount = frame.head.sys_count;
                if((int)nCurCount != nLastCount)
                {
                    nLastCount = nCurCount;
                    
                    t_c_newest = frame.head.time_stamp * 1000;

                    cv::Mat l_grayImg = YuvBufToGrayMat(frame.buf + sizeof(CameraFrameHead) + (frame.head.len >> 2),
                                                (frame.head.len - sizeof(CameraFrameHead)) >> 1, 640, 544);

                    cv::Mat r_grayImg = YuvBufToGrayMat(frame.buf + sizeof(CameraFrameHead) + (frame.head.len >> 1),
                                                    (frame.head.len - sizeof(CameraFrameHead)) >> 1, 640, 544);

                    //转换为保存对象
                    {
                        StereoImages tmpStereoImage;
                        std::unique_lock<std::mutex> lock(mutex_Save_Stereo_Image);
                        tmpStereoImage.mLeftImg   = l_grayImg;
                        tmpStereoImage.mRightImg  = r_grayImg;
                        tmpStereoImage.count      = nCurCount;
                        tmpStereoImage.mTimeStamp = t_c_newest;
                        this->mqStereoImage.push(tmpStereoImage);
                    }
                }

            }

            usleep(SLEEP_PERIOD);

        } // while循环的结束大括号

    } 


    void DataCapturer::SaveStereoImage()
    {
        std::string strPath("/mnt/UDISK/calib_save/");
        std::ofstream of(strPath + std::string("imageFile.txt"), std::ios::out);
        if (!of.is_open()) 
        {
            std::cout << "imu file can not open to write" << std::endl;
            return;
        }
        of << "timestamp,count" << std::endl;
        while(true)
        {
            bool bGetImg = false;
            StereoImages curStereoImage;
            {
                
                if(!mqStereoImage.empty())
                {
                    std::unique_lock<std::mutex> lock(mutex_Save_Stereo_Image);
                    curStereoImage = this->mqStereoImage.front();
                    this->mqStereoImage.pop();
                    bGetImg = true;
                }
                if(bGetImg)
                {
                    std::string leftName1  = strPath + std::string("cam0/") + std::to_string(curStereoImage.mTimeStamp)  + std::string(".png");
                    std::string rightName1 = strPath + std::string("cam1/") + std::to_string(curStereoImage.mTimeStamp)  + std::string(".png");
                    cv::imwrite(leftName1, curStereoImage.mLeftImg);
                    cv::imwrite(rightName1, curStereoImage.mRightImg);
                    of << curStereoImage.mTimeStamp << ","<<curStereoImage.count<<std::endl;
                }
                else
                {
                    usleep(5000);
                }
            }
        }
        of.close();
    }
    

    cv::Mat DataCapturer::YuvBufToGrayMat(uint8_t *buf, long size, uint32_t width, uint32_t height)
    {
        // int yuv_img_buf_len = CAMERA_WIDTH * CAMERA_HEIGHT * 3 / 2;
        // // // 创建 YUV NV21 缓冲区
        // unsigned char* nv21Buffer = new unsigned char[size];
        // unsigned char *nv21Buffer;
        // uint64_t t0 = NowMs();
        // memcpy(nv21Buffer, buf, size * sizeof(unsigned char));
        cv::Mat yuvMat(height + height/2, width, CV_8UC1, (unsigned char*)buf);
        cv::Mat grayMat;
        // uint64_t t1 = NowMs();
        // std::cout<<"memcopy NV21 time cost:"<<t1-t0<<std::endl;
        cv::cvtColor(yuvMat, grayMat, cv::COLOR_YUV2GRAY_NV21);
        // uint64_t t2 = NowMs();
        // std::cout<<"cvt NV21 time cost:"<<t2-t1<<std::endl;
        // delete[] nv21Buffer;
        // nv21Buffer = nullptr;
        return grayMat;
    }
}