#include "DataCapturer.h"
#include "ld_log.h"
#include "date_time.h"

#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "CamImuAligner.h"
#include<chrono>

namespace VSLAM 
{
    DataCapturer::DataCapturer(int camFreq,int imuFreq,bool saveData):DataCapture()
    {
        double ImuInverval = 1000 / imuFreq;
        double ImgInverval = 1000 / camFreq;

        double halfImuInverval = ImuInverval / 4.0;
        double halfImgInverval = ImgInverval / 4.0;
        this->mMinImuInverval = ImuInverval - halfImuInverval;
        this->mMinImgInverval = ImgInverval - halfImgInverval;
        this->mbStop = false;
        mAligner.Set(camFreq,imuFreq);
        this->mnFrameId = 0;

        this->mbSaveData = saveData;
    }

    void DataCapturer::Set(int camFreq,int imuFreq,bool saveData)
    {
        double ImuInverval = 1000 / imuFreq;
        double ImgInverval = 1000 / camFreq;

        double halfImuInverval = ImuInverval / 4.0;
        double halfImgInverval = ImgInverval / 4.0;
        this->mMinImuInverval = ImuInverval - halfImuInverval;
        this->mMinImgInverval = ImgInverval - halfImgInverval;
        this->mbStop = false;
        mAligner.Set(camFreq,imuFreq);
        this->mnFrameId = 0;

        this->mbSaveData = saveData;
    }

    void DataCapturer::IMUReceiveLoop() 
    {

        int usleep_time = 2000; // 2ms

        // get imu data thread
        int imu_frame_rate = 200;                                     // 200hz
        uint64_t imu_frame_rate_ustime = 0.5 * 1.0 / (float)imu_frame_rate * 1e6; // 单位是us，注意1e6表示的是10^6
        FLOGD("IMUReceiveLoop - imu_frame_rate_ustime:%llu", imu_frame_rate_ustime);

        // 共享内存
        ShmSensorQueue imu_ssq;

        uint64_t last_time_stamp = 0;              // 上一帧时间戳
        double g = 9.81;                           // 重力加速度
        double acc_unit = 1.0 / 2048 * g;          // 加速度单位

        bool allow_successfully_print = false;
        while (!mbStop) 
        {
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
                usleep(usleep_time);
                continue;
            }

            std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
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

            ImuData_NotAligned imu_save;
            imu_save.time_stamp = imudata.time_stamp * 1000;//us2ns
            imu_save.sync_count = imudata.sync_count;
            imu_save.wm = imumessage.wm;
            imu_save.am = imumessage.am;
            {
                std::unique_lock<std::mutex> lock(mutex_Save_IMU);
                this->mqImuDataForSave.push(imu_save);
            }
            this->mAligner.AddImuData(imu_save);

            
            // FLOGD("imu,%llu,%lu,%f,%f,%f,%f,%f,%f\n", imudata.time_stamp, imudata.sync_count, 
            //     imumessage.wm(0), imumessage.wm(1),imumessage.wm(2),
            //     imumessage.am(0), imumessage.am(1),imumessage.am(2));

            // 成功更新IMU数据之后需要sleep一个周期
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            double totaltime = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count() * 1000;
            if(totaltime < this->mMinImuInverval)
            {
                double sleepTime = this->mMinImuInverval - totaltime;
                usleep(sleepTime * 1000);
            }
        }
    }


    void DataCapturer::ImageReceiveLoop()
    {
        // 共享内存
        ShmSensorQueue cam_ssq;

        // buff
        uint8_t *read_buf = new uint8_t[FRAME_MAX_LEN];
        CameraFrame frame;
        frame.buf = read_buf;
        frame.max_len = FRAME_MAX_LEN;

        // 最新一帧相机图像帧的时间戳
        uint64_t t_c_newest = 0;

        
        uint32_t camera_data_lenth = (640 * 544 * 3 * 2 );

        // 该标志量用于表示上一帧双目图像是否已经更新
        int nLastCount = -1;
        while (!mbStop) 
        {
            
            // 读取图像buf
            int ret_len = cam_ssq.PopAllCameraData(IMAGE_RESIZE_HALF, frame);
            
            bool bOK = GET_BIT(frame.head.capture_flag, LEFT_MIDDEL_CAMERA_BIT_FLAG) == 1 && GET_BIT(frame.head.capture_flag, RIGHT_MIDDEL_CAMERA_BIT_FLAG) == 1;
            if(ret_len >= 0 && bOK) 
            {
                int nCurCount = frame.head.sys_count;
                LOGD("Get New Frame,count = %d",nCurCount);
                if((int)nCurCount != nLastCount)
                {
                    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
                    nLastCount = nCurCount;
                    
                    t_c_newest = frame.head.time_stamp * 1000;

                    cv::Mat l_grayImg = YuvBufToGrayMat(frame.buf + sizeof(CameraFrameHead) + (camera_data_lenth >> 2),640, 544);

                    cv::Mat r_grayImg = YuvBufToGrayMat(frame.buf + sizeof(CameraFrameHead) + (camera_data_lenth >> 1), 640, 544);
                    this->mnFrameId++;

                    //转换为保存对象
                    if(mbSaveData)
                    {
                        StereoImages tmpStereoImage;
                        std::unique_lock<std::mutex> lock(mutex_Save_Stereo_Image);
                        tmpStereoImage.mLeftImg   = l_grayImg;
                        tmpStereoImage.mRightImg  = r_grayImg;
                        tmpStereoImage.count      = nCurCount;
                        tmpStereoImage.mTimeStamp = t_c_newest;
                        this->mqStereoImage.push(tmpStereoImage);
                    }

                    //与IMU对齐
                    double camTimeStamp = t_c_newest;
                    std::vector<ImuData> vImus = mAligner.GetAlignImuData(camTimeStamp,nCurCount);
                    for(uint32_t id = 0; id < vImus.size();id++)
                    {
                        jarvis_pic::ImuData imudata;
                        
                        imudata.time = (uint64_t)(vImus[id].timestamp * 1e6);
                        imudata.linear_acceleration = vImus[id].am;
                        imudata.angular_velocity = vImus[id].wm;
                        for (const auto& f : imu_call_backs_) 
                        {
                           f(imudata);
                        }
                    }

                    jarvis_pic::Frame tmpframe;
                    tmpframe.id    = 0;
                    tmpframe.time  = frame.head.time_stamp;
                    tmpframe.image = l_grayImg;
                    for (auto& f : frame_call_backs_) 
                    {
                       f(tmpframe);
                    }
                    

                    if(mbSaveData)
                    {
                        std::unique_lock<std::mutex> lock(mutex_Aligned_IMU);
                        for(long unsigned int id = 0; id < vImus.size();id++)
                        {
                            mqImuDataAligned.push(vImus[id]);
                        }
                    }
                    
                    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                    double totaltime = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count() * 1000;
                    if(totaltime < this->mMinImgInverval)
                    {
                        double sleepTime = this->mMinImgInverval - totaltime;
                        usleep(sleepTime * 1000);
                    }

                }
            }
            else
            {
                usleep(5000);
            }
        } // while循环的结束大括号

    }

    void DataCapturer::Start() 
    {
        this->mbStop = false;
        // 申请IMU数据的处理线程
        this->mtGetimuthread = std::thread([](DataCapturer *p_this) { p_this->IMUReceiveLoop(); }, this);

        if(this->mbSaveData)
        {
            this->mtSaveSensorDataThread = std::thread([](DataCapturer *p_this) { p_this->SaveSensorData(); }, this);
        }

        this->mtCamCapture_AlignedThread = std::thread([](DataCapturer *p_this) { p_this->ImageReceiveLoop(); }, this);

        // this->mtCamCapture_AlignedThread.join();
    } 

    void DataCapturer::Stop()
    {
        this->mbStop = true;
    }

    void DataCapturer::SaveSensorData()
    {
        
        std::string strPath("/mnt/UDISK/calib_save/");
        std::ofstream of1(strPath + std::string("imageFile.txt"), std::ios::out);
        if (!of1.is_open()) 
        {
            std::cout << "imu file can not open to write" << std::endl;
            return;
        }
        of1 << "timestamp,count" << std::endl;

        std::ofstream of2(strPath + std::string("imu.txt"), std::ios::out);
        if (!of2.is_open()) {
            std::cout << "imu file can not open to write" << std::endl;
            return;
        }
        of2 << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z" << std::endl;

        std::ofstream of3(strPath + std::string("imu1.csv"), std::ios::out);
        of3 << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z" << std::endl;

        std::ofstream of4(strPath + std::string("imu_aligned.csv"), std::ios::out);
        of4 << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z" << std::endl;

        while(!mbStop)
        {
            bool bGetImg = false;
            int nQueueSize = 0;
            StereoImages curStereoImage;
            {
                std::unique_lock<std::mutex> lock(mutex_Save_Stereo_Image);
                if(!mqStereoImage.empty())
                {
                    
                    curStereoImage = this->mqStereoImage.front();
                    nQueueSize =  this->mqStereoImage.size();
                    this->mqStereoImage.pop();
                    bGetImg = true;
                }
            }

            if(bGetImg)
            {
                std::string leftName1  = strPath + std::string("cam0/") + std::to_string(curStereoImage.mTimeStamp)  + std::string(".png");
                std::string rightName1 = strPath + std::string("cam1/") + std::to_string(curStereoImage.mTimeStamp)  + std::string(".png");
                cv::imwrite(leftName1, curStereoImage.mLeftImg);
                cv::imwrite(rightName1, curStereoImage.mRightImg);
                of1 << curStereoImage.mTimeStamp << ","<<curStereoImage.count<<","<<nQueueSize<<std::endl;
            }
            
            bool bImuToSave = true;
            bool bAlignedImuToSave = false;
            int nImuSegCount = 0;
            while(bImuToSave && nImuSegCount < 20)//一次最多存20个
            {
                bImuToSave = false;
                ImuData_NotAligned imudata;
                {
                    std::unique_lock<std::mutex> lock(mutex_Save_IMU);
                    if(!mqImuDataForSave.empty())
                    {
                        
                        imudata = this->mqImuDataForSave.front();
                        this->mqImuDataForSave.pop();
                        bImuToSave = true;
                        nImuSegCount++;
                    }

                }

                if(bImuToSave)
                {
                    of2  << std::setiosflags(std::ios::fixed) << std::setprecision(20);
                    of2  <<imudata.time_stamp << ","
                        <<imudata.sync_count<<","
                        << imudata.wm(0) << "," << imudata.wm(1) << "," << imudata.wm(2) << ","
                        << imudata.am(0) << "," << imudata.am(1) << "," << imudata.am(2) << std::endl;

                    of3 << std::setiosflags(std::ios::fixed) << std::setprecision(20);
                    of3 <<imudata.time_stamp << ","
                        << imudata.wm(0) << "," << imudata.wm(1) << "," << imudata.wm(2) << ","
                        << imudata.am(0) << "," << imudata.am(1) << "," << imudata.am(2) << std::endl;
                
                }

                bAlignedImuToSave = false;
                ImuData AlignedImu;
                {
                    std::unique_lock<std::mutex> lock(mutex_Aligned_IMU);
                    if(!mqImuDataAligned.empty())
                    {
                        AlignedImu = mqImuDataAligned.front();
                        bAlignedImuToSave = true;
                        mqImuDataAligned.pop();
                    }
                }

                if(bAlignedImuToSave)
                {
                    of4 << std::setiosflags(std::ios::fixed) << std::setprecision(10);
                    of4 <<(uint64_t)(AlignedImu.timestamp * 1.0e9) << ","
                        << AlignedImu.wm(0) << "," << AlignedImu.wm(1) << "," << AlignedImu.wm(2) << ","
                        << AlignedImu.am(0) << "," << AlignedImu.am(1) << "," << AlignedImu.am(2) << std::endl;
                }
            }
            


        }
        of1.close();
        of2.close();
        of3.close();
        of4.close();
    }

    cv::Mat DataCapturer::YuvBufToGrayMat(uint8_t *buf, uint32_t width, uint32_t height)
    {
        cv::Mat yuvMat(height + height/2, width, CV_8UC1, (unsigned char*)buf);
        cv::Mat grayMat;
        cv::cvtColor(yuvMat, grayMat, cv::COLOR_YUV2GRAY_NV21);
        return grayMat;
    }

    DataCapturer::~DataCapturer()
    {
        this->mbStop = true;
        this->mtGetimuthread.join();

        if(this->mbSaveData)
        {
           this->mtSaveSensorDataThread.join();
        }
    }
}