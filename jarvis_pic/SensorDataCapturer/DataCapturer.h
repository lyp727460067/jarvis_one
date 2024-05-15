#ifndef SYNCONLIENDATA_H
#define SYNCONLIENDATA_H

#include <thread>
#include <memory>
#include <queue>
#include <mutex>
#include <deque>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "data_protocol.h"
#include "shm_sensor_queue.h"

#include "sensor_data.h"
// #include "utils/utility.h"
#include "CamImuAligner.h"
#include <chrono>

#include "../data_capture.h"
#define FRAME_MAX_LEN (4116580)

#define us2s(a) (a * 1e-6)  //us转换成s
#define s2us(a) (a * 1e6)  //s转换成us
#define us2ns(a) (a * 1e3)  //us转换成ns
#define s2ns(a) (a * 1e9)  //s转换成ns
namespace VSLAM 
{


class DataCapturer:public jarvis_pic::DataCapture{

public:
    DataCapturer(int camFreq,int imuFreq,bool saveData = false);

    DataCapturer(){};

    void Set(int camFreq,int imuFreq,bool saveData = false);

    void Start();

    void Stop();

    ~DataCapturer();

private:

    /**
     * @brief 处理IMU数据的线程，负责接收IMU数据并且维持对应的数据队列
     */
    void IMUReceiveLoop();

    void ImageReceiveLoop();

    void SaveSensorData();

    cv::Mat YuvBufToGrayMat(uint8_t *buf, long size, uint32_t width, uint32_t height);


private:
    // IMU数据队列
    std::deque<ModSyncImuFb> imu_data_deque_;
    // IMU数据用的锁
    std::mutex imu_data_qmutex_;

    // IMU数据的接收和处理线程
    std::thread get_imu_thread_;

    //待保存图像序列
    std::queue<StereoImages> mqStereoImage;//first是路径，second是图像
    std::mutex mutex_Save_Stereo_Image;

    std::queue<ImuData_NotAligned> mqImuDataForSave;
    std::mutex mutex_Save_IMU;

    std::mutex mutex_Aligned_IMU;
    std::queue<ImuData> mqImuDataAligned;

    std::thread mtSaveSensorDataThread;

    CamImuAligner mAligner;

    int mnFrameId;
    bool mbStop;
    bool mbSaveData;
    double mMinImgInverval;
    double mMinImuInverval; 

};

}


#endif
