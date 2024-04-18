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

#include "utils/sensor_data.h"
#include "utils/utility.h"


namespace VSLAM 
{

typedef struct StereoImages
{
    cv::Mat mLeftImg;
    cv::Mat mRightImg;
    uint64_t mTimeStamp;
    int count;
}StereoImages;
class DataCapturer {

public:
    DataCapturer();

    void Run();

private:

    /**
     * @brief 处理IMU数据的线程，负责接收IMU数据并且维持对应的数据队列
     */
    void IMUReceiveLoop();

    void SaveStereoImage();
    
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
    std::thread mtSaveStereoImageThread;
};

}


#endif
