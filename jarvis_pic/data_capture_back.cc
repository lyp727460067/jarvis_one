#include <chrono>
#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "data_capture.h"
#include "ld_log.h"

namespace VSLAM {

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
#define COUNT_BE_0_PERIOD 2  // 单位s
// 该变量用于当imu_queue中没找到合适count的数据帧时 对同一图像帧
// 再次查询CHECK_I_TH帧IMU数据
#define CHECK_I_TH 15
// 当找不到同步count的IMU数据帧时，通过时间差寻找IMU数据帧时，允许的最大时间差，单位是us
#define DELTA_T_TH 10 * 1000  // 单位us
// 相机图像数据处理线程的Sleep周期
#define SLEEP_PERIOD 5000

// 相机图像帧数据队列的长度
#define STEREO_QUEUE_SIZE 3
// 用于控制图像的读取频率
#define POP_COUNT_TH 15

// #define MTEST    // for test
// #define CAPDATA     // for capture imu&camera data
// #define ONLYCAPIMU

#define FIXED_IMU_DEQUE

DataCapturer::DataCapturer() { this->mbStop = false; }

void DataCapturer::IMUReceiveLoop() {
  int usleep_time = 2500;  // 5ms

  // get imu data thread
  int imu_frame_rate = 200;  // 200hz
  uint64_t imu_frame_rate_ustime =
      0.5 * 1.0 / (float)imu_frame_rate * 1e6;  // 单位是us，注意1e6表示的是10^6
  FLOGD("IMUReceiveLoop - imu_frame_rate_ustime:%llu", imu_frame_rate_ustime);

  // 共享内存
  ShmSensorQueue imu_ssq;

  uint64_t last_time_stamp = 0;      // 上一帧时间戳
  double g = 9.81;                   // 重力加速度
  double acc_unit = 1.0 / 2048 * g;  // 加速度单位

  // #ifndef ONLYCAPIMU
  // #ifdef FIXED_IMU_DEQUE
  //     uint16_t imu_data_qsize = imu_frame_rate * 10; // imu队列大小：200hz *
  //     10秒
  // #endif
  // #endif

  bool allow_successfully_print = false;
  while (!mbStop) {
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
      // FLOGW("IMUReceiveLoop - IMU data update successfully!
      // imudata.sync_count:%d", imudata.sync_count);
    }

    last_time_stamp = imudata.time_stamp;

    // 输入imu数据
    ImuData imumessage;
    // 将时间戳转化成以秒为单位
    imumessage.timestamp = us2s(imudata.time_stamp);
    imumessage.wm << imudata.imu_data.gyro_x * 1e-3,
        imudata.imu_data.gyro_y * 1e-3,
        imudata.imu_data.gyro_z * 1e-3;  // mrad/s to rad/s
    imumessage.am << imudata.imu_data.accel_x * acc_unit,
        imudata.imu_data.accel_y * acc_unit,
        imudata.imu_data.accel_z * acc_unit;  // 1/2048 g to m/s2

    ImuData_forSave imu_save;
    imu_save.time_stamp = imudata.time_stamp;
    imu_save.sync_count = imudata.sync_count;
    imu_save.wm = imumessage.wm;
    imu_save.am = imumessage.am;
    {
      std::unique_lock<std::mutex> lock(mutex_Save_IMU);
      this->mqImuData.push(imu_save);
    }

    FLOGD("imu,%llu,%lu,%f,%f,%f,%f,%f,%f\n", imudata.time_stamp,
          imudata.sync_count, imumessage.wm(0), imumessage.wm(1),
          imumessage.wm(2), imumessage.am(0), imumessage.am(1),
          imumessage.am(2));

    // 成功更新IMU数据之后需要sleep一个周期
    usleep(imu_frame_rate_ustime);
  }
}

void DataCapturer::ImageReceiveLoop() {
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
  while (!mbStop) {
    // 读取图像buf
    int ret_len = cam_ssq.PopAllCameraData(IMAGE_RESIZE_HALF, frame);
    LOGD("ret len:%d", ret_len);

    if (ret_len >= 0) {
      int nCurCount = frame.head.sys_count;
      if ((int)nCurCount != nLastCount) {
        nLastCount = nCurCount;

        t_c_newest = frame.head.time_stamp * 1000;

        cv::Mat l_grayImg = YuvBufToGrayMat(
            frame.buf + sizeof(CameraFrameHead) + (frame.head.len >> 2),
            (frame.head.len - sizeof(CameraFrameHead)) >> 1, 640, 544);

        cv::Mat r_grayImg = YuvBufToGrayMat(
            frame.buf + sizeof(CameraFrameHead) + (frame.head.len >> 1),
            (frame.head.len - sizeof(CameraFrameHead)) >> 1, 640, 544);

        // 转换为保存对象
        {
          StereoImages tmpStereoImage;
          std::unique_lock<std::mutex> lock(mutex_Save_Stereo_Image);
          tmpStereoImage.mLeftImg = l_grayImg;
          tmpStereoImage.mRightImg = r_grayImg;
          tmpStereoImage.count = nCurCount;
          tmpStereoImage.mTimeStamp = t_c_newest;
          this->mqStereoImage.push(tmpStereoImage);
        }
      }
    }

    usleep(SLEEP_PERIOD);

  }  // while循环的结束大括号
}

void DataCapturer::Run() {
  // 申请IMU数据的处理线程
  this->get_imu_thread_ =
      std::thread([](DataCapturer *p_this) { p_this->IMUReceiveLoop(); }, this);
  this->mtSaveSensorDataThread =
      std::thread([](DataCapturer *p_this) { p_this->SaveSensorData(); }, this);

  ImageReceiveLoop();
}

void DataCapturer::SaveSensorData() {
  std::string strPath("/mnt/UDISK/calib_save/");
  std::ofstream of1(strPath + std::string("imageFile.txt"), std::ios::out);
  if (!of1.is_open()) {
    std::cout << "imu file can not open to write" << std::endl;
    return;
  }
  of1 << "timestamp,count" << std::endl;

  std::ofstream of2(strPath + std::string("imu.txt"), std::ios::out);
  if (!of2.is_open()) {
    std::cout << "imu file can not open to write" << std::endl;
    return;
  }
  of2 << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z"
      << std::endl;

  std::ofstream of3(strPath + std::string("imu1.csv"), std::ios::out);
  of3 << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z"
      << std::endl;
  while (true) {
    bool bGetImg = false;
    int nQueueSize = 0;
    StereoImages curStereoImage;
    {
      std::unique_lock<std::mutex> lock(mutex_Save_Stereo_Image);
      if (!mqStereoImage.empty()) {
        curStereoImage = this->mqStereoImage.front();
        nQueueSize = this->mqStereoImage.size();
        this->mqStereoImage.pop();
        bGetImg = true;
      }
    }

    if (bGetImg) {
      std::string leftName1 = strPath + std::string("cam0/") +
                              std::to_string(curStereoImage.mTimeStamp) +
                              std::string(".png");
      std::string rightName1 = strPath + std::string("cam1/") +
                               std::to_string(curStereoImage.mTimeStamp) +
                               std::string(".png");
      cv::imwrite(leftName1, curStereoImage.mLeftImg);
      cv::imwrite(rightName1, curStereoImage.mRightImg);
      of1 << curStereoImage.mTimeStamp << "," << curStereoImage.count << ","
          << nQueueSize << std::endl;
    }

    bool bImuToSave = true;
    int nImuSegCount = 0;
    while (bImuToSave && nImuSegCount < 20)  // 一次最多存20个
    {
      bImuToSave = false;
      ImuData_forSave imudata;
      {
        std::unique_lock<std::mutex> lock(mutex_Save_IMU);
        if (!mqImuData.empty()) {
          imudata = this->mqImuData.front();
          this->mqImuData.pop();
          bImuToSave = true;
          nImuSegCount++;
        }
      }

      if (bImuToSave) {
        of2 << std::setiosflags(std::ios::fixed) << std::setprecision(20);
        of2 << imudata.time_stamp * 1000 << "," << imudata.sync_count << ","
            << imudata.wm(0) << "," << imudata.wm(1) << "," << imudata.wm(2)
            << "," << imudata.am(0) << "," << imudata.am(1) << ","
            << imudata.am(2) << std::endl;

        of3 << std::setiosflags(std::ios::fixed) << std::setprecision(20);
        of3 << imudata.time_stamp * 1000 << "," << imudata.wm(0) << ","
            << imudata.wm(1) << "," << imudata.wm(2) << "," << imudata.am(0)
            << "," << imudata.am(1) << "," << imudata.am(2) << std::endl;
      }
    }
  }
  of1.close();
  of2.close();
  of3.close();
}

cv::Mat DataCapturer::YuvBufToGrayMat(uint8_t *buf, long size, uint32_t width,
                                      uint32_t height) {
  cv::Mat yuvMat(height + height / 2, width, CV_8UC1, (unsigned char *)buf);
  cv::Mat grayMat;
  cv::cvtColor(yuvMat, grayMat, cv::COLOR_YUV2GRAY_NV21);
  return grayMat;
}

DataCapturer::~DataCapturer() {
  this->mbStop = true;
  this->get_imu_thread_.join();
  this->mtSaveSensorDataThread.join();
}
}  // namespace VSLAM
