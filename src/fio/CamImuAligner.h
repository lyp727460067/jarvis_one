#ifndef __MOWER_PACK_CAM_IMU_ALIGNER__
#define __MOWER_PACK_CAM_IMU_ALIGNER__
#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include <list>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "Eigen/Core"
// #include "datatype.h"
#include "utils/sensor_data.h"
using namespace VSLAM;
class CamImuAligner
{
public:
    CamImuAligner(int imgFreq,int imuFreq);
    CamImuAligner(){};

    void Set(int imgFreq,int imuFreq);

    //输入时间均为ns
    std::vector<ImuData> GetAlignImuData(double camTimeStamp,uint32_t imgCount);

    uint32_t ProessImg(double camTimeStamp,uint32_t imgCount);
    void AddImuData(ImuData_NotAligned &imu);
private:
    double mPreImuTime;
    bool   mbFirstImu;
    double mRealImuInverval;
    double mImuInverval;
    double mMaxImuInverval;
    double mMinImuInverval;
    bool mbImuInterrupt;
    uint32_t mnLastImuCountNotRec;
    uint32_t mnLastImuCount;
    bool mbIsFirstImg;
    double mImgImuTimeDiff;
    double mImuSegStartTime;

    std::mutex mutex_Align_IMU;
    std::queue<ImuData_NotAligned> mqNotAlignedImu;

    std::list<ImuData_NotAligned> mlOutdatedImu;
    double mLastImuAlignedTime;
    bool mbImuTimeLess;
    double mImgInverval;
    double mLastAlignTime;
    double mPreImgImuTimeDiff;
    double mMaxImgInverval;
    double mMinImgInverval;

    double mPreImgTime;
    double mnFrameId;
    uint32_t mnLastImgCount;
};
#endif