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
#include <mutex>
#include "Eigen/Core"
// #include "datatype.h"
#include "sensor_data.h"
// using namespace VSLAM;
namespace VSLAM
{
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
        void PopNotAlignedImu();
        ImuData_NotAligned GetFrontNotAlignedImu();
    private:
        
        double mRealImuInverval;
        double mImuInverval;
        double mMaxImuInverval;
        double mMinImuInverval;
        double mImgInverval;
        double mMaxImgInverval;
        double mMinImgInverval;

        double mImgImuTimeDiff;
        double mPreImgImuTimeDiff;
        double mImuSegStartTime;
        double mLastImuAlignedTime;
        double mCountStartImuTime;//同一个count的起始时间

        double mPreImgTime;
        double mPreImuTime;

        bool mbImuInterrupt;
        bool mbFirstImu;
        bool mbIsFirstImg;
        bool mbImuTimeLess;


        int mnFrameId;
        uint32_t mnLastImuCountNotRec;
        uint32_t mnLastImuCount;
        uint32_t mnLastImgCount;
        uint32_t mnPrevImuCount;


        std::mutex mutex_Align_IMU;
        std::queue<ImuData_NotAligned> mqNotAlignedImu;

        std::list<ImuData_NotAligned> mlOutdatedImu;
    };
}
#endif