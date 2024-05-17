#include "CamImuAligner.h"

namespace VSLAM
{
    CamImuAligner::CamImuAligner(int imgFreq,int imuFreq)
    {
        this->mPreImuTime = 0.0;
        this->mbFirstImu  = true;
        this->mRealImuInverval = 0.0;
        this->mImuInverval = 1.0 / imuFreq;
        this->mImgInverval = 1.0 / imgFreq;

        double halfImuInverval = mImuInverval / 4.0;
        double halfImgInverval = mImgInverval / 4.0;
        this->mMaxImuInverval = mImuInverval + halfImuInverval;
        this->mMinImuInverval = mImuInverval - halfImuInverval;
        this->mMaxImgInverval = mImgInverval + halfImgInverval;
        this->mMinImgInverval = mImgInverval - halfImgInverval;

        this->mbImuInterrupt = false;
        this->mnLastImuCountNotRec = -1;
        this->mnLastImuCount = -1;
        this->mbIsFirstImg = true;
        this->mImgImuTimeDiff = 0.0;
        this->mImuSegStartTime = 0.0;
        this->mlOutdatedImu.clear();

        this->mLastImuAlignedTime = -1.0;
        this->mbImuTimeLess = false;
        this->mPreImgImuTimeDiff = 0.0;

        this->mPreImgTime = 0.0;
        this->mnFrameId = 0;
        this->mnLastImgCount = 0;
        this->mCountStartImuTime = 0.0;

        this->mnPrevImuCount = 100;
    }

    void CamImuAligner::Set(int imgFreq,int imuFreq)
    {
        this->mPreImuTime = 0.0;
        this->mbFirstImu  = true;
        this->mRealImuInverval = 0.0;
        this->mImuInverval = 1.0 / imuFreq;
        this->mImgInverval = 1.0 / imgFreq;

        double halfImuInverval = mImuInverval / 4.0;
        double halfImgInverval = mImgInverval / 4.0;
        this->mMaxImuInverval = mImuInverval + halfImuInverval;
        this->mMinImuInverval = mImuInverval - halfImuInverval;
        this->mMaxImgInverval = mImgInverval + halfImgInverval;
        this->mMinImgInverval = mImgInverval - halfImgInverval;

        this->mbImuInterrupt = false;
        this->mnLastImuCountNotRec = 0;
        this->mnLastImuCount = 0;
        this->mbIsFirstImg = true;
        this->mImgImuTimeDiff = 0.0;
        this->mImuSegStartTime = 0.0;
        this->mlOutdatedImu.clear();

        this->mLastImuAlignedTime = -1.0;
        this->mbImuTimeLess = false;
        this->mPreImgImuTimeDiff = 0.0;

        this->mPreImgTime = 0.0;
        this->mnFrameId = 0;
        this->mnLastImgCount = 0;
        this->mCountStartImuTime = 0.0;

        this->mnPrevImuCount = 100;
    }

    void CamImuAligner::AddImuData(ImuData_NotAligned &imu)
    {
        std::unique_lock<std::mutex> lock(mutex_Align_IMU);
        this->mqNotAlignedImu.push(imu);
    }

    void CamImuAligner::PopNotAlignedImu()
    {
        std::unique_lock<std::mutex> lock(mutex_Align_IMU);
        mqNotAlignedImu.pop();
    }

    ImuData_NotAligned CamImuAligner::GetFrontNotAlignedImu()
    {
        std::unique_lock<std::mutex> lock(mutex_Align_IMU);
        return mqNotAlignedImu.front();
    }
    uint32_t CamImuAligner::ProessImg(double camTimeStamp,uint32_t imgCount)
    {
        int rightImgCount = imgCount - 1;
        if(rightImgCount < 0)
        {
            rightImgCount += 21;
        }

        double curImgTimeStamp = camTimeStamp;//ns转变成s

        //判断图像时间间隔
        if(curImgTimeStamp > mPreImgTime && this->mnFrameId > 0)
        {
            double realImgInverval = curImgTimeStamp - mPreImgTime;//计算图像间隔
            int nPossibleCountInc = realImgInverval / mMinImgInverval;

            //计算count间隔
            if((uint32_t)rightImgCount !=  mnLastImgCount)
            {
                int nCountDiff = rightImgCount - mnLastImgCount;
                if(nCountDiff < 0)
                {
                    nCountDiff = rightImgCount + 21 - mnLastImgCount;
                }

                //防止出现count号没到20就清零的情况
                if(abs(nCountDiff) > 1 && nPossibleCountInc == 1)
                {
                    rightImgCount = mnLastImgCount + 1;
                    if(rightImgCount > 20)
                    {
                        rightImgCount = 21 - rightImgCount;
                    }
                }
            }
            mnLastImgCount = rightImgCount;
            
        }
        else if(curImgTimeStamp > mPreImgTime)
        {
            this->mnLastImgCount = rightImgCount;
        }
        else
        {
            printf("图像时间戳错误\n");
        }

        this->mPreImgTime = curImgTimeStamp;
        return rightImgCount;
    }

    std::vector<ImuData> CamImuAligner::GetAlignImuData(double camTimeStamp,uint32_t imgCount)
    {
        std::vector<ImuData> vRetIMU;

        double curImgTimeStamp = camTimeStamp / (double)(1.0e9);
        uint32_t rightImgCount = ProessImg(curImgTimeStamp,imgCount);
        this->mbImuTimeLess = false;
        while(mnFrameId > 1 && !mqNotAlignedImu.empty())
        {
            ImuData_NotAligned nAImuData;
            nAImuData = GetFrontNotAlignedImu();

            ImuData aImuData;
            aImuData.am = nAImuData.am;
            aImuData.wm = nAImuData.wm;
            double curImuTimeStamp = nAImuData.time_stamp / (double)(1.0e9);

            if(nAImuData.sync_count != mnLastImuCountNotRec)
            {
                this->mCountStartImuTime = curImuTimeStamp;
            }

            int nNewCount = nAImuData.sync_count ;
            uint32_t nOriCount = nAImuData.sync_count ;
            if(curImuTimeStamp > mPreImuTime)
            {
                if(mbFirstImu)//防止第一个IMU对应的mPreImuTime没有值的情况
                {
                    mPreImuTime = curImuTimeStamp;
                    mbFirstImu = false; 
                }

                bool bStatus = (nAImuData.sync_count == 0 && this->mnPrevImuCount == 20);
                if(!bStatus)
                {
                    mRealImuInverval = curImuTimeStamp - mPreImuTime;
                    int nPossibleCountInc = mRealImuInverval / mMinImuInverval;
                    
                    if(mRealImuInverval > mMaxImuInverval && mRealImuInverval / mMinImuInverval >= 2)
                    {
                        //存在图像丢失的情况
                        mbImuInterrupt = true;
                    }
                    
                    
                    if(nAImuData.sync_count != mnLastImuCountNotRec)
                    {
                        if(nAImuData.sync_count != mnLastImuCount && mnLastImuCount >= 0)
                        {
                            //预防出现，时间戳间隔是对的，但是count不对的情况
                            int nCountDiff = nAImuData.sync_count - mnLastImuCount;
                            if(nCountDiff < 0)
                            {
                                nCountDiff = nAImuData.sync_count + 21 - mnLastImuCount;
                            }
                            if(abs(nCountDiff) > 1 && nPossibleCountInc == 1)
                            {
                                nNewCount = mnLastImuCount + 1;
                                if(nAImuData.sync_count > 20)
                                {
                                    nNewCount = 21 - nAImuData.sync_count;
                                }
                            }
                        }
                    }
                    else
                    {
                        nNewCount = mnLastImuCount;
                    }  
                }

                this->mnPrevImuCount = nOriCount;
                mnLastImuCountNotRec = nAImuData.sync_count;
                nAImuData.sync_count = nNewCount;
                mnLastImuCount =nAImuData.sync_count;
                
            }
            else
            {
                //错误
            }

            
            if(rightImgCount  == nAImuData.sync_count)
            {
                //永远都是和图像count相同的第一个IMU
                if(mbIsFirstImg)
                {
                    //第一张图，无论如何和第一个相同的count号对齐
                    mbIsFirstImg = false;

                    //计算图像和IMU之间的时间差
                    mImgImuTimeDiff = curImgTimeStamp - curImuTimeStamp;
                    mImuSegStartTime = curImuTimeStamp;
                    
                    //IMU时间更改为图像的时间
                    aImuData.timestamp = curImgTimeStamp;
                    vRetIMU.push_back(aImuData);

                    mPreImuTime = curImuTimeStamp;

                    PopNotAlignedImu();
                    
                    break;
                }
                else
                {
                    
                    while(mlOutdatedImu.size() > 0)
                    {
                        ImuData_NotAligned tmpNaImuData = mlOutdatedImu.front();
                        mlOutdatedImu.pop_front();
                        ImuData tmpAImuData;
                        tmpAImuData.am = tmpNaImuData.am;
                        tmpAImuData.wm = tmpNaImuData.wm;
                        double tmpCurImuTimeStamp = tmpNaImuData.time_stamp / (double)(1.0e9);
                        tmpAImuData.timestamp = tmpCurImuTimeStamp + mImgImuTimeDiff;
                        mLastImuAlignedTime = tmpAImuData.timestamp;
                        vRetIMU.push_back(tmpAImuData);
                    }
                    

                    //判断和前一个IMU的时间差
                    // double curImuTimeDiff = curImuTimeStamp - mPreImuTime;
                    double predictImuTime = curImuTimeStamp + mImgImuTimeDiff;
                    double predictTimeDiff = curImgTimeStamp - predictImuTime;
                    if(curImgTimeStamp < mLastImuAlignedTime)
                    {
                        //可能出现了问题，例如同一个count的IMU个数超过规定的个数，如果强行对齐，就会出现IMU时间差为负数的情况
                        mImuSegStartTime = curImuTimeStamp;
                        //IMU时间更改为图像时间
                        aImuData.timestamp = predictImuTime;
                        vRetIMU.push_back(aImuData);

                        mPreImuTime = curImuTimeStamp;
                        PopNotAlignedImu();
                        
                        mbImuTimeLess = mImgImuTimeDiff > 0;
                        mLastImuAlignedTime = aImuData.timestamp;
                        break;

                    }
                    else if(fabs(predictTimeDiff) < mImgInverval)
                    {
                        //计算图像和IMU之间的时间差
                        mImgImuTimeDiff  = curImgTimeStamp - curImuTimeStamp;
                        mImuSegStartTime = curImuTimeStamp;
                        //IMU时间更改为图像时间
                        aImuData.timestamp = curImgTimeStamp;
                        vRetIMU.push_back(aImuData);

                        mPreImuTime = curImuTimeStamp;

                        PopNotAlignedImu();
                        
                        mbImuTimeLess = mImgImuTimeDiff > 0;
                        mLastImuAlignedTime = aImuData.timestamp;
                        break;
                    }
                    else
                    {
                        if(curImgTimeStamp > predictImuTime)
                        {
                            //图像时间比IMU预测时间大，说明从上一帧对齐后，出现了图像和IMU count相等，但是时间不对齐的情况；
                            //原因可能是图像或者IMU丢失了，则使用上一次对齐的数据，IMU继续往后，直到找到差距最小的
                            aImuData.timestamp = predictImuTime;
                            vRetIMU.push_back(aImuData);

                            mPreImuTime = curImuTimeStamp;

                            {
                                std::unique_lock<std::mutex> lock(mutex_Align_IMU);
                                mqNotAlignedImu.pop();
                            }
                            
                            mbImuTimeLess = true;
                            mPreImgImuTimeDiff = curImgTimeStamp - predictImuTime;
                            mLastImuAlignedTime = aImuData.timestamp;
                        }
                        else
                        {
                            //IMU预测时间比图像时间大，则可能是MCU没收到硬件同步信号
                            double imuSegTime = curImuTimeStamp - mImuSegStartTime;
                            int nImuSegImgFrames = (int)(imuSegTime / mImgInverval); 
                            if(nImuSegImgFrames > 1)
                            {
                                //MCU确实没收到触发信号，导致大量的IMU数据使用同一个count，跳过了多帧图像，则保持IMU不变，获取新的图像
                                break;
                            }
                            if(mbImuTimeLess)
                            {
                                mlOutdatedImu.push_back(nAImuData);

                                mPreImuTime = curImuTimeStamp;

                                {
                                    std::unique_lock<std::mutex> lock(mutex_Align_IMU);
                                    mqNotAlignedImu.pop();
                                }
                                
                                
                                if(imuSegTime > mMaxImgInverval)
                                {
                                    //很可能就是MCU没有收到触发信号
                                    //则以lImuGreater的第一帧IMU为对齐时间,即预测时间大于图像时间的第一帧IMU
                                    if(mlOutdatedImu.size() > 0)
                                    {
                                        ImuData_NotAligned tmpNaImuData = mlOutdatedImu.front();
                                        mlOutdatedImu.pop_front();
                                        ImuData tmpAImuData;
                                        tmpAImuData.am = tmpNaImuData.am;
                                        tmpAImuData.wm = tmpNaImuData.wm;
                                        double tmpCurImuTimeStamp = tmpNaImuData.time_stamp / (double)(1.0e9);
                                        tmpAImuData.timestamp = curImgTimeStamp;
                                        vRetIMU.push_back(tmpAImuData);
                                        mImgImuTimeDiff = curImgTimeStamp - tmpCurImuTimeStamp;
                                        mImuSegStartTime = tmpCurImuTimeStamp;
                                        mLastImuAlignedTime = tmpNaImuData.time_stamp;
                                    }
                                    while(mlOutdatedImu.size() > 0)
                                    {
                                        ImuData_NotAligned tmpNaImuData = mlOutdatedImu.front();
                                        mlOutdatedImu.pop_front();
                                        ImuData tmpAImuData;
                                        tmpAImuData.am = tmpNaImuData.am;
                                        tmpAImuData.wm = tmpNaImuData.wm;
                                        double tmpCurImuTimeStamp = tmpNaImuData.time_stamp / (double)(1.0e9);
                                        tmpAImuData.timestamp = tmpCurImuTimeStamp + mImgImuTimeDiff;
                                        vRetIMU.push_back(tmpAImuData);
                                        mLastImuAlignedTime = tmpNaImuData.time_stamp;
                                    }


                                    mbImuTimeLess = false;
                                    continue;;
                                }

                                //虽然IMU预测的时间已经超过图像了，但是还没超过最大间隔
                                continue;
                            }
                        }//if(curImgTimeStamp > predictImuTime)

                    }
                    
                }
                
                //图像和IMU count号对齐
            }
            else 
            {
                uint32_t tmpRightCount = rightImgCount;
                if(nAImuData.sync_count - tmpRightCount > 5 && tmpRightCount + 21 - nAImuData.sync_count < 5  )
                {
                    tmpRightCount += 21;
                }

                if(tmpRightCount < nAImuData.sync_count)
                {
                    if(mbIsFirstImg)
                    {
                        //第一张图像，由于无法比较图像和IMU时间的大小，认为图像比IMU时间小
                        break;
                    }
                    double predictImuTime = curImuTimeStamp + mImgImuTimeDiff;
                    double predictTimeDiff = curImgTimeStamp - predictImuTime;
                    if(predictTimeDiff < 0.0)
                    {
                        //图像时间比IMU时间小,则该IMU不是该图像对齐的，图像时间靠前，图像跳过当前一帧
                        break;
                    }
                    else
                    {
                        //图像比IMU时间靠后，则IMU是之前的数据，则全部按照之前的推算预测,这种情况，可能是图像比IMU快得较多
                        aImuData.timestamp = predictImuTime;
                        vRetIMU.push_back(aImuData);

                        mPreImuTime = curImuTimeStamp;

                        PopNotAlignedImu();
                        
                        mbImuTimeLess = true;
                        mPreImgImuTimeDiff = curImgTimeStamp - predictImuTime;
                        mLastImuAlignedTime = aImuData.timestamp;
                    }
                }
                else//(tmpRightCount > nAImuData.sync_count)
                {
                    //图像的count比IMU的count大
                    if(mbIsFirstImg)
                    {
                        //抛弃当前的IMU
                        
                        PopNotAlignedImu();
                        mPreImuTime = curImuTimeStamp;
                        continue;
                    }
                    else
                    {
                        //不是第一帧图像，则对IMU时间进行预测
                        double predictImuTime = curImuTimeStamp + mImgImuTimeDiff;
                        double predictTimeDiff = curImgTimeStamp - predictImuTime;
                        
                        //比较图像和IMU预测时间之间的大小，分大小测试
                        if(predictTimeDiff > 0.0)
                        {
                            //IMU继续往后，直到找到差距最小的
                            aImuData.timestamp = predictImuTime;
                            vRetIMU.push_back(aImuData);

                            mPreImuTime = curImuTimeStamp;

                            PopNotAlignedImu();
                            
                            mbImuTimeLess = true;
                            mPreImgImuTimeDiff = curImgTimeStamp - predictImuTime;
                            mLastImuAlignedTime = aImuData.timestamp;
                        }
                        else//IMU预测时间比图像时间大
                        {
                            double imuSegTime = curImuTimeStamp - mImuSegStartTime;
                            int nImuSegImgFrames = (int)(imuSegTime / mImgInverval); 
                            if(nImuSegImgFrames > 1)
                            {
                                //MCU确实没收到触发信号，且跳过了多帧图像，则保持IMU不变，获取新的图像
                                break;
                            }
                            if(mbImuTimeLess)
                            {
                                mlOutdatedImu.push_back(nAImuData);

                                mPreImuTime = curImuTimeStamp;

                                PopNotAlignedImu();
                                
                                if(imuSegTime > mMaxImgInverval)
                                {
                                    //很可能就是MCU没有收到触发信号
                                    //则以lImuGreater的第一帧IMU为对齐时间,即预测时间大于图像时间的第一帧IMU
                                    if(mlOutdatedImu.size() > 0)
                                    {
                                        ImuData_NotAligned tmpNaImuData = mlOutdatedImu.front();
                                        mlOutdatedImu.pop_front();
                                        ImuData tmpAImuData;
                                        tmpAImuData.am = tmpNaImuData.am;
                                        tmpAImuData.wm = tmpNaImuData.wm;
                                        double tmpCurImuTimeStamp = tmpNaImuData.time_stamp / (double)(1.0e9);
                                        tmpAImuData.timestamp = tmpCurImuTimeStamp + mImgImuTimeDiff;
                                        vRetIMU.push_back(tmpAImuData);
                                        mImuSegStartTime = tmpCurImuTimeStamp;
                                        mLastImuAlignedTime = tmpAImuData.timestamp;
                                    }
                                    while(mlOutdatedImu.size() > 0)
                                    {
                                        ImuData_NotAligned tmpNaImuData = mlOutdatedImu.front();
                                        mlOutdatedImu.pop_front();
                                        ImuData tmpAImuData;
                                        tmpAImuData.am = tmpNaImuData.am;
                                        tmpAImuData.wm = tmpNaImuData.wm;
                                        double tmpCurImuTimeStamp = tmpNaImuData.time_stamp / (double)(1.0e9);
                                        tmpAImuData.timestamp = tmpCurImuTimeStamp + mImgImuTimeDiff;
                                        vRetIMU.push_back(tmpAImuData);
                                        mLastImuAlignedTime = tmpAImuData.timestamp;
                                    }


                                    mbImuTimeLess = false;
                                    break;
                                }

                                //虽然IMU预测的时间已经超过图像了，但是还没超过最大间隔
                                continue;
                            }
                        }//if(curImgTimeStamp > predictImuTime)
                    }//if(mbIsFirstImg)
                }//if(tmpRightCount < nAImuData.sync_count)
            }//if(rightImgCount  == nAImuData.sync_count)
            
        }//while(!qImu.empty())

        this->mnFrameId++;
        
        return vRetIMU;
    }
}