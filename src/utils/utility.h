#ifndef UTILITY_H
#define UTILITY_H

namespace VSLAM 
{

    #define double_equal(a,b) (abs(a-b) < 1e-8)

    #define us2s(a) (a * 1e-6)  //us转换成s
    #define s2us(a) (a * 1e6)  //s转换成us
    #define us2ns(a) (a * 1e3)  //us转换成ns
    #define s2ns(a) (a * 1e9)  //s转换成ns
    
    typedef struct DoubleEyeFrame 
    {
        CameraFrameHead head;
        uint32_t max_len{0};
        uint8_t *buf{nullptr};
    }DoubleEyeFrame;
}
#endif