#ifndef JARVIS_GPU_PYRAMID_COMPUTE
#define JARVIS_GPU_PYRAMID_COMPUTE
#include <vector>

#include "opencl_handler.h"
#include "opencv2/opencv.hpp"

namespace jarvis_pic {
class OpenCLHandler;
std::vector<cv::Mat> BuildPyramidsUsingGPU(const cv::Mat&,
                                           OpenCLHandler* opencl_handler_,
                                           const void* option);

std::vector<cv::Mat> BuildPyramidsUingGPUWithBorder(
    const cv::Mat&, OpenCLHandler* opencl_handler_, const void* option);
}  // namespace jarvis_pic
#endif