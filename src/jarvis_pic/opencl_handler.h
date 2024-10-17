#ifndef JARVIS_PIC_OPENCL_HANDLER_H
#define JARVIS_PIC_OPENCL_HANDLER_H
#define CL_TARGET_OPENCL_VERSION 300
#include <fstream>
#include <iostream>
#include <string>

#include <CL/cl.h>
#include "opencv2/core/core.hpp"
namespace jarvis_pic{
#define CHECK_ERROR(err)                                    \
  if (err != CL_SUCCESS) {                                  \
    fprintf(stderr, "Line: %d Error: %d\n", __LINE__, err); \
    exit(EXIT_FAILURE);                                     \
  }

typedef int16_t deriv_type;

class OpenCLHandler {
 public:
  OpenCLHandler();
  ~OpenCLHandler();
  void initOpenCL(const std::string &kernelFile);
  void executeKernel(const cv::Mat &img, int level,
                     std::vector<cv::Mat> &pyramids);
  void executeKernelWithBorder(const cv::Mat &img, int level,
                               std::vector<cv::Mat> &pyramids,
                               cv::Size winSize);
  void loadData(const cv::Mat &img, std::vector<cv::Mat> &pyramids);
  void loadDataWithBorder(const cv::Mat &img, std::vector<cv::Mat> &pyramids);

 private:
  cl_context context;
  cl_command_queue queue;
  cl_program program;
  cl_kernel kernelDownSampling, kernelCalcDeriv, kernelDownSamplingWithBorder,
      kernelCalcDerivWithBorder;
  cl_mem *buffers, *buffers_deriv;
  float *E_ptr;
  cl_int err;
  int level_;
  int *width_, *width_with_border_;
  int *height_, *height_with_border_;
  cv::Size winSize_;

  std::string loadKernelSource(const std::string &filename);
  void printPlatformInfo(cl_platform_id platform);
  void printDeviceInfo(cl_device_id device);
};
}
#endif