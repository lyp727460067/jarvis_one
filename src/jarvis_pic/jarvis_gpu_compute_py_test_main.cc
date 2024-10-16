#include <dirent.h>
#include <sys/types.h>

#include <condition_variable>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fstream"
#include "glog/logging.h"
#include "glog_sink.h"
#include "jarvis/common/fixed_ratio_sampler.h"
#include "jarvis/common/time.h"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "jarvis_brige.h"
#include "mutex"
#include "ostream"
#include "slip_detection/slip_detect.h"
#include "time.h"
#include "unistd.h"
//
#include "CL/cl.h"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
namespace {}  // namespace
//

std::string pyramidKernelSource =
    R"(__kernel void pyramid_convolution(__global const uint8*A,
                                  __global uint8 const* B,const int width) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  // int s_x = (x * 2);
  // int s_y = (y * 2);
  // int s_width = width;
  // uint8  v= (A[s_y * s_width + s_x] + A[s_y * s_width + s_x + 1] +
  //            A[(s_y + 1) * s_width + s_x] + A[(s_y + 1) * s_width +s_x+1]) >>
  //         2;
  B[y * width+x] = 254;
}
)";
inline void checkErr(cl_int err, const char *name) {
  if (err != CL_SUCCESS) {
    std::cerr << "ERROR: " << name << " (" << err << ")" << std::endl;
    exit(EXIT_FAILURE);
  }
}
//
int main(int argc, char *argv[]) {
  //
  //   InitOpenCL(kernelFile);
  //
  const std::string input_file(argv[1]);
  const std::string outputput_file_dir(argv[2]);
  cl_int errNum;
  cl_uint numPlatforms;
  cl_uint numDevices;
  cl_platform_id *platformIDs;
  cl_context context = NULL;
  cl_command_queue queue;
  cl_program program;
  cl_kernel kernel;

  errNum = clGetPlatformIDs(0, NULL, &numPlatforms);
  checkErr(
      (errNum != CL_SUCCESS) ? errNum : (numPlatforms <= 0 ? -1 : CL_SUCCESS),
      "clGetPlatformIDs");

  cl_device_id *deviceIDs = NULL;
  cl_uint i;
  for (i = 0; i < numPlatforms; i++) {
    errNum = clGetDeviceIDs(platformIDs[i], CL_DEVICE_TYPE_CPU, 0, NULL,
                            &numDevices);
    if (errNum != CL_SUCCESS && errNum != CL_DEVICE_NOT_FOUND) {
      checkErr(errNum, "clGetDeviceIDs");
    } else if (numDevices > 0) {
      deviceIDs = (cl_device_id *)alloca(sizeof(cl_device_id) * numDevices);
      break;
    }
  }

  cl_context_properties contextProperties[] = {
      CL_CONTEXT_PLATFORM, (cl_context_properties)platformIDs[i], 0};
  context = clCreateContext(contextProperties, numDevices, deviceIDs, nullptr,
                            NULL, &errNum);
  checkErr(errNum, "clCreateContext");

  std::ifstream srcFile("Convolution.cl");
  std::string srcProg(std::istreambuf_iterator<char>(srcFile),
                      (std::istreambuf_iterator<char>()));

  const char *src = srcProg.c_str();
  size_t length = srcProg.length();

  program = clCreateProgramWithSource(context, 1, &src, &length, &errNum);
  checkErr(errNum, "clCreateProgramWithSource");

  errNum = clBuildProgram(program, numDevices, deviceIDs, NULL, NULL, NULL);
  checkErr(errNum, "clBuildProgram");

  kernel = clCreateKernel(program, "convolve", &errNum);
  checkErr(errNum, "clCreateKernel");

  checkErr(errNum, "clCreateBuffer(mask)");
  //
  queue = clCreateCommandQueue(context, deviceIDs[0], 0, &errNum);
  checkErr(errNum, "clCreateCommandQueue");
  //
  // Create kernels
  cl_kernel kernel_convolution =
      clCreateKernel(program, "pyramid_convolution", nullptr);

  const cv::Mat image = cv::imread(input_file, cv::IMREAD_GRAYSCALE);
  cl_mem bufferA = clCreateBuffer(
      context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, image.cols * image.rows,
      const_cast<uint8_t *>(image.data), nullptr);

  //
  cv::Mat out_put(cv::Size(image.cols/2, image.rows/2), CV_8UC1);
  size_t globalWorkSize[2] = {size_t(out_put.cols), size_t(out_put.rows)};

  cl_mem bufferB =
      clCreateBuffer(context, CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR,
                     globalWorkSize[0] * globalWorkSize[1],
                     const_cast<uint8_t *>(out_put.data), nullptr);

  int width = out_put.rows;
  clSetKernelArg(kernel_convolution, 0, sizeof(cl_mem), &bufferA);
  clSetKernelArg(kernel_convolution, 1, sizeof(cl_mem), &bufferB);
  clSetKernelArg(kernel_convolution, 2, sizeof(int), &width);
  clEnqueueNDRangeKernel(queue, kernel_convolution, 2, nullptr,
                         &globalWorkSize[0], nullptr, 0, nullptr, nullptr);

  clFinish(queue);
  // std::cout<<out_put<<std::endl;
  clEnqueueReadBuffer(queue, bufferB, CL_TRUE, 0, out_put.cols * out_put.rows,
                      out_put.data, 0, nullptr, nullptr);

  // std::cout<<out_put<<std::endl;
  cv::imwrite(outputput_file_dir + "1.png", out_put);
  return 0;

  //
}