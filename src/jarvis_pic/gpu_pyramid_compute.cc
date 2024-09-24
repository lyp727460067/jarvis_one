#include "gpu_pyramid_compute.h"
#include "opencl_handler.h"
#include "jarvis/estimator/featureTracker/pyramid_image.h" 
namespace jarvis_pic {

using namespace jarvis;
using namespace estimator;
//
std::vector<cv::Mat> BuildPyramidsUsingGPU(const cv::Mat& img,
                                           OpenCLHandler* opencl_handler_,
                                           const void* option) {
  auto& option_ = *reinterpret_cast<const PyramidImageOption*>(option);
  //   CHECK_NOTNULL(_pyramids);

  std::vector<cv::Mat> pyramids;
  std::vector<cv::Mat> pyramids_temp;
  pyramids.resize((option_.layer + 1) * 2);
  cv::Size winSize(option_.lk_win_size, option_.lk_win_size);

  opencl_handler_->executeKernel(img, option_.layer, pyramids_temp);

  TicToc timer;
  for (int i = 0; i <= option_.layer; ++i) {
    cv::Size sz = pyramids_temp[i * 2].size();
    pyramids[i * 2].create(sz.height + winSize.height * 2,
                           sz.width + winSize.width * 2, img.type());
    copyMakeBorder(pyramids_temp[i * 2], pyramids[i * 2], winSize.height,
                   winSize.height, winSize.width, winSize.width, 4);
    pyramids[i * 2].adjustROI(-winSize.height, -winSize.height, -winSize.width,
                              -winSize.width);

    pyramids[i * 2 + 1].create(sz.height + winSize.height * 2,
                               sz.width + winSize.width * 2, img.type());
    copyMakeBorder(pyramids_temp[i * 2 + 1], pyramids[i * 2 + 1],
                   winSize.height, winSize.height, winSize.width, winSize.width,
                   4);
    pyramids[i * 2 + 1].adjustROI(-winSize.height, -winSize.height,
                                  -winSize.width, -winSize.width);
  }
  return pyramids;
}

std::vector<cv::Mat> BuildPyramidsUingGPUWithBorder(
    const cv::Mat& img, OpenCLHandler* opencl_handler_, const void* option) {
  auto& option_ = *reinterpret_cast<const PyramidImageOption*>(option);
  std::vector<cv::Mat> pyramids;
  cv::Size winSize(option_.lk_win_size, option_.lk_win_size);

  opencl_handler_->executeKernelWithBorder(img, option_.layer, pyramids,
                                           winSize);

  TicToc timer;
  for (int i = 0; i <= option_.layer; ++i) {
    pyramids[i * 2].adjustROI(-winSize.height, -winSize.height, -winSize.width,
                              -winSize.width);
    pyramids[i * 2 + 1].adjustROI(-winSize.height, -winSize.height,
                                  -winSize.width, -winSize.width);
  }
  double duration = timer.toc();
  std::cout << "adjust: " << duration << std::endl;
  return  pyramids;
}

//
}  // namespace jarvis_pic