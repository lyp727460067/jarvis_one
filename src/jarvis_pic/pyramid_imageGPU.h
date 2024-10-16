#ifndef JARVIS_ESTIMATOR_PYRAMID_IMAGE_H
#define JARVIS_ESTIMATOR_PYRAMID_IMAGE_H
#include "jarvis/sensor/image_data.h"
//
//
#include <execinfo.h>

#include "Eigen/Dense"
#include <csignal>
#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>

#include "jarvis/utility/tic_toc.h"
namespace jarvis {
namespace estimator {

//

void ComputePyramidImage(int level, sensor::ImageData& image);

}  // namespace estimator
}  // namespace jarvis
#endif