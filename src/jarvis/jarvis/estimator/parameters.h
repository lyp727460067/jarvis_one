#ifndef JARVIS_ESTIMATOR_PARAMETERS_H
#define JARVIS_ESTIMATOR_PARAMETERS_H
#include "Eigen/Dense"
#include <fstream>
#include <map>
// #include "opencv2/core/eigen.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

#include "../utility/utility.h"

constexpr uint8_t kGlogLevel = 0;
constexpr uint8_t kGlogCostTimeLevel = 1;
constexpr uint8_t kGlogCeresLevel = 0;
namespace jarvis {
namespace estimator {


// constexpr double FOCAL_LENGTH = 377.0;
constexpr int WINDOW_SIZE = 6;
// constexpr int NUM_OF_F = 1000;
enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };
enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };
enum SIZE_PARAMETERIZATION {
  SIZE_POSE = 7,
  SIZE_SPEEDBIAS = 9,
  SIZE_FEATURE = 1
};
}  // namespace vins
}  // namespace internal

#endif