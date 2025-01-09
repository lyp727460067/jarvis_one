#ifndef __JARVIS_MAPPING_DES__DATA_TYPE_H
#define __JARVIS_MAPPING_DES__DATA_TYPE_H

#include <bitset>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "glog/logging.h"
#include "opencv2/opencv.hpp"
namespace jarvis {
namespace mapping {

constexpr int kBitSetLenth = 256;
using BrifBitset = std::bitset<kBitSetLenth>;
using Descriptor = BrifBitset;
using Descriptors = std::vector<BrifBitset>;

//

//
inline int HammingDis(const BrifBitset& l, const BrifBitset& r) {
  return (l ^ r).count();
}
//

//

template <typename BitType, int Lenth = kBitSetLenth>
BitType CvMatToBitSet(const cv::Mat& bytes) {
  BitType result(Lenth);
  int j = 0;
  for (int index = 0; index < bytes.cols; index++) {
    const uint8_t& byte = bytes.at<uint8_t>(index);
    for (int i = 0; i < 8; i++) {
      if (byte & (0x01 << i)) {
        result.set(j);
      } else {
        result.reset(j);
      }
      j++;
    }
  }
  return result;
}
//
template <typename BitType, int Lenth = kBitSetLenth>
BitType Uint8ToBitSet(const std::vector<uint8_t>& bytes) {
  BitType result(Lenth);
  int j = 0;
  for (const auto& byte : bytes) {
    for (int i = 0; i < 8; i++) {
      if (byte & (0x80 >> i)) {
        result.set(j);
      } else {
        result.reset(j);
      }
      j++;
    }
  }
  return result;
}
//
//
template <typename BitType, int Lenth = kBitSetLenth>
std::vector<uint8_t> BitSetToUint8(const BitType& bit) {
  std::vector<uint8_t> result;
  for (int i = 0; i < Lenth; i += 8) {
    uint8_t byte = 0;
    for (int j = 0; j < 8; j++) {
      if (bit[i + j]) {
        byte |= (0x01 << (7 - j));
      }
    }
    result.push_back(byte);
  }
  return result;
}
//

inline std::vector<BrifBitset> CvMatToBrief(const cv::Mat& desci) {
  CHECK(!desci.empty());
  std::vector<BrifBitset> brief;
  for (int i = 0; i < desci.rows; i++) {
    CHECK(!desci.row(i).empty());
    brief.push_back(CvMatToBitSet<BrifBitset>(desci.row(i)));
  }
  return brief;
}

//
inline Eigen::Vector2f CvKeyPointToEigen(const cv::KeyPoint& kp) {
  return Eigen::Vector2f{kp.pt.x, kp.pt.y};
}

//
}  // namespace mapping
}  // namespace jarvis
#endif