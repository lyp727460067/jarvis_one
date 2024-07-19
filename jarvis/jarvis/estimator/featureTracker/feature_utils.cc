/******************************************************************************
 * Copyright 2017 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "feature_utils.h"

#include <math.h>

#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <iostream>
#include <mutex>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_set>

#ifndef __DEVELOPMENT_DEBUG_MODE__
#define __FEATURE_UTILS_NO_DEBUG__
#endif
// #define VERIFY_NEON

namespace XP {
void build_pyramids(const cv::Mat& img, int max_level,
                    std::vector<cv::Mat>* _pyramids,
                    uchar* const pyra_buf_ptr) {
  CHECK_NOTNULL(_pyramids);
  std::vector<cv::Mat>& pyramids = *_pyramids;
  pyramids.resize(max_level + 1);
  const int size = img.rows * img.cols;
  // no fixed buffer specified, will allocate memory dynamically
  if (pyra_buf_ptr == nullptr) {
    pyramids[0] = img.clone();
    for (int i = 1; i <= max_level; ++i) {
      pyramids[i] = fast_pyra_down(pyramids[i - 1]);
    }
  } else {
    // fixed buffer provided
    // the pyramids kept in the fixed buffer in this way:
    // level n |  level n-1 |  level 1  |  level 0
    // so we can use them in a very cache-friendly way
    // and no need to call malloc
    for (int lvl = 0; lvl <= max_level; ++lvl) {
      int offset = 0;
      // compute pyramid start address
      for (int i = lvl + 1; i <= max_level; ++i) {
        offset += size >> (2 * i);
      }
      if (lvl != 0) {
        pyramids[lvl] =
            fast_pyra_down(pyramids[lvl - 1], pyra_buf_ptr + offset);
      } else {
        cv::Mat tmp(img.rows, img.cols, img.type(), pyra_buf_ptr + offset);
        img.copyTo(tmp);
        pyramids[lvl] = tmp;
      }
    }
  }
}

}  // namespace XP
