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
#ifndef XP_INCLUDE_XP_UTIL_FEATURE_UTILS_H_
#define XP_INCLUDE_XP_UTIL_FEATURE_UTILS_H_

#include <glog/logging.h>

#include <map>
#include <mutex>
#include <opencv2/video/tracking.hpp>
#include <random>
#include <vector>
namespace XP {
#ifndef __ARM_NEON__
/******************************************************************************
 * fast_pyra_down_internal       This interface should not used directly
 * @param[in]   img_in_smooth    input image
 * @param[out]  _img_in_small    output image, the memory of _img_in_small must
 *                               be pre-allocated before calling this function
 * TODO(yanghongtian) : use SSE intrinsics here
 */
inline void fast_pyra_down_internal(const cv::Mat& img_in_smooth,
                                    cv::Mat* _img_in_small) {
  CHECK_NOTNULL(_img_in_small);
  CHECK_NOTNULL(_img_in_small->data);
  CHECK_EQ(img_in_smooth.type(), CV_8U);
  CHECK_EQ(img_in_smooth.rows & 1, 0);
  CHECK_EQ(img_in_smooth.cols & 3, 0);
  cv::Mat& img_in_small = *_img_in_small;

  // use our own pyra down for faster performance
  const int width_step_in = img_in_smooth.step1();
  int sum0, sum1, sum2, sum3;
  const uchar* data0_ptr = img_in_smooth.data;
  const uchar* data1_ptr = img_in_smooth.data + width_step_in;
  uchar* target_data_ptr = img_in_small.data;

  for (int rows = img_in_small.rows, y = 0; y < rows; ++y) {
    for (int cols = img_in_small.cols & ~3, x = 0; x < cols;
         x += 4, target_data_ptr += 4) {
      sum0 =
          ((static_cast<int>(*data0_ptr) + static_cast<int>(*(data0_ptr + 1))) +
           (static_cast<int>(*data1_ptr) +
            static_cast<int>(*(data1_ptr + 1)))) /
          4;
      sum1 = ((static_cast<int>(*(data0_ptr + 2)) +
               static_cast<int>(*(data0_ptr + 3))) +
              (static_cast<int>(*(data1_ptr + 2)) +
               static_cast<int>(*(data1_ptr + 3)))) /
             4;
      sum2 = ((static_cast<int>(*(data0_ptr + 4)) +
               static_cast<int>(*(data0_ptr + 5))) +
              (static_cast<int>(*(data1_ptr + 4)) +
               static_cast<int>(*(data1_ptr + 5)))) /
             4;
      sum3 = ((static_cast<int>(*(data0_ptr + 6)) +
               static_cast<int>(*(data0_ptr + 7))) +
              (static_cast<int>(*(data1_ptr + 6)) +
               static_cast<int>(*(data1_ptr + 7)))) /
             4;

      *(target_data_ptr) = static_cast<uchar>(sum0);
      *(target_data_ptr + 1) = static_cast<uchar>(sum1);
      *(target_data_ptr + 2) = static_cast<uchar>(sum2);
      *(target_data_ptr + 3) = static_cast<uchar>(sum3);
      data0_ptr += 8;
      data1_ptr += 8;
    }
    data0_ptr += width_step_in;
    data1_ptr += width_step_in;
  }
}
#else
inline void fast_pyra_down_internal(const cv::Mat& img_in_smooth,
                                    cv::Mat* _img_in_small) {
  CHECK_NOTNULL(_img_in_small);
  CHECK_NOTNULL(_img_in_small->data);
  CHECK_EQ(img_in_smooth.type(), CV_8U);
  CHECK_EQ(img_in_smooth.rows & 1, 0);

  cv::Mat& img_in_small = *_img_in_small;

  // use our own pyra down for faster performance
  const int width_step_in = img_in_smooth.step1();
  const int width_step_small = img_in_small.step1();
  uchar* target_data_ptr = img_in_small.data;
  uchar* data0_ptr = img_in_smooth.data;
  uchar* data1_ptr = img_in_smooth.data + width_step_in;

  // provide hits to gcc for optimization
  const int cols = img_in_small.cols - 8;
  for (int rows = img_in_small.rows, y = 0; y < rows; ++y) {
    int x = 0;
    for (; x <= cols; x += 8, target_data_ptr += 8) {
      // loading 32 pixels per row from source image
      uint8x8x2_t row0 = vld2_u8(data0_ptr);
      uint8x8x2_t row1 = vld2_u8(data1_ptr);
      // compute 16 target pixels per loop
      uint16x8_t sum = vaddq_u16(vaddl_u8(row0.val[0], row0.val[1]),
                                 vaddl_u8(row1.val[0], row1.val[1]));
      vst1_u8(target_data_ptr, vmovn_u16(vshrq_n_u16(sum, 2)));
      data0_ptr += 16;
      data1_ptr += 16;
    }
    for (; x < img_in_small.cols; ++x) {
      int sum =
          ((static_cast<int>(*data0_ptr) + static_cast<int>(*(data0_ptr + 1))) +
           (static_cast<int>(*data1_ptr) +
            static_cast<int>(*(data1_ptr + 1)))) /
          4;
      *target_data_ptr++ = static_cast<uchar>(sum);
      data0_ptr += 2;
      data1_ptr += 2;
    }
    data0_ptr += width_step_in;
    data1_ptr += width_step_in;
  }
}
#endif  // __ARM_NEON__

/**************************************************************************************
 * fast_pyra_down : used this function for half-down sample image
 * @param[in]  img_in_smooth  input image
 * @param[in]  data           existence storage for the downsample images
 *                            default is nullptr, don't use existence
 * @return   half-down sample image.
 */
inline cv::Mat fast_pyra_down(const cv::Mat& img_in_smooth,
                              uchar* const data = nullptr) {
  constexpr int compress_ratio = 2;
  if (data == nullptr) {
    cv::Mat img_in_small(img_in_smooth.rows / compress_ratio,
                         img_in_smooth.cols / compress_ratio, CV_8U);
    fast_pyra_down_internal(img_in_smooth, &img_in_small);
    return img_in_small;
  } else {
    cv::Mat img_in_small(img_in_smooth.rows / compress_ratio,
                         img_in_smooth.cols / compress_ratio, CV_8U, data);
    fast_pyra_down_internal(img_in_smooth, &img_in_small);
    return img_in_small;
  }
}
/**************************************************************
 * @param[in] img          original image, level 0
 * @param[in] max_level    pyramid level, including max_level
 * @param[in] pyra_buf_ptr buffer address of pyramid buffer
 * @param[out] pyramids    pyramids
 * [NOTE] pyramids include: level 0   -> original image size
 *                          level 1   -> original image size / 2
 *                          level 2   -> original image size / 4
 *                          level 3   -> original image size / 8
 */
void build_pyramids(const cv::Mat& img, int max_level,
                    std::vector<cv::Mat>* pyramids,
                    uchar* const pyra_buf_ptr = NULL);

}  // namespace XP
#endif  // XP_INCLUDE_XP_UTIL_FEATURE_UTILS_H_
