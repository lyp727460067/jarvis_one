#include <thread>
#include <vector>
#include "feature_extract.h"
#include "glog/logging.h"
namespace jarvis {
namespace estimator {
void GoodFeaturesToTrack_neon(const cv::Mat& image0,
                                     std::vector<cv::KeyPoint>& corners,  // NOLINT
                                     int maxCorners,
                                     double qualityLevel,
                                     double minDistance) {
  auto start_time_goodFeaturesToTrack = std::chrono::high_resolution_clock::now();
#ifdef __ARM_NEON__
  // STEP 1: calculate eig
  cv::Mat image = image0.clone();
  cv::Mat eig;
  eig.create(image.size(), CV_32F);
  if (!image.isContinuous() || !eig.isContinuous() || image.type() != CV_8UC1) {
    LOG(FATAL) << "FAILED at the very beginning..";
  }
  int w = image.cols;
  int h = image.rows;
  float *eigdata = reinterpret_cast<float*>(eig.data);
  float *cov = NULL;
  if (posix_memalign(reinterpret_cast<void**>(&cov), 16, w*h*3*sizeof(float))) {
    LOG(FATAL) << "posix_memalign FAILED..";
  }
  memset(eigdata, 0, w * sizeof(float));
  memset(eigdata + (w * (h - 1)), 0, w * sizeof(float));
  memset(cov, 0, w * 3 * sizeof(float));
  memset(cov + (w * 3 * (h - 1)), 0, w * 3 * sizeof(float));

  float* dest = cov + w * 3;
  const unsigned char* const srcmax = reinterpret_cast<unsigned char*>(image.data) + w * (h - 1);
  /*
   __m128 dxdx_prev = _mm_setzero_ps();
   __m128 dxdy_prev = _mm_setzero_ps();
   __m128 dydy_prev = _mm_setzero_ps();
   __m128 dxdx_sum_prev = _mm_setzero_ps();
   __m128 dxdy_sum_prev = _mm_setzero_ps();
   __m128 dydy_sum_prev = _mm_setzero_ps();
   */
  float32x4_t dxdx_prev = vdupq_n_f32(0.0f);
  float32x4_t dxdy_prev = vdupq_n_f32(0.0f);
  float32x4_t dydy_prev = vdupq_n_f32(0.0f);
  float32x4_t dxdx_sum_prev = vdupq_n_f32(0.0f);
  float32x4_t dxdy_sum_prev = vdupq_n_f32(0.0f);
  float32x4_t dydy_sum_prev = vdupq_n_f32(0.0f);
  float harris_k = 0.04;

  for (const unsigned char *p = reinterpret_cast<unsigned char*>(image.data) + w;
       p < srcmax; p += 16) {
    /*
     __m128i in_u = _mm_load_si128(reinterpret_cast<const __m128i*>(p - w));
     __m128i in_d = _mm_load_si128(reinterpret_cast<const __m128i*>(p + w));
     __m128i in_l = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p - 1));
     __m128i in_r = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p + 1));
     */
    // NOTE: www.itlab.unn.ru/file.php?id=731 "ARM NEON SIMD" lecture pdf
    // Page 37: "NEON - no way to specify alignment for intrinsics"
    // The (const int32_t*) cast seems working fine and gets recommended (Page 43).
    int32x4_t in_l = vld1q_s32((const int32_t*)(p - 1));
    int32x4_t in_r = vld1q_s32((const int32_t*)(p + 1));
    int32x4_t in_u = vld1q_s32((const int32_t*)(p - w));
    int32x4_t in_d = vld1q_s32((const int32_t*)(p + w));

    /*
     __m128i dx16[2] = {
     _mm_sub_epi16(_mm_cvtepu8_epi16(in_r), _mm_cvtepu8_epi16(in_l)),
     _mm_sub_epi16(_mm_cvtepu8_epi16(_mm_srli_si128(in_r, 8)),
     _mm_cvtepu8_epi16(_mm_srli_si128(in_l, 8)))
     };
     __m128i dy16[2] = {
     _mm_sub_epi16(_mm_cvtepu8_epi16(in_d), _mm_cvtepu8_epi16(in_u)),
     _mm_sub_epi16(_mm_cvtepu8_epi16(_mm_srli_si128(in_d, 8)),
     _mm_cvtepu8_epi16(_mm_srli_si128(in_u, 8)))
     };
     */
    // NOTE: _mm_cvtepu8_epi16 is "zero-extended" and treated as unsigned
    // A great reference check for NEON vs SSE: https://software.intel.com/sites/default/files/managed/cf/f6/NEONvsSSE.h
    // int32x4_t -> int32x2_t -> uint8x8_t -> uint16x8_t
    uint16x8_t in_r_low = vmovl_u8(vreinterpret_u8_s32(vget_low_s32(in_r)));
    uint16x8_t in_l_low = vmovl_u8(vreinterpret_u8_s32(vget_low_s32(in_l)));
    uint16x8_t in_r_high = vmovl_u8(vreinterpret_u8_s32(vget_high_s32(in_r)));
    uint16x8_t in_l_high = vmovl_u8(vreinterpret_u8_s32(vget_high_s32(in_l)));
    uint16x8_t in_d_low = vmovl_u8(vreinterpret_u8_s32(vget_low_s32(in_d)));
    uint16x8_t in_u_low = vmovl_u8(vreinterpret_u8_s32(vget_low_s32(in_u)));
    uint16x8_t in_d_high = vmovl_u8(vreinterpret_u8_s32(vget_high_s32(in_d)));
    uint16x8_t in_u_high = vmovl_u8(vreinterpret_u8_s32(vget_high_s32(in_u)));
    // todo: confirm that re-interpret uint16x8_t -> int16x8_t before subtraction is good practice
    int16x8_t dx16[2] = {
      vsubq_s16(vreinterpretq_s16_u16(in_r_low), vreinterpretq_s16_u16(in_l_low)),
      vsubq_s16(vreinterpretq_s16_u16(in_r_high), vreinterpretq_s16_u16(in_l_high))};
    int16x8_t dy16[2] = {
      vsubq_s16(vreinterpretq_s16_u16(in_d_low), vreinterpretq_s16_u16(in_u_low)),
      vsubq_s16(vreinterpretq_s16_u16(in_d_high), vreinterpretq_s16_u16(in_u_high))};

    /*
     __m128 dx[4] = {
     _mm_cvtepi32_ps(_mm_cvtepi16_epi32(dx16[0])),
     _mm_cvtepi32_ps(_mm_cvtepi16_epi32(_mm_srli_si128(dx16[0], 8))),
     _mm_cvtepi32_ps(_mm_cvtepi16_epi32(dx16[1])),
     _mm_cvtepi32_ps(_mm_cvtepi16_epi32(_mm_srli_si128(dx16[1], 8)))
     };
     __m128 dy[4] = {
     _mm_cvtepi32_ps(_mm_cvtepi16_epi32(dy16[0])),
     _mm_cvtepi32_ps(_mm_cvtepi16_epi32(_mm_srli_si128(dy16[0], 8))),
     _mm_cvtepi32_ps(_mm_cvtepi16_epi32(dy16[1])),
     _mm_cvtepi32_ps(_mm_cvtepi16_epi32(_mm_srli_si128(dy16[1], 8)))
     };
     http://stackoverflow.com/questions/30528352/how-to-convert-unsigned-char-to-signed-integer-by-neon
     "The NEON vector types are not guaranteed to be convertible by casts,
     so for most portability you should write vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(v)))."
     _mm_cvtepi16_epi32 is sign-extended, and vmovl_s16 is sign-extended.
     NOTE: _mm_cvtepi32_ps shoud be: float32x4_t vcvtq_f32_s32(int32x4_t a);
     NOTE: dx16/dy16: int16x8_t -> int16x4_t -> int32x4_t -> float32x4_t
     */
    float32x4_t dx[4] = {
      vcvtq_f32_s32(vmovl_s16(vget_low_s16(dx16[0]))),
      vcvtq_f32_s32(vmovl_s16(vget_high_s16(dx16[0]))),
      vcvtq_f32_s32(vmovl_s16(vget_low_s16(dx16[1]))),
      vcvtq_f32_s32(vmovl_s16(vget_high_s16(dx16[1])))
    };
    float32x4_t dy[4] = {
      vcvtq_f32_s32(vmovl_s16(vget_low_s16(dy16[0]))),
      vcvtq_f32_s32(vmovl_s16(vget_high_s16(dy16[0]))),
      vcvtq_f32_s32(vmovl_s16(vget_low_s16(dy16[1]))),
      vcvtq_f32_s32(vmovl_s16(vget_high_s16(dy16[1])))
    };


    // NOTE(): bracket No.1
    {
    // We process in one iteration:
    // |p1 p2 p3 p4|a b c d|e f g h|i j k l|m n o p|

    /***** Previous Cell starts here *****/
    // |a b c d|e f g h|
    /*
     __m128 product[2] = {
     _mm_mul_ps(dx[0], dx[0]), _mm_mul_ps(dx[1], dx[1])
     };
     */
    float32x4_t product[2] = {vmulq_f32(dx[0], dx[0]), vmulq_f32(dx[1], dx[1])};

    // |p2 p3 p4 0| AND |0 0 0 a| => |p2 p3 p4 a|
    /*
     __m128 shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(dxdx_prev), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 12)));
     NOTE: http://stackoverflow.com/questions/11259596/arm-neon-intrinsics-rotation
     */
    float32x4_t shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(dxdx_prev), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[0]), 1)));

    // Store |p2 p3 p4 a| + prev_sum
    /*
     _mm_store_ps(dest-36, _mm_add_ps(shiftfrom_r, dxdx_sum_prev));
     */
    vst1q_f32(dest - 36, vaddq_f32(shiftfrom_r, dxdx_sum_prev));
    /***** Previous Cell ends here *****/


    /***** First Cell starts here *****/
    /*
     // |0 a b c| AND |p4 0 0 0| => |p4 a b c|
     __m128 shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(dxdx_prev), 12)));
     */
    float32x4_t shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[0]), 3),
        vextq_s32(vreinterpretq_s32_f32(dxdx_prev), vdupq_n_s32(0), 3)));
    /*
     // |b c d 0| AND |0 0 0 e| => |b c d e|
     shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 12)));
     */
    shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 1)));

    /*
     // Store |p4 a b c| +  |a b c d| + |b c d e|
     _mm_store_ps(dest,
     _mm_add_ps(_mm_add_ps(product[0], shiftfrom_l), shiftfrom_r));
     */
    vst1q_f32(dest, vaddq_f32(vaddq_f32(product[0], shiftfrom_l), shiftfrom_r));
    /***** First Cell starts here *****/


    /***** Second Cell starts here *****/
    /*
     // |0 e f g| AND |d 0 0 0| => |d e f g|
     shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 12)));
     */
    shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 3),
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 3)));

    /*
     // |i j k l|
     product[0] = _mm_mul_ps(dx[2], dx[2]);
     */
    product[0] = vmulq_f32(dx[2], dx[2]);

    /*
     // |f g h 0| AND |0 0 0 i| => |f g h i|
     shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[1]), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 12)));

     // Store |d e f g| +  |e f g h| + |f g h i|
     _mm_store_ps(dest+4,
     _mm_add_ps(_mm_add_ps(product[1], shiftfrom_l), shiftfrom_r));
     */
    shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 1),
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 1)));
    vst1q_f32(dest + 4, vaddq_f32(vaddq_f32(product[1], shiftfrom_l), shiftfrom_r));
    /***** Second Cell starts here *****/


    /***** Third Cell starts here *****/
    /*
     // |0 i j k| AND |h 0 0 0| => |h i j k|
     shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[1]), 12)));
     */
    shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 3),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 3)));

    /*
     // |m n o p|
     product[1] = _mm_mul_ps(dx[3], dx[3]);
     */
    product[1] = vmulq_f32(dx[3], dx[3]);

    /*
     // |j k l 0| AND |0 0 0 m| => |j k l m|
     shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 12)));

     // Store |h i j k| +  |i j k l| + |j k l m|
     _mm_store_ps(dest+8,
     _mm_add_ps(_mm_add_ps(product[0], shiftfrom_l), shiftfrom_r));
     */
    shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 1)));
    vst1q_f32(dest + 8, vaddq_f32(vaddq_f32(product[0], shiftfrom_l), shiftfrom_r));
    /***** Third Cell starts here *****/


    /***** Fourth Cell starts here *****/
    /*
     // |0 m n o| AND |l 0 0 0| => |l m n o|
     shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 12)));
     
     // Store |l m n o| +  |m n o p|
     dxdx_sum_prev = _mm_add_ps(product[1], shiftfrom_l);
     dxdx_prev = product[1];
     */
    shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 3),
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 3)));
    dxdx_sum_prev = vaddq_f32(product[1], shiftfrom_l);
    dxdx_prev = product[1];
    /***** Fourth Cell starts here *****/

    dest += 16;
    }


    // NOTE(): bracket No.2
    {
    /*
     // ######## Previous Cell
     __m128 product[2] = {
     _mm_mul_ps(dx[0], dy[0]), _mm_mul_ps(dx[1], dy[1])
     };

     __m128 shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(dxdy_prev), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 12)));

     _mm_store_ps(dest-36, _mm_add_ps(shiftfrom_r, dxdy_sum_prev));


     // ######## First Cell
     __m128 shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(dxdy_prev), 12)));

     shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 12)));

     _mm_store_ps(dest,
     _mm_add_ps(_mm_add_ps(product[0], shiftfrom_l), shiftfrom_r));


     // ######## Second Cell
     shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 12)));

     product[0] = _mm_mul_ps(dx[2], dy[2]);

     shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[1]), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 12)));

     _mm_store_ps(dest+4,
     _mm_add_ps(_mm_add_ps(product[1], shiftfrom_l), shiftfrom_r));


     // ######## Third Cell
     shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[1]), 12)));

     product[1] = _mm_mul_ps(dx[3], dy[3]);

     shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 12)));

     _mm_store_ps(dest+8,
     _mm_add_ps(_mm_add_ps(product[0], shiftfrom_l), shiftfrom_r));


     // ######## Fourth Cell
     shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 12)));

     dxdy_sum_prev = _mm_add_ps(product[1], shiftfrom_l);
     dxdy_prev = product[1];

     dest += 16;
     */

    // ######## Previous Cell
    float32x4_t product[2] = {vmulq_f32(dx[0], dy[0]), vmulq_f32(dx[1], dy[1])};

    float32x4_t shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(dxdy_prev), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[0]), 1)));

    vst1q_f32(dest - 36, vaddq_f32(shiftfrom_r, dxdy_sum_prev));


    // ######## First Cell
    float32x4_t shiftfrom_l =
    vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[0]), 3),
        vextq_s32(vreinterpretq_s32_f32(dxdy_prev), vdupq_n_s32(0), 3)));

    shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 1)));

    vst1q_f32(dest, vaddq_f32(vaddq_f32(product[0], shiftfrom_l), shiftfrom_r));


    // ######## Second Cell
    shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 3),
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 3)));

    product[0] = vmulq_f32(dx[2], dy[2]);

    shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(product[1]), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[0]), 1)));

    vst1q_f32(dest + 4, vaddq_f32(vaddq_f32(product[1], shiftfrom_l), shiftfrom_r));


    // ######## Third Cell
    shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[0]), 3),
        vextq_s32(vreinterpretq_s32_f32(product[1]), vdupq_n_s32(0), 3)));

    product[1] = vmulq_f32(dx[3], dy[3]);

    shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 1)));

    vst1q_f32(dest + 8, vaddq_f32(vaddq_f32(product[0], shiftfrom_l), shiftfrom_r));


    // ######## Fourth Cell
    shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 3),
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 3)));

    dxdy_sum_prev = vaddq_f32(product[1], shiftfrom_l);
    dxdy_prev = product[1];

    dest += 16;
    }


    // NOTE(): bracket No.3
    {
    /*
     // ######## Previous Cell
     __m128 product[2] = {
     _mm_mul_ps(dy[0], dy[0]), _mm_mul_ps(dy[1], dy[1])
     };

     __m128 shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(dydy_prev), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 12)));

     _mm_store_ps(dest-36, _mm_add_ps(shiftfrom_r, dydy_sum_prev));


     // ######## First Cell
     __m128 shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(dydy_prev), 12)));

     shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 12)));

     _mm_store_ps(dest,
     _mm_add_ps(_mm_add_ps(product[0], shiftfrom_l), shiftfrom_r));


     // ######## Second Cell
     shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 12)));

     product[0] = _mm_mul_ps(dy[2], dy[2]);

     shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[1]), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 12)));

     _mm_store_ps(dest+4,
     _mm_add_ps(_mm_add_ps(product[1], shiftfrom_l), shiftfrom_r));


     // ######## Third Cell
     shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[1]), 12)));

     product[1] = _mm_mul_ps(dy[3], dy[3]);

     shiftfrom_r = _mm_and_ps(
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 4)),
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 12)));

     _mm_store_ps(dest+8,
     _mm_add_ps(_mm_add_ps(product[0], shiftfrom_l), shiftfrom_r));


     // ######## Fourth Cell
     shiftfrom_l = _mm_and_ps(
     _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(product[1]), 4)),
     _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(product[0]), 12)));

     dydy_sum_prev = _mm_add_ps(product[1], shiftfrom_l);
     dydy_prev = product[1];

     dest += 16;
     */

    // ######## Previous Cell
    float32x4_t product[2] = {vmulq_f32(dy[0], dy[0]), vmulq_f32(dy[1], dy[1])};

    float32x4_t shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(dydy_prev), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[0]), 1)));

    vst1q_f32(dest - 36, vaddq_f32(shiftfrom_r, dydy_sum_prev));


    // ######## First Cell
    float32x4_t shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[0]), 3),
        vextq_s32(vreinterpretq_s32_f32(dydy_prev), vdupq_n_s32(0), 3)));

    shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 1)));

    vst1q_f32(dest, vaddq_f32(vaddq_f32(product[0], shiftfrom_l), shiftfrom_r));


    // ######## Second Cell
    shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 3),
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 3)));

    product[0] = vmulq_f32(dy[2], dy[2]);

    shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(product[1]), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[0]), 1)));

    vst1q_f32(dest + 4, vaddq_f32(vaddq_f32(product[1], shiftfrom_l), shiftfrom_r));


    // ######## Third Cell
    shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[0]), 3),
        vextq_s32(vreinterpretq_s32_f32(product[1]), vdupq_n_s32(0), 3)));

    product[1] = vmulq_f32(dy[3], dy[3]);

    shiftfrom_r = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 1),
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 1)));

    vst1q_f32(dest + 8, vaddq_f32(vaddq_f32(product[0], shiftfrom_l), shiftfrom_r));


    // ######## Fourth Cell
    shiftfrom_l = vreinterpretq_f32_s32(
      vandq_s32(
        vextq_s32(vdupq_n_s32(0), vreinterpretq_s32_f32(product[1]), 3),
        vextq_s32(vreinterpretq_s32_f32(product[0]), vdupq_n_s32(0), 3)));

    dydy_sum_prev = vaddq_f32(product[1], shiftfrom_l);
    dydy_prev = product[1];

    dest += 16;
    }
  }
  const int t1 =
  std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::high_resolution_clock::now() - start_time_goodFeaturesToTrack).count() / 1000;

  /*
   // Smooth the cov in y with a 3-neighborhood,
   // compute the eigenvalues, and store the lower one to eig
   const __m128 half = _mm_set1_ps(8.0);
   const __m128 full = _mm_set1_ps(16.0);
   const int w3 = w*3;

   __m128 maximum = _mm_set1_ps(-1.0f);
   dest = eigdata+w;
   const float * const covmax = cov + w*(h-1)*3;
   
   for (const float *p = cov + w3; p < covmax; p+=32) {
   for (int i = 0; i < 4; i++, p+=4, dest+=4) {
   const __m128 dxdx = _mm_mul_ps(half, _mm_add_ps(_mm_add_ps(
   _mm_load_ps(p), _mm_load_ps(p-w3)),
   _mm_load_ps(p+w3)));
   const __m128 dxdy = _mm_mul_ps(full, _mm_add_ps(_mm_add_ps(
   _mm_load_ps(p+16), _mm_load_ps(p-w3+16)),
   _mm_load_ps(p+w3+16)));
   const __m128 dydy = _mm_mul_ps(half, _mm_add_ps(_mm_add_ps(
   _mm_load_ps(p+32), _mm_load_ps(p-w3+32)),
   _mm_load_ps(p+w3+32)));
   __m128 t = _mm_sub_ps(dxdx, dydy);
   t = _mm_add_ps(_mm_mul_ps(t, t), _mm_mul_ps(dxdy, dxdy));
   const __m128 eig2 = _mm_sub_ps(_mm_add_ps(dxdx, dydy), _mm_sqrt_ps(t));
   _mm_store_ps(dest, eig2);
   // Update the maximum
   maximum = _mm_max_ps(maximum, eig2);
   }
   }
   */

  float32x4_t half = vdupq_n_f32(8.0f);
  float32x4_t full = vdupq_n_f32(16.0f);
  const int w3 = w * 3;
  float32x4_t maximum = vdupq_n_f32(-1.0f);
  /*
   * NOTE(): float *eigdata = reinterpret_cast<float*>(eig.data);
   * below is to compute the smaller eigen value and store to dest/eigdata/eig
   * */
  dest = eigdata + w;
  const float* const covmax = cov + w * (h - 1) * 3;
  float32x4_t min_dxdx = {900, 900, 900, 900};
  int skip_count = 0;
  for (const float *p = cov + w3; p < covmax; p += 32) {
    for (int i = 0; i < 4; i++, p += 4, dest += 4) {
      float32x4_t dxdx = vmulq_f32(half,
        vaddq_f32(vaddq_f32(vld1q_f32(p), vld1q_f32(p - w3)), vld1q_f32(p + w3)));
      float32x4_t dydy = vmulq_f32(half,
        vaddq_f32(vaddq_f32(vld1q_f32(p + 32), vld1q_f32(p - w3 + 32)), vld1q_f32(p + w3 + 32)));
      // skip if both dxdx dydy are small.
      uint32x4_t small_dydx_flags = vandq_u32(vcltq_f32(dxdx, min_dxdx), vcltq_f32(dydy, min_dxdx));
      bool skip_flag = true;
      /*
       if (p == cov + w3)
       LOG(ERROR)
       << "dxdx[0]" << vgetq_lane_f32(dxdx, 0)
       << " dydy[0] " << vgetq_lane_f32(dydy, 0)
       << " dxdx[1]" << vgetq_lane_f32(dxdx, 1)
       << " dydy[1] " << vgetq_lane_f32(dydy, 1)
       << " dxdx[2]" << vgetq_lane_f32(dxdx, 2)
       << " dydy[2] " << vgetq_lane_f32(dydy, 2)
       << " dxdx[3]" << vgetq_lane_f32(dxdx, 3)
       << " dydy[3] " << vgetq_lane_f32(dydy, 3)
       << " vcltq_f32(dxdx, min_dxdx)[0] " << vgetq_lane_u32(vcltq_f32(dxdx, min_dxdx), 0)
       << " vcltq_f32(dydy, min_dxdx)[0] " << vgetq_lane_u32(vcltq_f32(dydy, min_dxdx), 0)
       << " small_dydx_flags[0] " << vgetq_lane_u32(small_dydx_flags, 0);*/
      // vgetq_lane_u32 returns 0 or 4294967295 (0xffff)
      // https://community.arm.com/thread/9285
      if (vgetq_lane_u32(small_dydx_flags, 0) == static_cast<uint32_t>(0) ||
          vgetq_lane_u32(small_dydx_flags, 1) == static_cast<uint32_t>(0) ||
          vgetq_lane_u32(small_dydx_flags, 2) == static_cast<uint32_t>(0) ||
          vgetq_lane_u32(small_dydx_flags, 3) == static_cast<uint32_t>(0)) {
        skip_flag = false;
      }
      if (skip_flag) {
        skip_count++;
        dest[0] = 0;
        dest[1] = 0;
        dest[2] = 0;
        dest[3] = 0;
        continue;
      }
      float32x4_t dxdy = vmulq_f32(full,
        vaddq_f32(vaddq_f32(vld1q_f32(p + 16), vld1q_f32(p - w3 + 16)), vld1q_f32(p + w3 + 16)));

      /* SHI-TOMASI */
      float32x4_t t = vsubq_f32(dxdx, dydy);
      t = vaddq_f32(vmulq_f32(t, t), vmulq_f32(dxdy, dxdy));
      /*
       NEON implementation of _mm_sqrt_ps:
       - the first try: https://github.com/jratcliff63367/sse2neon/blob/master/SSE2NEON.h#L653
       - the current approximative quadword float inverse square root:
       - https://pmeerw.net/blog/programming/neon1.html
       */
      float32x4_t sqrt_reciprocal = vrsqrteq_f32(t);
      float32x4_t eig2 = vsubq_f32(vaddq_f32(dxdx, dydy),
        t * vrsqrtsq_f32(t * sqrt_reciprocal, sqrt_reciprocal) * sqrt_reciprocal);
      vst1q_f32(dest, eig2);
      // Update the maximum
      maximum = vmaxq_f32(maximum, eig2);
      /* SHI-TOMASI */

      //      /* HARRIS */
      //      float32x4_t det =
      //          vsubq_f32(vmulq_f32(dxdx, dydy), vmulq_f32(dxdy, dxdy));
      //      float32x4_t trace = vaddq_f32(dxdx, dydy);
      //      float32x4_t k_trace_sqr =
      //          vmulq_f32(vdupq_n_f32(harris_k), vmulq_f32(trace, trace));
      //      float32x4_t harris = vsubq_f32(det, k_trace_sqr);
      //      vst1q_f32(dest, harris);
      //      // Update the maximum
      //      maximum = vmaxq_f32(maximum, harris);
      //      /* HARRIS */
    }
  }

  free(cov);
  const int t2 = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::high_resolution_clock::now() - start_time_goodFeaturesToTrack).count() / 1000;

  // STEP 2: NMS + spatial binning to construct std::vector<cv::Point2f>& points
  cv::Mat tmp;
  const cv::Size imgsize = image.size();
  std::vector<const float*> temp_corners;
  // TODO(xp): verify if border_size is OK with 1
  // TODO(xp): move parameters to config
  const int border_size = 1;
  const float threshold = 1000;
  const float robust_ratio = 0.9;

  cv::dilate(eig, tmp, cv::Mat());   //, cv::Point(-1,-1), 3);
  const int t3 = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::high_resolution_clock::now() - start_time_goodFeaturesToTrack).count() / 1000;

  //  std::vector<int> eig_ct(5, 0);
  //  std::vector<int> eig_thres(5, 0);
  //  eig_thres[0] = 0;
  //  eig_thres[1] = threshold;
  //  eig_thres[2] = threshold * 2;
  //  eig_thres[3] = threshold * 3;
  //  eig_thres[4] = threshold * 4;
  //  int eig_thres_minus = 0;
  //  collect list of pointers to features - put them into temporary image
  temp_corners.reserve(1000);

  for (int y = border_size; y < imgsize.height - border_size; y++) {
    float * eig_data = reinterpret_cast<float *>(eig.ptr(y));
    float * tmp_data = reinterpret_cast<float *>(tmp.ptr(y));

    for (int x = border_size; x < imgsize.width - border_size; x++) {
      float val = eig_data[x];
      if (val > threshold && val == tmp_data[x]) {
            int y = static_cast<int>(x / eig.step);
            int xx = static_cast<int>((x - y*eig.step)/sizeof(float));
            corners.emplace_back(xx,y,2);
        // eig_data[x] = val;
        // temp_corners.push_back(eig_data + x);
      }

      //      int tmp = val / threshold;
      //      if (tmp < 0) {
      //        eig_thres_minus++;
      //      } else if (tmp >= 4) {
      //        eig_thres[4]++;
      //      } else {
      //        eig_thres[tmp]++;
      //      }
    }
  }
  
//   const int t4 = std::chrono::duration_cast<std::chrono::microseconds>(
//     std::chrono::high_resolution_clock::now() - start_time_goodFeaturesToTrack).count() / 1000;

//   //  LOG(ERROR) << "temp_corner.size(): " << temp_corners.size();
//   //  for (int i = 0; i < 5; i++) {
//   //    LOG(ERROR) << "eig_thres[" << i << "] - " << eig_thres[i];
//   //  }
//   //  LOG(ERROR) << "eig_thres_minus: " << eig_thres_minus;
//   std::sort(temp_corners.begin(), temp_corners.end(), greaterThanPtr);
//   int i, j, total = temp_corners.size(), ncorners = 0;

//   // Partition the image into larger grids
//   const int cell_size = cvRound(minDistance);
//   const int grid_width = (w + cell_size - 1) / cell_size;
//   const int grid_height = (h + cell_size - 1) / cell_size;
//   std::vector<std::vector<cv::Point2f> > grid(grid_width*grid_height);
//   std::vector<std::vector<float> > response_grid(grid_width*grid_height);
//   minDistance *= minDistance;

//   const int bin_cell_size = 80;
//   const int bin_grid_width = (w + bin_cell_size - 1) / bin_cell_size;  // 640 / 80 = 8
//   const int bin_grid_height = (h + bin_cell_size - 1) / bin_cell_size;  // 480 / 80 = 6
//   // #of features / (8 * 6)
//   const int bin_threshold = maxCorners / (bin_grid_width * bin_grid_height);
//   std::vector<int> bin_counter((w / bin_cell_size) * (h / bin_cell_size), 0);

//   corners.reserve(total);
//   for (i = 0; i < total; i++) {
//     int ofs = static_cast<int>(
//       reinterpret_cast<const unsigned char *>(temp_corners[i]) - eig.data);
//     //    LOG(ERROR) << "temp_corners[" << i << "] - " << *temp_corners[i];
//     int y = static_cast<int>(ofs / eig.step);
//     int x = static_cast<int>((ofs - y*eig.step)/sizeof(float));

//     float response = *(temp_corners[i]);

//     bool good = true;

//     int x_cell = x / cell_size;
//     int y_cell = y / cell_size;
//     int x_bin_cell = x / bin_cell_size;
//     int y_bin_cell = y / bin_cell_size;

//     int x1 = x_cell - 1;
//     int y1 = y_cell - 1;
//     int x2 = x_cell + 1;
//     int y2 = y_cell + 1;

//     // boundary check
//     x1 = std::max(0, x1);
//     y1 = std::max(0, y1);
//     x2 = std::min(grid_width - 1, x2);
//     y2 = std::min(grid_height - 1, y2);

//     for (int yy = y1; yy <= y2; yy++) {
//       for (int xx = x1; xx <= x2; xx++) {
//         std::vector<cv::Point2f> &m = grid[yy*grid_width + xx];
//         std::vector<float> &r = response_grid[yy*grid_width + xx];

//         if (m.size()) {
//           for (j = 0; j < m.size(); j++) {
//             float dx = x - m[j].x;
//             float dy = y - m[j].y;

//             // Compare the ratio of responses, so features are only
//             // suppressed if they are significantly weaker
//             if (dx*dx + dy*dy < minDistance &&
//                 response <= robust_ratio * r[j]) {
//               good = false;
//               goto break_out;
//             }
//           }
//         }
//       }
//     }

//     break_out:

//     if (good) {
//       if (bin_counter[y_bin_cell * bin_grid_width + x_bin_cell] < bin_threshold) {
//         bin_counter[y_bin_cell * bin_grid_width + x_bin_cell]++;
//         grid[y_cell*grid_width + x_cell].push_back(
//           cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
//         response_grid[y_cell*grid_width + x_cell].push_back(response);

//         corners.push_back(
//                           cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
//         ++ncorners;

//         if (maxCorners > 0 && static_cast<int>(ncorners) == maxCorners) {
//           break;
//         }
//       }
//     }
//   }
//   const int t5 = std::chrono::duration_cast<std::chrono::microseconds>(
//     std::chrono::high_resolution_clock::now() - start_time_goodFeaturesToTrack).count() / 1000;
//   LOG(ERROR) << "goodFeaturesToTrack neon t1-t5: " << t1
//   << " " << t2 << " " << t3 << " " << t4 << " " << t5 << " ms";

#else
  LOG(FATAL) << "goodFeaturesToTrack_neon is called without neon support";
#endif
}
}}
