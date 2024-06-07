#include "jarvis/estimator/featureTracker/pyramid_image.h"
#include "jarvis/estimator/featureTracker/feature_utils.h"
namespace jarvis {
namespace estimator {
//

namespace {
void build_pyramids(const cv::Mat& img, int max_level,
                    std::vector<cv::Mat>* _pyramids,
                    uchar* const pyra_buf_ptr) {
  // std::vector<cv::Mat> pyra;
  // LOG(INFO)<<img.channels();
  // // _pyramids->resize(pyra.size());
  // buildOpticalFlowPyramid(img, *_pyramids, cv::Size(21,21),max_level, true);
  // // (*_pyramids)[0] =  img;
  // // for(int i  =0;i<pyra.size();i++){
  // //   (*_pyramids)[i] =  cv::cvtColor(pyra[i],cv::) ;
  // // }
  // LOG(INFO) << (*_pyramids)[0].channels();
  // return ;
  CHECK_NOTNULL(_pyramids);
  std::vector<cv::Mat>& pyramids = *_pyramids;
  pyramids.resize(max_level + 1);
  const int size = img.rows * img.cols;
  // no fixed buffer specified, will allocate memory dynamically
  if (pyra_buf_ptr == nullptr) {
    pyramids[0] = img.clone();
    for (int i = 1; i <= max_level; ++i) {
      pyramids[i] = XP::fast_pyra_down(pyramids[i - 1]);
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
            XP::fast_pyra_down(pyramids[lvl - 1], pyra_buf_ptr + offset);
      } else {
        cv::Mat tmp(img.rows, img.cols, img.type(), pyra_buf_ptr + offset);
        img.copyTo(tmp);
        pyramids[lvl] = tmp;
      }
    }
  }
}

}  // namespace

PyramidImage::PyramidImage(const PyramidImageOption& option)
    : option_(option),
      prev_pyramids_buffer_(
          new uchar[option.image_size.x() * option.image_size.y() * 2]),
      curr_pyramids_buffer_(
          new uchar[option.image_size.x() * option.image_size.y() * 2]) {}
//

void PyramidImage::Build(const cv::Mat& image) {
  //
  //
  if (!prev_img_pyramids_.empty()) {
    curr_img_pyramids_.swap(prev_img_pyramids_);
    curr_pyramids_buffer_.swap(prev_pyramids_buffer_);

  } else {
    build_pyramids(image, option_.layer, &prev_img_pyramids_,
                   nullptr);
  }
  //
  LOG(INFO)<<"1";
  build_pyramids(image, option_.layer, &curr_img_pyramids_,
                 nullptr);

  LOG(INFO)<<"1";
          
}

//
}  // namespace estimator
}  // namespace jarvis