#include "zmq_component.h"

#include <opencv2/imgcodecs.hpp>

#include "data_protocol.h"
#include "dev_socket.h"
#include "glog/logging.h"
#include "jarvis/common/time.h"
#include "opencv2/opencv.hpp"
#include "zmq.h"

namespace jarvis_pic {

namespace {

// ModLocPoseFb ToMpcData(const jarvis::TrackingData &data){return {
//     jarvis::common::ToUniversal(data.data->time),
//     data.data->pose.translation().x(), data.data->pose.translation().y(),
//     data.data->pose.translation().z(), data.data->pose.rotation().w(),
//     data.data->pose.rotation().x(), data.data->pose.rotation().y(),
//     data.data->pose.rotation().z(), 0, data.status}};

}

//
std::pair<std::string, int> host_ip{"127.0.0.1", 97555};
//

struct PoseData {
  double x;
  double y;
  double z;
  double qw;
  double qx;
  double qy;
  double qz;
  uint8_t flag;
};
// namespace

cv::Mat GenerateImageWithKeyPoint(
    const cv::Mat &l_img, const std::map<uint64_t,cv::KeyPoint> &l_key_points,
     const std::map<uint64_t,cv::KeyPoint> &predict_pts, const cv::Mat &r_img,
     const std::map<uint64_t,cv::KeyPoint> &r_key_points, const std::string &l_name,
    const std::string &r_name, std::vector<uint64_t> outlier_pointclass_id) {
  // cv::Mat l_img_feat;
  // cv::cvtColor(l_img, l_img_feat, cv::COLOR_GRAY2RGB);
  // cv::Mat r_img_feat;
  // cv::cvtColor(r_img, r_img_feat, cv::COLOR_GRAY2RGB);
  // //
  //   const int gap = 10;
  //   cv::Mat gap_image(row, gap, CV_8UC1, cv::Scalar(255, 255, 255));
  cv::Mat gray_img, loop_match_img;
  //   cv::hconcat(l_img, gap_image, gap_image);
  //   cv::hconcat(gap_image, r_img, gray_img);
  //
  // common::FixedRatioSampler sampler(0.1);
  cvtColor(l_img, loop_match_img, cv::COLOR_GRAY2RGB);
  //
  for (auto &&keypoint : (l_key_points)) {
    double len = std::min(1.0, 1.0 * keypoint.second.octave / 10);
    cv::Scalar color = cv::Scalar(255 * (1 - len), 0, 255 * len);
    // if (class_id.count(keypoint.class_id)) {
    //   cv::circle(loop_match_img, keypoint.pt, 2 ,cv::Scalar(0, 0, 255), 2);
    // } else {
    cv::circle(loop_match_img, keypoint.second.pt, 2, color, 2);
    // }
    // cv::putText(loop_match_img, std::to_string(keypoint.class_id),
    // keypoint.pt,
    //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
  }
  for (auto &&keypoint : r_key_points) {
    cv::circle(loop_match_img, keypoint.second.pt, 1, cv::Scalar(0, 255, 0), 1);
  }
  return loop_match_img;
}
std::vector<uint8_t> ToCData(const jarvis::TrackingData &data,
                             uint8_t slip_data) {
  //
  //
  const auto &tracking_data = data;
  std::vector<uint8_t> datas;
  std::vector<int> params;
  params.resize(9, 0);
  params[0] = cv::IMWRITE_JPEG_QUALITY;
  params[1] = 20;
  params[2] = cv::IMWRITE_JPEG_PROGRESSIVE;
  params[3] = 0;
  params[4] = cv::IMWRITE_JPEG_OPTIMIZE;
  params[5] = 0;
  params[6] = cv::IMWRITE_JPEG_RST_INTERVAL;
  params[7] = 0;
  cv::Mat merge_image;
  std::vector<cv::Mat> cvresult1;
  for (auto &cam_feature : tracking_data.data->features_datas) {
    auto image_result =
        GenerateImageWithKeyPoint(cam_feature.second.features.data->images[0],
                                  cam_feature.second.key_points, {}, {}, {},
                                  "pre_imag", "curr_imag", {0});
    cvresult1.emplace_back(image_result);
    // image_result.resize(640, 544);
    // cv::hconcat(image_result, image_result, merge_image);
    // CommpressedImagePub(cam_feature.first, image_result);
    // for (auto &feature : cam_feature.second.features.data->features) {
    //   map_points.push_back(cam_feature.second.map_points[feature.first]);
    // }
  }
  cv::Size resize{640,544};
  if (cvresult1.size() == 1) {
    merge_image = cvresult1[0];
  } else if (cvresult1.size() == 2) {
    cv::Mat temp ;
    cv::resize(cvresult1[1],temp,resize);
    cv::hconcat(cvresult1[0], temp, merge_image);
  } else if (cvresult1.size() == 3) {
    cv::Mat temp = cvresult1[1];
    cv::Mat temp1 = cvresult1[2];
    cv::resize(cvresult1[1],temp,resize);
    cv::resize(cvresult1[2],temp1,resize);
    cv::Mat back(640, 540,CV_8UC3,cv::Scalar::all(0));
    cv::hconcat(cvresult1[0], temp, merge_image);
    cv::Mat tmp_2_out;
    cv::hconcat(temp1, back, tmp_2_out);
    cv::vconcat(merge_image, tmp_2_out, merge_image);
  }
  // auto image_result = GenerateImageWithKeyPoint(
  //     tracking_data.data->features_datas[0].features.data->images[0],
  //     tracking_data.data->key_points, {}, cv::Mat(), {}, "pre_imag",
  //     "curr_imag", {0});

  cv::imencode(".jpeg", merge_image, datas, params);
  //

  PoseData pose{data.data->imu_state.Pose().translation().x(),
                data.data->imu_state.Pose().translation().y(),
                data.data->imu_state.Pose().translation().z(),
                data.data->imu_state.Pose().rotation().w(),
                data.data->imu_state.Pose().rotation().x(),
                data.data->imu_state.Pose().rotation().y(),
                data.data->imu_state.Pose().rotation().z(),
                slip_data};
  int lenth = datas.size();
  datas.resize(datas.size() + sizeof(PoseData));
  memcpy((void *)(datas.data() + lenth), (void *)&pose, sizeof(PoseData));
  std::vector<uint8_t> result;
  result.push_back(0xaa);
  result.push_back(0x55);
  result.push_back(0xaa);
  result.push_back(0x5a);
  //
  uint32_t data_lenth = datas.size();
  char c_lenth[4];
  memcpy((void *)(c_lenth), (void *)&data_lenth, 4);
  for (int i = 0; i < 4; i++) {
    result.push_back(c_lenth[i]);
  }
  result.insert(result.end(), datas.begin(), datas.end());
  return result;
}
//
ZmqComponent::ZmqComponent() {
  try {
    device_.emplace_back(
        new internal::DevSocket(host_ip, [](std::vector<uint8_t> &&d) {}));
    thread_ = std::thread([this]() {
      while (!kill_thread_) {
        Run();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        // usleep(1000);
      }
    });
  } catch (const std::string s) {
    LOG(INFO) << "Devive creat err" << s;
  }
};
//
//

void ZmqComponent::PubLocalData(const jarvis::TrackingData &data,
                                uint8_t slip_data) {
  //
  std::lock_guard<std::mutex> lock(mutex_);
  tasks_.push([=]() {
    for (auto &dev : device_) {
      if (dev->HasConnect()) {
        dev->tx(ToCData(data, slip_data));
      }
    }
  });
};
void ZmqComponent::Run() {
  size_t task_size = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    task_size = tasks_.size();
  }
  std::function<void(void)> f;
  while (task_size != 0) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      while (!tasks_.empty()) {
        f = std::move(tasks_.front());
        tasks_.pop();
        task_size = tasks_.size();
      }
    }
    LOG_EVERY_N(INFO,100) << "send task size " << task_size;
    f();
  }
};

ZmqComponent::~ZmqComponent() {

}
//
MpcComponent::MpcComponent() : shm_mod_(new ShmMod()) {}
//
void MpcComponent::Write(const jarvis::transform::Rigid3d &pose,
                         const jarvis::TrackingData &data,
                         const uint64_t &imu_base, bool slip) {
  // /
  ModLocPoseFb mpc_data{
      static_cast<uint64_t>(jarvis::common::ToUniversal(data.data->time) * 1e2),
      static_cast<uint64_t>(imu_base * 1e3),
      pose.translation().x(),
      pose.translation().y(),
      pose.translation().z(),
      pose.rotation().x(),
      pose.rotation().y(),
      pose.rotation().z(),
      pose.rotation().w(),
      slip?uint8_t(9):uint8_t(0),
      static_cast<uint8_t>(data.status)};
  auto start = std::chrono::high_resolution_clock::now();
  shm_mod_->SetModByID(vio_id_, reinterpret_cast<void *>(&mpc_data));
  int titic = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::high_resolution_clock::now() - start)
                  .count();
  if(titic >500){
    LOG(ERROR)<<"SetModByID cost: "<<titic<<" "<<mpc_data.timestamp;
  }
  // CHECK(titic<3000)<titic;
  // LOG(INFO) << "SetModByID cost: "
  //           << std::chrono::duration_cast<std::chrono::microseconds>(
  //                  std::chrono::high_resolution_clock::now() - start)
  //                  .count();

  //
  // // //
  // memset(reinterpret_cast<void *>(&mpc_data), 0, sizeof(ModLocPoseFb));
  // shm_mod_->GetModByID(vio_id_, reinterpret_cast<void *>(&mpc_data));

  // jarvis::transform::Rigid3d read_pose(
  //     Eigen::Vector3d{mpc_data.x, mpc_data.y, mpc_data.z},
  //     Eigen::Quaterniond(mpc_data.qw, mpc_data.qx, mpc_data.qy, mpc_data.qz));
  // LOG_EVERY_N(INFO, 1) << "Read pose: " << mpc_data.timestamp << " "
  //                        << read_pose << " "
  //                        << "state " << int(mpc_data.state);
}
}  // namespace jarvis_pic