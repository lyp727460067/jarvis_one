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
};
// namespace
cv::Mat GenerateImageWithKeyPoint(
    const cv::Mat &l_img, const std::vector<cv::KeyPoint> &l_key_points,
    const std::vector<cv::KeyPoint> &predict_pts, const cv::Mat &r_img,
    const std::vector<cv::KeyPoint> &r_key_points, const std::string &l_name,
    const std::string &r_name, std::vector<uint64_t> outlier_pointclass_id) {
  cv::Mat gray_img, loop_match_img;
  cvtColor(l_img, loop_match_img, cv::COLOR_GRAY2RGB);
  std::set<uint64_t> class_id(outlier_pointclass_id.begin(),
                              outlier_pointclass_id.end());
  //
  for (auto &&keypoint : (l_key_points)) {
    double len = std::min(1.0, 1.0 * keypoint.octave / 20);
    cv::Scalar color = cv::Scalar(255 * (1 - len), 0, 255 * len);
    cv::circle(loop_match_img, keypoint.pt, 2, color, 2);
  }
  for (auto &&keypoint : r_key_points) {
    cv::circle(loop_match_img, keypoint.pt, 1, cv::Scalar(0, 255, 0), 1);
  }
  CHECK_EQ(outlier_pointclass_id.size(), size_t(1))
      << "Outlier_pointclass_id is used display zupt,please assignment it..";
  if (outlier_pointclass_id[0]) {
    cv::putText(loop_match_img, "ZUPT", cv::Point2f(20, 100),
                cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 3);
  }

  return loop_match_img;
}
std::vector<uint8_t> ToCData(const jarvis::TrackingData &data) {
  //
  //
  const auto &tracking_data = data;
  std::vector<uint8_t> datas;
  std::vector<int> params;
  params.resize(9, 0);
  params[0] = cv::IMWRITE_JPEG_QUALITY;
  params[1] = 60;
  params[2] = cv::IMWRITE_JPEG_PROGRESSIVE;
  params[3] = 0;
  params[4] = cv::IMWRITE_JPEG_OPTIMIZE;
  params[5] = 0;
  params[6] = cv::IMWRITE_JPEG_RST_INTERVAL;
  params[7] = 0;

  auto image_result = GenerateImageWithKeyPoint(
      *tracking_data.data->image, tracking_data.data->key_points, {},
      *tracking_data.data->rimage, tracking_data.data->extend_points,
      "pre_imag", "curr_imag", tracking_data.data->outlier_pointclass_id);

  cv::imencode(".jpeg", image_result, datas, params);
  //

  PoseData pose{
      data.data->pose.translation().x(), data.data->pose.translation().y(),
      data.data->pose.translation().z(), data.data->pose.rotation().w(),
      data.data->pose.rotation().x(),    data.data->pose.rotation().y(),
      data.data->pose.rotation().z()};
  // LOG(INFO)<<pose.qw;
  int lenth = datas.size();
  datas.resize(datas.size() + sizeof(PoseData));
  memcpy((void *)(datas.data() + lenth), (void *)&pose, sizeof(PoseData));
  // for(size_t i  = 0;i<sizeof(PoseData);i++){
  //   std::cout<<int((datas.data() + lenth)[i])<<" ";
  // }
  // std::cout<<std::endl;
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

  } catch (const std::string s) {
    LOG(INFO) << "Devive creat err" << s;
  }
};
//
//

void ZmqComponent::PubLocalData(const jarvis::TrackingData &data) {
  //
  for (auto &dev : device_) {
    dev->tx(ToCData(data));
  }
};

ZmqComponent::~ZmqComponent() {}
//
MpcComponent::MpcComponent() : shm_mod_(new ShmMod()) {}
//
void MpcComponent::Write(const jarvis::TrackingData &data) {
  ModLocPoseFb mpc_data{
      static_cast<uint64_t>(jarvis::common::ToUniversal(data.data->time)*1e2),
      data.data->pose.translation().x(),
      data.data->pose.translation().y(),
      data.data->pose.translation().z(),
      data.data->pose.rotation().x(),
      data.data->pose.rotation().y(),
      data.data->pose.rotation().z(),
      data.data->pose.rotation().w(),
      0,
      static_cast<uint8_t>(data.status)};

  LOG(INFO) << mpc_data.timestamp;
  shm_mod_->SetModByID(vio_id_, reinterpret_cast<void *>(&mpc_data));
  //
  //

  memset(reinterpret_cast<void *>(&mpc_data), 0, sizeof(ModLocPoseFb));
  shm_mod_->GetModByID(vio_id_, reinterpret_cast<void *>(&mpc_data));
  //
  jarvis::transform::Rigid3d read_pose(
      Eigen::Vector3d{mpc_data.x, mpc_data.y, mpc_data.z},
      Eigen::Quaterniond(mpc_data.qw, mpc_data.qx, mpc_data.qy, mpc_data.qz));
  LOG(INFO) << "Read pose: " << mpc_data.timestamp << " " << read_pose;
}
}  // namespace jarvis_pic