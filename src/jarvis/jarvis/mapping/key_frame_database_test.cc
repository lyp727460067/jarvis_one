#include "jarvis/mapping/key_frame_database.h"

#include <dirent.h>
#include <sys/types.h>

#include <opencv2/opencv.hpp>
#include <set>
#include <string>

#include "gtest/gtest.h"
#include "jarvis/common/fixed_ratio_sampler.h"
#include "jarvis/mapping/key_point_exract.h"
#include "mapping/covisibility.h"
namespace jarvis {
namespace mapping {
//

namespace {

std::map<std::string, std::string> map_to_name{
    // {"/home/lyp/data/test_0045/jarvis/data/2024_10_26_16_47/",
    //  "kTestPictureDataDir_2024_10_26_16_47"},
    // {"/home/lyp/data/test_0045/jarvis/data/2024_10_26_16_57",
    //  "kTestPictureDataDir_2024_10_26_16_57"},
    {"/home/lyp/data/long_data/2025_3_12_14_45/",
     "kTestPictureDataDir_2025_3_12_14_45"}};

constexpr char kTestPictureDataDirTarget[] =
    "/home/lyp/data/long_data/2025_3_12_14_45/";

//
constexpr char vocabulary_filebrif[] =
    "/home/lyp/project/mowerpack/mowerpack_all/vslam/src/jarvis/configuration/jarvis.dbow";

std::set<std::string> ReadFileFromDir(const std::string& path) {
  std::set<std::string> fp_set;
  DIR* dir = opendir(path.c_str());
  CHECK(dir);
  struct dirent* entry = nullptr;
  while ((entry = readdir(dir)) != nullptr) {
    if (std::string(entry->d_name) == ".") continue;
    if (std::string(entry->d_name) == "..") continue;
    std::string pic_name = path + std::string(entry->d_name);
    fp_set.emplace(pic_name);
  }
  closedir(dir);
  // //
  LOG(INFO) << "dir path has file size :" << fp_set.size();
  return fp_set;
  //
}

}  // namespace

uint64_t GetTimeFromeName(const std::string& name) {
  CHECK(!name.empty());
  auto it = name.find_last_of('/');
  std::string outdir = name.substr(0, it + 1);
  const std::string file_name =
      name.substr(it + 1, name.size() - outdir.size());
  auto it1 = file_name.find_last_of('.');
  return std::stol(file_name.substr(0, it1));
}

//

//
class KeyFrameDatabaseTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    voc_ = std::make_unique<dbow::Vocabulary>(
        dbow::GetVocabulary(0, vocabulary_filebrif));
    data_base_ =
        std::make_unique<KeyFrameDataBase>(KeyFrameDataBaseOption{});
    des_extractor_ =
        std::make_unique<DescriptorExtract>(DescriptorExtractOption{});
    key_extractor_ = std::make_unique<KeyPointExtract>(KeyPointExtractOption{});
  }
  std::shared_ptr<const KeyFrameData::Data> ComuteData(
      uint64_t time, const std::vector<cv::Mat>& image, dbow::Vocabulary* voc);

  std::unique_ptr<DescriptorExtract> des_extractor_;
  std::shared_ptr<dbow::Vocabulary> voc_;
  std::unique_ptr<KeyFrameDataBase> data_base_;
  std::unique_ptr<KeyPointExtract> key_extractor_;
};
const std::vector<std::vector<int>> track_sequence = {{0, 1}, {2}, {3}};
//
cv::Mat ComputeDescriptors(const cv::Mat& image,
                           std::vector<cv::KeyPoint>& keyPoints) {}

//
//
//
std::shared_ptr<const KeyFrameData::Data> KeyFrameDatabaseTest::ComuteData(
    uint64_t time, const std::vector<cv::Mat>& images, dbow::Vocabulary* voc) {
  //
  KeyFrameData::Data data{common::Time(common::FromSeconds(time)),
                          transform::Rigid3d::Identity()};

  for (int s = 0; s < 1; s++) {
    std::vector<cv::KeyPoint> key_points =
        key_extractor_->Extract(images[track_sequence[s][0]], 1000);
    if (key_points.empty()) continue;
    Descriptors descriptors =
        des_extractor_->Extract(images[track_sequence[s][0]], key_points);
    for (int j = 0; j < key_points.size(); j++) {
      const FeatureId feat_id(s, j);
      data.descriptors.Insert(feat_id, descriptors[j]);
    }
  }
  // /
  data.dbow_data = voc->Transform(data.descriptors, 4);
  return std::make_shared<const KeyFrameData::Data>(data);
}
//
uint64_t GetTimeFromName(const std::string& name) {
  CHECK(!name.empty());
  auto it = name.find_last_of('/');
  std::string outdir = name.substr(0, it + 1);
  const std::string file_name =
      name.substr(it + 1, name.size() - outdir.size());
  auto it1 = file_name.find_last_of('.');
  // LOG(INFO)<<std::stol(file_name.substr(0, it1-2));
  return std::stol(file_name.substr(0, it1 - 2));
}
//
std::string GetFromName(const std::string& name) {
  CHECK(!name.empty());
  auto it1 = name.find_last_of('.');
  // LOG(INFO)<<(name.substr(0, it1-2));
  return name.substr(0, it1 - 2);
}

struct ImageData {
  uint64_t time;
  // cv::Mat images;
  std::string image_name;
  static std::map<uint64_t, ImageData> Parse(const std::string& dir_file) {
    const auto image_files_name = ReadFileFromDir(dir_file);
    CHECK(!image_files_name.empty()) << "Need Image file in dir..";
    std::map<uint64_t, ImageData> result;
    //

    for (const auto& file : image_files_name) {
#ifdef CHECK_DATA
      static uint64_t last_imu_time = GetTimeFromName(file);
      LOG(INFO) << (GetTimeFromName(file) - last_imu_time);
      last_imu_time = GetTimeFromName(file);

#endif
      //   LOG(INFO) << "Read Image: " << file;
      LOG_IF_EVERY_N(
          ERROR,
          !result
               .emplace(GetTimeFromName(file),
                        ImageData{GetTimeFromName(file), GetFromName(file)})
               .second,
          1000)
          << "Image time duplicate..";
    }
    return result;
  }
};

TEST_F(KeyFrameDatabaseTest, FindCanditate) {
  //
  std::map<KeyFrameId, std::string> id_map_name;
  std::map<KeyFrameId, std::shared_ptr<const KeyFrameData::Data>> datas;
  int t = 0;
  int  load_size  =1000; 
  for (auto dir : map_to_name) {
    auto image_datas = ImageData::Parse(dir.first + "/image/");
    int index = 0;
    common::FixedRatioSampler sampler();
    for (const auto& image : image_datas) {
     if(--load_size ==0)break;
    //   if (!sampler.Pulse()) continue;
      LOG_EVERY_N(INFO, 100) << "parse :" << image.second.image_name + "_0.jpg";
      const cv::Mat lr_image =
          cv::imread(image.second.image_name + "_0.jpg", cv::IMREAD_GRAYSCALE);
      const cv::Mat vr_image =
          cv::imread(image.second.image_name + "_1.jpg", cv::IMREAD_GRAYSCALE);
      uint64_t time = image.second.time;
      if (lr_image.empty() || vr_image.empty()) continue;
      auto data = ComuteData(
          time,
          std::vector<cv::Mat>{lr_image(cv::Rect(0, 0, 640, 544)).clone(),
                               lr_image(cv::Rect(640, 0, 640, 544)).clone(),
                               vr_image(cv::Rect(0, 0, 544, 640)).clone(),
                               vr_image(cv::Rect(544, 0, 544, 640)).clone()},
          voc_.get());
      datas.emplace(KeyFrameId{t, index++}, data);
      data_base_->AddData(KeyFrameId(t, index), data);
    }
    t++;
  }

  auto target = datas.begin()->second;
  auto target_mame = id_map_name[datas.begin()->first];
  LOG(INFO) << target_mame;
  std::set<KeyFrameId> exclude_ids;
//   for (int i = -100; i < 100; i++) {
//     exclude_ids.emplace(0, 1000 - i);
//   }

  //
  for (auto& d : datas) {
    auto result = data_base_->FindSimilarCandidate(d.second, {},0);
    for (auto const& r : result) {
      LOG(INFO) << "result  :" << r.first << " score: " << r.second
                << " name:" << d.first;
    }
  }

//   auto image_datas =
//       ImageData::Parse(std::string(kTestPictureDataDirTarget) + "/image/");
//   int index = 0;
//   for (const auto& image : image_datas) {
//     common::FixedRatioSampler sampler(0.001);
//     if (!sampler.Pulse()) continue;
//     uint64_t time = image.second.time;
//     const cv::Mat lr_image =
//         cv::imread(image.second.image_name + "_0.jpg", cv::IMREAD_GRAYSCALE);
//     const cv::Mat vr_image =
//         cv::imread(image.second.image_name + "_1.jpg", cv::IMREAD_GRAYSCALE);
//     //
//     LOG(INFO) << "target :" << image.second.image_name;
//     //

//    if(lr_image.empty()||vr_image.empty() )continue;
//     auto query_data = ComuteData(
//         time,
//         std::vector<cv::Mat>{lr_image(cv::Rect(0, 0, 640, 544)).clone(),
//                              lr_image(cv::Rect(640, 0, 640, 544)).clone(),
//                              vr_image(cv::Rect(0, 0, 544, 640)).clone(),
//                              vr_image(cv::Rect(544, 0, 544, 640)).clone()},
//         voc_.get());

//     auto result = data_base_->FindSimilarCandidate(query_data, {});
//     for (auto const& r : result) {
//       LOG(INFO) << "result  :" << r.first << " score: " << r.second << " name "
//                 << id_map_name[r.first];
//     }
//   }
  }
}  // namespace mapping
}  // namespace jarvis