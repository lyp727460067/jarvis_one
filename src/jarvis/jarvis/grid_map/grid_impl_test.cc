#include "jarvis/grid_map/grid_impl.h"
#include "opencv2/opencv.hpp"
#include <dirent.h>
#include <sys/types.h>
#include <fstream>
#include "glog/logging.h"
#include "gtest/gtest.h"
namespace jarvis {
namespace grid_map {
//
class GridMapTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::map<int, GridMapOption> options;
    options[1];
    options[2].insert_free_sector_space =true;
    options[2].insert_free_min_distance=0.2;
    options[3];
    options[4];
    options[5].insert_free_sector_space=true;
    options[5].insert_free_min_distance=0.2;
    grid_map_ = GridMap::Create(options);
  }

 protected:
  std::unique_ptr<GridMap> grid_map_;
};
//
uint64_t GetTimeFromName(const std::string& name) {
  CHECK(!name.empty());
  auto it = name.find_last_of('/');
  auto it1 = name.find_first_of('_');

  std::string outdir = name.substr(0, it + 1);
  const std::string file_name = name.substr(it + 1, it1 - outdir.size());
  return std::stol(file_name.substr(0, it1));
}
//
//
std::string GetFromName(const std::string& name) {
  CHECK(!name.empty());
  auto it1 = name.find_last_of('.');
  return name.substr(0, it1 - 2);
}
//
std::set<uint64_t> GetTimes(const std::set<std::string>& files) {
  std::set<uint64_t> result;
  for (auto& f : files) {
    result.insert(GetTimeFromName(f));
  }
  return result;
}

struct Point3d {
  Eigen::Vector3d point;
};
//
std::istringstream& operator>>(std::istringstream& ifs, Point3d& point_data) {
  std::string type;
  ifs >> point_data.point.x() >> point_data.point.y() >> point_data.point.z();
  return ifs;
}

struct Pose3d {
  uint64_t time;
  Eigen::Vector3d t;
  Eigen::Quaterniond q;
};

std::istringstream& operator>>(std::istringstream& ifs, Pose3d& pose) {
  std::string type;
  float ration =  0.001;
  ifs >> pose.t.x() >> pose.t.y()>> pose.t.z();
  pose.t*=ration;
  double yaw, pith, roll;
  ifs >> yaw >> pith >> roll;

  pose.q = transform::RollPitchYaw(roll*ration, pith*ration, yaw*ration);
  return ifs;
}
//
template <typename TypeName>
std::vector<TypeName> ReadFile(const std::string& txt) {
  std::ifstream file;
  file.open(txt);
  if (!file.good()) {
    // LOG(ERROR) << txt << " not exist";
    file.close();
    return {};
  }
  // CHECK(file.good()) << txt;
  std::string line;
  std::vector<TypeName> result;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    try {
      TypeName data;
      iss >> data;
      // CHECK(file.good());
      result.push_back(data);
    } catch (...) {
    }
  }
  file.close();
  return result;
}

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
    LOG(INFO)<<pic_name;
  }
  closedir(dir);
  // //
  LOG(INFO) << "dir path has file size :" << fp_set.size();
  return fp_set;
  //
}
//
std::map<int, std::string> type_name{{1, "ground"},
                                     {2, "obstale"},
                                     {3, "seg_soil"},
                                     {4, "seg_grass"},
                                     {5, "seg_background"}};
//
std::vector<AiObject> ParseObjecs(const std::string& dir) {
  std::set<std::string> file_name = ReadFileFromDir(dir);
  auto times = GetTimes(file_name);
  uint64_t fisrt_time = *times.begin();
  std::vector<AiObject> resutl;
  for (const auto& t : times) {
    // LOG(INFO) << int(t - fisrt_time);
    fisrt_time = t; 
    // LOG(INFO) << dir + std::to_string(t) + "_" + "pose.txt";
    auto pose = ReadFile<Pose3d>(dir + std::to_string(t) + "_" + "pose.txt");
    if(pose.empty()){
      // LOG(ERROR)<<dir + std::to_string(t) + "_" + "pose.txt not exist";
      continue;
    }
    resutl.emplace_back();
    // CHECK(!pose.empty())<<dir + std::to_string(t) + "_" + "pose.txt";
    resutl.back().pose.rotaion = pose[0].q;
    resutl.back().pose.tanslation = pose[0].t;
    resutl.back().time = t;
    //
    LOG(INFO) << pose[0].t.transpose();  
    for (auto const& name : type_name) {

      auto points = ReadFile<Point3d>(dir + std::to_string(t) + "_" +
                                      name.second + ".txt");
      // resutl.back().points_clouds.emplace(name.first);
      if (points.empty()) {
        LOG(INFO) << t << " "
                  << dir + std::to_string(t) + "_" + name.second + ".txt";
 
      }
      for (auto& p : points) {
        resutl.back().points_clouds[name.first].push_back(
            p.point.cast<float>());
      }
    }
  }
  return resutl;
}
constexpr char kTesData[] =
    "/home/lyp/data/ai_data/2024_12_5_gz/perceive_map_object_save/";

const int map_size=500;
//
TEST_F(GridMapTest, GridMapTesInsert) {
  auto ai_object =  ParseObjecs( kTesData);
  LOG(INFO)<<"parse done ,size:"<<ai_object.size();

  std::map<int, cv::Mat> images;
  for (auto n : type_name) {
    images[n.first] = cv::Mat(map_size, map_size, CV_8UC1);
  }
  const std::string test_string_file_dir = "/home/lyp/data/ai_data/local_map/";
  for (auto const& a : ai_object) {
    grid_map_->Insert(a);
    GridImpl* grid = dynamic_cast<GridImpl*>(grid_map_.get());
    for (int i = -map_size / 2; i < map_size / 2; i++) {
      for (int j = -map_size / 2; j < map_size / 2; j++) {
        auto vs = grid_map_->IndexValue(Eigen::Vector2f(i * 0.01, j * 0.01));
        for (const auto& v : vs) {
          // images[v.first].at<uint8_t>(map_size - (i + map_size / 2)-1,
          //                             j + map_size / 2) = v.second.p;
          images[v.first].at<uint8_t>(  map_size - (j + map_size / 2)-1,
                                      i + map_size / 2) = v.second.p;


        }
      }
    }
    for (auto const& imae : images) {
      cv::imwrite(test_string_file_dir + std::to_string(a.time) + "_" +
                      type_name[imae.first] + ".png",
                  imae.second);
      if (imae.first == 5) {
        cv::imshow("grid", imae.second);
        cv::waitKey(0);
      }
    }

    // grid->ToPgn("/home/lyp/data/ai_data/");
    // cv::waitKey(0);
  }
  GridImpl* grid = dynamic_cast<GridImpl*>(grid_map_.get());
  grid->ToPgn("/home/lyp/data/ai_data/");
}
}  // namespace grid_map
}  // namespace jarvis