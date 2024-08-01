
#include "ros_viewer.h"

#include "geometry_msgs/Point.h"
#include "glog/logging.h"
#include "random"
#include "fstream"
#include "jarvis/transform/transform.h"
namespace jarvis_ros {
namespace {
using namespace jarvis;
constexpr char kFileName[] = "global_pose.txt";
constexpr char kGroundTrueFile[] = "stamped_traj_estimate.txt";
constexpr float kInitialHue = 0.69f;
constexpr float kSaturation = 0.85f;
constexpr float kValue = 0.77f;

using FloatColor = std::array<float, 3>;
static FloatColor HsvToRgb(const float h, const float s, const float v) {
  const float h_6 = (h == 1.f) ? 0.f : 6 * h;
  const int h_i = std::floor(h_6);
  const float f = h_6 - h_i;

  const float p = v * (1.f - s);
  const float q = v * (1.f - f * s);
  const float t = v * (1.f - (1.f - f) * s);

  if (h_i == 0) {
    return {{v, t, p}};
  } else if (h_i == 1) {
    return {{q, v, p}};
  } else if (h_i == 2) {
    return {{p, v, t}};
  } else if (h_i == 3) {
    return {{p, q, v}};
  } else if (h_i == 4) {
    return {{t, p, v}};
  } else if (h_i == 5) {
    return {{v, p, q}};
  } else {
    return {{0.f, 0.f, 0.f}};
  }
}

static FloatColor GetColor(int id) {
  constexpr float kGoldenRatioConjugate = 0.6180339887498949f;
  const float hue = std::fmod(kInitialHue + kGoldenRatioConjugate * id, 1.f);
  return HsvToRgb(hue, kSaturation, kValue);
}

std::vector<std::pair<common::Time, transform::Rigid3d>> ReadGroundTrueFromFile(
    const std::string& txt) {
  std::vector<std::pair<common::Time, transform::Rigid3d>> result;
  std::ifstream file;
  file.open(txt);
  if (!file.good()) return {};
  CHECK(file.good());
  std::string line;
  int state = 0;
  double time_firs = 0;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    double x, y, z, qw, qx, qy, qz;
    double t;
    if (!(iss >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      continue;
    }  // error
    if (state == 0) {
      state = 1;
      time_firs = t;
      continue;
    }
    if (state == 1) {
      const transform::Rigid3d pose{{x, y, z}, {qw, qx, qy, qz}};
      const common::Time time = common::Time(common::FromSeconds(time_firs));
      result.push_back({time, pose});
      state = 2;
    }
    const common::Time time = common::Time(common::FromSeconds(t));
    const transform::Rigid3d pose{{x, y, z}, {qw, qx, qy, qz}};
    result.push_back({time, pose});
  }
  return result;
}

void WriteGlobPose(
    const std::map<KeyFrameId, transform::TimestampedTransform> poses,
    std::string file_name) {
  std::ofstream out_stream(file_name, std::ios_base::out);
  out_stream.setf(std::ios::fixed, std::ios::floatfield);
  out_stream.precision(10);
  for (const auto& pose_with_node : poses) {
    const auto& pose = pose_with_node.second;
    double time = ToUniversal(pose.time) * 1e-7;
    out_stream << time << " " << pose.transform.translation().x() << " "
               << pose.transform.translation().y() << " "
               << pose.transform.translation().z() << " "
               << pose.transform.rotation().x() << " "
               << pose.transform.rotation().y() << " "
               << pose.transform.rotation().z() << " "
               << pose.transform.rotation().w() << std::endl;
  }
  out_stream.close();
};

}  // namespace

RosViewer::RosViewer(ros::NodeHandle* nh, const std::string& dir,
                     bool enable_trud_true)
    : nh_(nh) {
  markpub_ =
      nh_->advertise<visualization_msgs::MarkerArray>("loop_detection", 10);
  // if (enable_trud_true) {

  auto poses = ReadGroundTrueFromFile(dir + kGroundTrueFile);
  if (!poses.empty()) {
    groud_true_poses_ =
        std::unique_ptr<transform::TransformInterpolationBuffer>(
            new transform::TransformInterpolationBuffer());
    CHECK(!poses.empty());
    LOG(INFO) << "groud true size:" << poses.size();
    for (const auto& pose : poses) {
      groud_true_poses_->Push(pose.first, pose.second);
    }
  }

  // }
}
void RosViewer::AddPoses(
    const std::map<KeyFrameId, transform::TimestampedTransform>& poses,
    const std::string& ns) {
  poses_[ns] = poses;
}

void RosViewer::AddPairIds(
    const std::vector<std::pair<KeyFrameId, KeyFrameId>>& ids) {
  pair_ids_ = ids;
}

void RosViewer::SavePoses(const std::string& base_dir) {
  for (const auto poses : poses_) {
    LOG(INFO) << "write pose " << (base_dir + poses.first + ".txt");
    WriteGlobPose(poses.second, base_dir + poses.first + ".txt");
  }
}

RosViewer::MarkPub::MarkPub(int idex, int a, std::string ns) {
  mark.header.frame_id = "map";
  mark.ns = ns;
  mark.header.stamp = ::ros::Time::now();
  mark.id = idex;
  mark.action = visualization_msgs::Marker::ADD;
  mark.type = visualization_msgs::Marker::POINTS;
  // mark.type = visualization_msgs::Marker::ARROW;
  std::default_random_engine e(idex);
  std::uniform_real_distribution<float> ran(0, 1);
  mark.lifetime = ros::Duration(0);
  auto color = GetColor(idex + a + 3);
  mark.scale.x = 0.03;
  mark.scale.y = 0.03;
  mark.scale.z = 0.03;
  mark.color.a = 1;
  mark.color.r = color[1];
  mark.color.b = color[0];
  mark.color.g = color[2];
}

RosViewer::MarkPub::MarkPub(int index, std::string ns, int line_type) {
  mark.header.frame_id = "map";
  mark.ns = ns;
  mark.header.stamp = ::ros::Time::now();
  mark.id = index;
  mark.action = visualization_msgs::Marker::ADD;
  mark.type = visualization_msgs::Marker::LINE_LIST;
  // mark.type = visualization_msgs::Marker::ARROW;
  // std::default_random_engine e(idex);
  // std::uniform_real_distribution<float> ran(0, 1);
  mark.lifetime = ros::Duration(0);
  auto color = GetColor(index);
  mark.scale.x = 0.01;
  mark.scale.y = 0.01;
  mark.scale.z = 0.01;
  mark.color.a = 1;
  mark.color.r = 1;
  mark.color.b = 0;
  mark.color.g = 0;
}
void RosViewer::MarkPub::AddPoint(const Eigen::Vector3d& p) {
  geometry_msgs::Point point;
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  mark.points.push_back(point);
}
void RosViewer::MarkPub::AddPoint(const Eigen::Vector3d& p, bool li) {
  geometry_msgs::Point point;
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  if (li) {
    mark.color.g = 1;
  }
  mark.points.push_back(point);
}

void RosViewer::Viewer() {
  int index = 0;
  std::vector<MarkPub> mark_pubs;
  if (groud_true_poses_ != nullptr) {
    mark_pubs.push_back(MarkPub(index++, 1, "groud_true"));
  }
  int i = 0;
  for (auto pose : poses_) {
    mark_pubs.push_back(MarkPub(index++,
                                pose.second.begin()->first.trajectory_id_,
                                "pose_" + std::to_string(i)));
    i++;
  }

  for (int i = 0; i < int(pair_ids_.size()); i++) {
    mark_pubs.push_back(MarkPub(index++, "pose_constraint_line", 1));
  }
  if (poses_.empty()) return;
  const auto global_poses = poses_["global"];
  if (global_poses.empty()) return;

  index = 0;
  if (groud_true_poses_ != nullptr) {
    auto first_pose =
        groud_true_poses_->Lookup(global_poses.begin()->second.time);
    auto global_first_pose = global_poses.begin()->second.transform;
    const auto global_to_grue = global_first_pose * first_pose.inverse();
    for (auto const& pose : poses_["global"]) {
      auto relative_glole_pose =
          global_to_grue * groud_true_poses_->Lookup(pose.second.time);
      mark_pubs[index].AddPoint(relative_glole_pose.translation());
    }
    index++;
  }
  for (auto const& poses : poses_) {
    for (auto const& pose : poses.second) {
      mark_pubs[index].AddPoint(pose.second.transform.translation());
    }
    index++;
  }

  std::map<KeyFrameId, transform::TimestampedTransform> all_global_poses;
  for (auto& pose : poses_) {
    all_global_poses.insert(pose.second.begin(), pose.second.end());
  }
  for (const auto& r : pair_ids_) {
    try {
      if (r.first.trajectory_id_ != r.second.trajectory_id_) {
        mark_pubs[index].AddPoint(
            all_global_poses.at(r.first).transform.translation(), true);
        mark_pubs[index].AddPoint(
            all_global_poses.at(r.second).transform.translation(), true);

      } else {
        mark_pubs[index].AddPoint(
            all_global_poses.at(r.first).transform.translation());
        mark_pubs[index].AddPoint(
            all_global_poses.at(r.second).transform.translation());
      }

    } catch (...) {
      LOG(ERROR) << r.first << r.second << "May Not In PoseGraph";
      continue;
    }
    index++;
  }

  visualization_msgs::MarkerArray marks;
  for (auto const& mark_pub : mark_pubs) {
    marks.markers.push_back(mark_pub.mark);
  }
  markpub_.publish(marks);
}
}  // namespace jarvis
