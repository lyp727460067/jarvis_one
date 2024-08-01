
#ifndef _ROS_VIWER_H
#define _ROS_VIWER_H
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "Eigen/Eigen"
#include "jarvis/common/id.h"
#include "jarvis/transform/transform.h"
#include "jarvis/transform/transform_interpolation_buffer.h"
#include "ros/ros.h"
namespace jarvis_ros {
class RosViewer {
 public:
  RosViewer(ros::NodeHandle* nh, const std::string& ground_true_file,
            bool enable_trud_true = false);
  jarvis::transform::TransformInterpolationBuffer* GroudTrue() {
    return groud_true_poses_.get();
  }
  void AddPoses(const std::map<jarvis::KeyFrameId,
                               jarvis::transform::TimestampedTransform>& poses,
                const std::string& ns);

  void AddPairIds(
      const std::vector<std::pair<jarvis::KeyFrameId, jarvis::KeyFrameId>>&);

  void Viewer();
  void SavePoses(const std::string& base_dir);

 private:
  std::vector<std::pair<jarvis::KeyFrameId, jarvis::KeyFrameId>> pair_ids_;

  std::unique_ptr<jarvis::transform::TransformInterpolationBuffer>
      groud_true_poses_;
  std::map<std::string, std::map<jarvis::KeyFrameId,
                                 jarvis::transform::TimestampedTransform>>
      poses_;

  ros::NodeHandle* nh_;
  ros::Publisher markpub_;
  struct MarkPub {
    visualization_msgs::Marker mark;
    static int id;
    MarkPub(int idex, int a, std::string ns);
    MarkPub(int index, std::string ns, int line_type);
    void AddPoint(const Eigen::Vector3d& p);
    void AddPoint(const Eigen::Vector3d& p, bool li);
  };
};
}  // namespace jarvis_ros
#endif