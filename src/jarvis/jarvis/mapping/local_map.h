#ifndef JARVIS_MAPPING_LOCAL_MAP_
#define JARVIS_MAPPING_LOCAL_MAP_
#include <map>
#include <memory>
#include <mutex>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "jarvis/common/id.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/key_point_exract.h"
#include "jarvis/mapping/local_map_optimization.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/mapping/data_culling.h"
#include "jarvis/common/fixed_ratio_sampler.h"
//
namespace jarvis {
namespace mapping {
//
struct LocalMapOption {
  int max_kf_num = 100;
  KeyFrameDataBaseOption key_frame_data_option;
  match::ProjectionOption local_track_project_search_option;
  DataCullingOption data_culling_option;

  std::vector<Eigen::AlignedBox2i> image_boxs;
  double culling_sampler = 0.2;
  int compute_map_point_min_des_num = 5;
  std::map<int, camera_models::CameraPtr> cameras;
};

//
struct KeyFrameIdWithPose {
  KeyFrameId id;
  transform::Rigid3d local_pose;
};
//
//
struct LocalMapConstraint {};

class LocalMap {
 public:
  //
  
  LocalMap(const LocalMapOption &option, const transform::Rigid3d &local_pose);
  //
  transform::Rigid3d LocalPose()const { return data_.local_pose; }
  LocalMap(const LocalMap &local_map)
      : LocalMap(local_map.options_, local_map.data_.local_pose) {
    local_to_ref_ = local_map.local_to_ref_;
  }

  //
  bool operator=(LocalMap &&local_map);
  bool operator=(const LocalMap &local_map);
  //
  ~LocalMap();
  void AddKeyFrameData(const KeyFrameId &kf_id,
                       const KeyFrameData &key_frame_data);
  //
  void Opimization();
  //
  void Finish();
  bool IsFinish() { return finish_; }
  bool IsOptimization() { return is_optimization ;};
  //
  void UpdateExistData(const LocalMap&rhs);
  void UpdadataExtendFinishData(bool f);
  //
  Eigen::Vector3d GetMapPointPosw(const MapPointId &mp_id) const {
    return data_.local_pose * data_.map_points.at(mp_id).data->pos;
  }
  //
  void Opimization(const std::vector<LocalMapConstraint>& constrants);
 
  //
  //
  const MapById<MapPointId, MapPointData> &AllMapPoints() const {
    return data_.map_points;
  }
  const MapById<KeyFrameId, const KeyFrameData> &AllKeyFrameDatas()const{
    return data_.key_frames_datas;
  }
  //
  // /
  const std::map<KeyFrameId, transform::Rigid3d> &AllKeyFrameRefPose() {
    return data_.key_frames_ref_pose;
  }
  //
  std::set<KeyFrameId> GetTrimBeforKeyFrameId() {
    return data_.trim_befor_key_frame_id;
  };
  //
  int Size() { return data_.key_frames_datas.size(); }
  const mapping::Covisibility *GetCovisibility() const {
    return &data_.covisibility;
  }
  //
  std::map<MapPointId, MapPointData> GetKeyFrameMapPoints(const KeyFrameId &id);
  //
  std::pair<std::map<FeatureId, MapPointId>, MapById<MapPointId, MapPointData>>
  GetKeyFrameMapPointsData(const KeyFrameId &frame_id) const;

  //
  struct Data {
    transform::Rigid3d local_pose;                                // 局部地图相对全局系的坐标(旋转量与全局坐标一致,只有平移量)
    MapById<KeyFrameId, const KeyFrameData> key_frames_datas;     // 局部地图中每帧的数据
    std::map<KeyFrameId, transform::Rigid3d> key_frames_ref_pose; // 每帧相对local_pose的位姿
    MapById<MapPointId, MapPointData> map_points;                 // 每个地图点在局部地图坐标系的位置
    Covisibility covisibility;
    std::set<KeyFrameId> trim_befor_key_frame_id;
  };
  //
  Data *MutableData() { return &data_; }
  const Data ConstData() const { return data_; }

 public:
  //
  Data data_;
  void ComputeMapPointDistinctiveDescriptors(const MapPointId &id);
  void FuseMapPoint(
      const KeyFrameId &kf_id,
      const std::map<MapPointId, std::map<KeyFrameId, FeatureId>> &matches);
  bool TrimMapPoint(const MapPointId &id);
  void TrimKeyFrame(const KeyFrameId &id);
  //
  std::unique_ptr<Eigen::Vector2d> ProjectMapPointToKeyFrame(
      const MapPointId &mp_id, const KeyFrameId &kf_id) const;

  std::map<int, camera_models::CameraPtr> cameras_;
  //

  void TrimRedundancy();
  //
  LocalMapOption options_;

  //
  std::unique_ptr<KeyFrameDataBase> key_frame_data_base_;

  std::unique_ptr<common::FixedRatioSampler> culling_sampler_;
  std::unique_ptr<DataCulling> data_culling_;
  bool finish_ = false;
  bool  is_optimization= false;

  class LocalDataFuse : public DataFuse {
   public:
    LocalDataFuse(LocalMap *local_map);

    void FuseMapPoint(
        const KeyFrameId &key_frame_id,
        const std::map<MapPointId, std::map<KeyFrameId, FeatureId>> &matches)
        override;

    void CullKeyFrame(const std::set<KeyFrameId> &target) override;
    const MapById<MapPointId, mapping::MapPointData> GetMapPoints(
        const KeyFrameId &id) override;
    //
    const std::set<KeyFrameId> GetMapObservations(
        const MapPointId &map_point_id) override;
    bool PorjectPoint(const transform::Rigid3d& cam_pose, const Eigen::Vector3d& point,
            int s, Eigen::Vector2d* p) override;
    //
    const MapById<KeyFrameId, const KeyFrameData> &GetAllKeyFramesData()const override;
    std::vector<std::pair<KeyFrameId, int>> GetKeyLevelConnectedKeyFrames(
        const KeyFrameId &frame_id, const std::vector<int> &levels) override;
    LocalMap *local_map_;
  };
  std::mutex mutex_;
  transform::Rigid3d local_to_ref_;
  std::unique_ptr<LocalDataFuse> data_fuse_;
};
//
class ActiveLocalMap {
 public:
  ActiveLocalMap(const LocalMapOption &option) : local_map_option_(option) {}
  std::vector<std::shared_ptr<LocalMap>> GetLocalMap(){return localmaps_;}
  //
  std::shared_ptr<LocalMap> FrontFinish() { return front_finsh_; }
  void AddKeyFrameData(const KeyFrameId &, const KeyFrameData &data);

 private:
  std::vector<std::shared_ptr<LocalMap>> localmaps_;
  std::shared_ptr<LocalMap> front_finsh_ = nullptr;
  void FinishLocalMap();
  void AddLocalMap(const LocalMapOption &local_option,
                   const transform::Rigid3d &local_pose);
  LocalMapOption local_map_option_;
};
}  // namespace mapping
}  // namespace jarvis

#endif