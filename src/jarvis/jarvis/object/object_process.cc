#include "jarvis/object/object_process.h"

#include "common/id.h"
#include "jarvis/key_frame_data.h"
#include "transform/transform.h"
//

namespace jarvis {
namespace object {
//

namespace {

// mapping::object::proto::ObejectData ToProtoObject(const ObejectData& data) {
//   mapping::object::proto::ObejectData proto;
//   *proto.mutable_pose() = ToProto(data.pose);
//   if (data.append != nullptr) {
//     proto.mutable_append()->set_name(data.append->name);
//     for (auto& corner : data.append->corners) {
//       auto proto_coner = proto.mutable_append()->add_corners();
//       proto_coner->set_x(corner.x);
//       proto_coner->set_y(corner.y);
//     }
//   }
//   return proto;
// }
// //
// ObejectData FromProtoObject(const mapping::object::proto::ObejectData& proto)
// {
//   ObejectData result;
//   result.pose = FromProto(proto.pose());
//   if (proto.has_append()) {
//     result.append = std::make_shared<ObejectData::Appended>();
//     result.append->name = proto.append().name();
//     for (const auto& proto_coner : proto.append().corners()) {
//       result.append->corners.push_back({proto_coner.x(), proto_coner.y()});
//     }
//   }
//   return result;
// }

// mapping::object::proto::ObejectDataPose ToProtoObject(
//     const ObejectDataPose& data) {
//   mapping::object::proto::ObejectDataPose proto;
//   proto.set_local_id(data.local_id);
//   *proto.mutable_local_pose() = ToProto(data.local_pose);
//   if (data.data != nullptr) {
//     *proto.mutable_data() = ToProtoObject(*data.data);
//   }
//   return proto;
// }
// //
// ObejectDataPose FromProtoObject(
//     const mapping::object::proto::ObejectDataPose& proto) {
//   ObejectDataPose result;
//   result.local_id = proto.local_id();
//   result.local_pose = FromProto(proto.local_pose());
//   if (proto.has_data()) {
//     result.data =
//         std::make_shared<const ObejectData>(FromProtoObject(proto.data()));
//   }
//   return result;
// }
// //
// mapping::object::proto::ObjectPhysics ToProtoObject(const ObjectPhysics&
// data) {
//   mapping::object::proto::ObjectPhysics proto;
//   *proto.mutable_object() = ToProtoObject(data.object);
//   *proto.mutable_relative_kf_pose() = ToProto(data.relative_kf_pose);

//   *proto.mutable_global_pose() = ToProto(data.global_pose);

//   return proto;
// }
// //
// ObjectPhysics FromProtoObject(
//     const mapping::object::proto::ObjectPhysics& proto) {
//   ObjectPhysics result;
//   result.global_pose = FromProto(proto.global_pose());
//   result.relative_kf_pose = FromProto(proto.relative_kf_pose());
//   result.object = FromProtoObject(proto.object());

//   return result;
// }
// //
// //
// mapping::object::proto::ObjectsPhysics ToProtoObject(
//     const ObjectProcess::ObjectsPhysics& data) {
//   mapping::object::proto::ObjectsPhysics proto;
//   proto.set_time(common::ToUniversal(data.time));
//   for (const auto& object : data.objects) {
//     *proto.add_objects() = ToProtoObject(object);
//   }

//   return proto;
// }
// //
// ObjectProcess::ObjectsPhysics FromProtoObject(
//     const mapping::object::proto::ObjectsPhysics& proto) {
//   ObjectProcess::ObjectsPhysics result;
//   result.time = common::FromUniversal(proto.time());
//   for (const auto& proto_ob : proto.objects()) {
//     result.objects.push_back(FromProtoObject(proto_ob));
//   }
//   return result;
// }

}  // namespace

WapObjectDetect::WapObjectDetect(std::unique_ptr<ObjectDetect> detect)
    : detect_(std::move(detect)) {}
//
std::vector<ObejectDataPose> WapObjectDetect::AddImage(
    const common::Time& time, const std::shared_ptr<cv::Mat>& image,
    const transform::Rigid3d& cam_pose) {
  auto objects = detect_->Detect(*image);
  if (objects.empty()) return {};
  //
  std::vector<ObejectDataPose> result;
  //
  for (const auto& object : objects) {
    result.emplace_back(
        ObejectDataPose{object.first, cam_pose * object.second.pose,
                        std::make_shared<const ObejectData>(object.second)});
  }
  return result;
};
//
//
ObjectProcess::ObjectProcess()
    : transformin_terpolation_buffer_(
          new transform::TransformInterpolationBuffer()) {}
//
void ObjectProcess::AddLandMark(
    const ObejectDataPose& data,
    const std::pair<KeyFrameId, transform::TimestampedTransform>& kf_data) {
  if (insert_local_ids_.count(data.local_id)) return;
  insert_local_ids_.insert(data.local_id);
  if (object_datas_.Contains(kf_data.first)) {
    const auto& objects = object_datas_.at(kf_data.first)->objects;
    //
    object_datas_.at(kf_data.first)
        ->objects.push_back(
            {data, kf_data.second.transform.inverse() * data.local_pose,
             data.local_pose});
  } else {
    object_datas_.Insert(
        kf_data.first,
        std::make_shared<ObjectsPhysics>(ObjectsPhysics{
            kf_data.second.time,
            {ObjectPhysics{data,
                           kf_data.second.transform.inverse() * data.local_pose,
                           data.local_pose}}}));
  }
}
//

void ObjectProcess::Serialization(const std::string& mark_file) {
  // mapping::object::proto::ObjectsPhysicsData proto;

  // for (const auto& object : object_datas_) {
  //   mapping::proto::KeyFrameId id;
  //   id.set_trajectory_id_(object.id.trajectory_id_);
  //   id.set_keyframe_index_(object.id.keyframe_index_);
  //   *proto.add_ids() = id;
  //   *proto.add_object_datas() = ToProtoObject(*object.data);
  // }
  // for (const auto& inser_id : insert_local_ids_) {
  //   proto.add_insert_local_ids(inser_id);
  // }
  // std::ofstream os(mark_file, std::ios::out | std::ios::binary);
  // CHECK(proto.SerializePartialToOstream(&os)) << "seria failed";
  // os.close();
}
//
void ObjectProcess::Load(const std::string& mark_file) {
  // std::ifstream is(mark_file, std::ios::in | std::ios::binary);
  // CHECK(is.good()) << "Load   " << mark_file << " Faild";
  // LOG(INFO) << "Load mark file " << mark_file;
  // mapping::object::proto::ObjectsPhysicsData proto;
  // proto.ParseFromIstream(&is);
  // for (int i = 0; i < proto.ids().size(); i++) {
  //   KeyFrameId id(proto.ids(i).trajectory_id_(),
  //                 proto.ids(i).keyframe_index_());
  //   object_datas_.Insert(id, std::make_shared<ObjectsPhysics>(
  //                                FromProtoObject(proto.object_datas(i))));
  // }
  // for (const auto& id : proto.insert_local_ids()) {
  //   insert_local_ids_.insert(id);
  // }
  // is.close();
}

void ObjectProcess::Update(
    const std::map<KeyFrameId, transform::TimestampedTransform>&
        optimazation_pose) {
  for (const auto& object_id : object_datas_) {
    // 有可能kf被trim 了
    if (optimazation_pose.count(object_id.id)) {
      for (auto& object : object_id.data->objects) {
        const transform::Rigid3d correct_pose =
            optimazation_pose.at(object_id.id).transform *
            object.relative_kf_pose;
        object.global_pose = correct_pose;
      }
    } else {
      auto low_it = optimazation_pose.lower_bound(object_id.id);
      auto upper_it = optimazation_pose.upper_bound(object_id.id);
      //
      if (low_it != optimazation_pose.begin()) {
        --low_it;
      }
      if (upper_it != optimazation_pose.end()) {
        ++upper_it;
      }
      //
      for (auto it = low_it; it != upper_it; ++it) {
        transformin_terpolation_buffer_->Push(it->second.time,
                                              it->second.transform);
      }
      CHECK(transformin_terpolation_buffer_->Has(object_id.data->time));
      const transform::Rigid3d time_point_pose =
          transformin_terpolation_buffer_->Lookup(object_id.data->time);
      //

      for (auto& object : object_id.data->objects) {
        const transform::Rigid3d correct_pose =
            time_point_pose * object.relative_kf_pose;
        object.global_pose = correct_pose;
      }
    }

    transformin_terpolation_buffer_ =
        std::make_unique<transform::TransformInterpolationBuffer>();
  }
}
//
std::map<int, ObjectPhysics> ObjectProcess::GetObjectData(int trajector) {
  std::map<int, ObjectPhysics> result;
  for (auto it = object_datas_.BeginOfTrajectory(trajector);
       it != object_datas_.EndOfTrajectory(trajector); ++it) {
    for (const auto object : it->data->objects) {
      result.emplace(object.object.local_id, object);
    }
  }
  return result;
}
ObjectImageProcess::ObjectImageProcess(
    const ObjectImageProcessOption& option,
    const camera_models::CameraBase* came_base)
    : cam_base_(came_base), bbox_(option.cube_min, option.cube_max) {}
//

//
void ObjectProcess::Clear(int local_id) {
  if (local_id == -1) {
    object_datas_ = MapById<KeyFrameId, ObjectsPhysicsData>();
  }
  for (auto const& object : object_datas_) {
    for (auto it = object.data->objects.begin();
         it != object.data->objects.begin();) {
      if (it->object.local_id == local_id) {
        it = object.data->objects.erase(it);
      } else {
        ++it;
      }
    }
  }
};
//
ObjectImageResult ObjectImageProcess::ProjectObject(
    const transform::Rigid3d& pose, const ObjectPhysics& object, bool global) {
  if (object.object.data == nullptr) {
    return ProjectObjectFaceMark(pose, object);
  }
  if (global) {
    auto result = ProjectObjectFaceMark(pose, object, true);
    result.type = "global";
    return result;
  }
  Eigen::Vector3d box_direction = 0.1 * Eigen::Vector3d::UnitZ();
  Eigen::Vector3d box_direction_origi = Eigen::Vector3d::Zero();

  const transform::Rigid3d delta_pose = object.object.data->pose;

  box_direction = delta_pose * box_direction;
  box_direction_origi = delta_pose * box_direction_origi;
  //
  std::vector<Eigen::Vector2d> coners;
  for (const auto& cor : object.object.data->append->corners) {
    coners.emplace_back(cor.x, cor.y);
  }
  std::vector<Eigen::Vector2d> project_bbox_direction;
  project_bbox_direction.push_back(cam_base_->Project(box_direction_origi));
  project_bbox_direction.push_back(cam_base_->Project(box_direction));
  return {coners, project_bbox_direction, object.global_pose, "online",
          object.object.local_id};
  //
}
ObjectImageResult ObjectImageProcess::ProjectObjectFaceMark(
    const transform::Rigid3d& pose, const ObjectPhysics& object, bool limit) {
  Eigen::Vector3d box_direction = 0.2 * Eigen::Vector3d::UnitZ();
  //
  const transform::Rigid3d delta_pose = pose.inverse() * object.global_pose;
  //   //
  const Eigen::Affine3d af_pose =
      Eigen::Translation3d(delta_pose.translation()) *
      delta_pose.rotation().toRotationMatrix();
  //

  Eigen::AlignedBox3d bbox = bbox_;
  //   bbox.transform(af_pose);
  box_direction = delta_pose * box_direction;

  //   Eigen::AlignedBox3d bbox(delta_pose * bbox_.min(), delta_pose *
  //   bbox_.max());
  if (delta_pose.translation().norm() > 4 && limit) {
    return {{}, {}, object.global_pose, "fack", object.object.local_id};
  }
  //
  std::vector<Eigen::Vector3d> bbox_coners;
  //   bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::BottomLeftFloor));
  //   bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::BottomLeftCeil));
  //   bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::BottomRightCeil));
  //   bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::BottomRightFloor));

  //   bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::TopLeftFloor));
  //   bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::TopLeftCeil));
  //   bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::TopRightCeil));
  //   bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::TopRightFloor));

  bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::BottomLeftFloor));
  bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::TopLeftFloor));
  bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::TopRightFloor));
  bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::BottomRightFloor));
  //
  bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::BottomLeftCeil));
  bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::TopLeftCeil));
  bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::TopRightCeil));
  bbox_coners.push_back(bbox.corner(Eigen::AlignedBox3d::BottomRightCeil));

  //
  std::vector<Eigen::Vector2d> project_bbox_coners;
  for (const auto coner : bbox_coners) {
    const auto t_coner = delta_pose * coner;
    if (t_coner.z() < 0.01f && limit) {
      return {{}, {}, object.global_pose, "fack", object.object.local_id};
    }
    project_bbox_coners.push_back(cam_base_->Project(t_coner));
  };
  //   std::vector<Eigen::Vector2d> in_image_coners;
  //   for (const auto coner : project_bbox_coners) {
  //     if (image_box_.contains(coner)) {
  //       in_image_coners.push_back(coner);
  //     }
  //   }
  //
  //
  //   LOG(INFO)<<bbox.center().transpose();
  std::vector<Eigen::Vector2d> project_bbox_direction;
  Eigen::Vector3d box_direction_origi = Eigen::Vector3d::Zero();
  box_direction_origi = delta_pose * box_direction_origi;
  project_bbox_direction.push_back(cam_base_->Project(box_direction_origi));
  project_bbox_direction.push_back(cam_base_->Project(box_direction));

  ///
  return {project_bbox_coners, project_bbox_direction, object.global_pose,
          "fack", object.object.local_id};
  //
}
}  // namespace object
}  // namespace jarvis