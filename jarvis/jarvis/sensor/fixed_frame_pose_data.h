

#ifndef JRVIS_SENSOR_FIXED_FRAME_POSE_DATA_H_
#define JRVIS_SENSOR_FIXED_FRAME_POSE_DATA_H_

#include <memory>

#include "jarvis/common/time.h"
#include "jarvis/transform/rigid_transform.h"

namespace jarvis {
namespace sensor {

struct FixedFramePoseData {
  common::Time time;
  transform::Rigid3d pose;
};

}  // namespace sensor
}  // namespace jarvis

#endif