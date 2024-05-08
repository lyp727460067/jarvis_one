#ifndef _JARVIS_VIO_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
#define _JARVIS_VIO_TRANSFORM_TIMESTAMPED_TRANSFORM_H_

#include "jarvis/common/time.h"
#include "jarvis/transform/rigid_transform.h"
namespace jarvis {
namespace transform {

struct TimestampedTransform {
  common::Time time;
  transform::Rigid3d transform;
};

TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time);

}  // namespace transform
}  // namespace jarvis

#endif
