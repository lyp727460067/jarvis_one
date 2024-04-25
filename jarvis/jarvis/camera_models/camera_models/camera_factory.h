#ifndef _JARVIS_CAMERA_MODELS_CAMERAFACTORY_H
#define _JARVIS_CAMERA_MODELS_CAMERAFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "camera_models/camera_models/camera.h"
namespace jarvis {
namespace camera_models {

class CameraFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraFactory();

  static boost::shared_ptr<CameraFactory> instance(void);

  CameraPtr generateCamera(Camera::ModelType modelType,
                           const std::string& cameraName,
                           cv::Size imageSize) const;

  CameraPtr generateCameraFromYamlFile(const std::string& filename);

 private:
  static boost::shared_ptr<CameraFactory> m_instance;
};

}  // namespace camera_models
}  // namespace jarvis
#endif
