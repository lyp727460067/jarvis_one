#ifndef _JARVIS_CAMERA_MODELS_CAMERAFACTORY_H
#define _JARVIS_CAMERA_MODELS_CAMERAFACTORY_H

#include <memory>
#include "opencv2/core/core.hpp"

#include "jarvis/camera_models/camera_models/camera.h"
#include "jarvis/option_parse.h"
namespace jarvis {
namespace camera_models {

class CameraFactory {
 public:
  CameraFactory();

  static std::shared_ptr<CameraFactory> instance(void);

  CameraPtr generateCamera(Camera::ModelType modelType,
                           const std::string& cameraName,
                           cv::Size imageSize) const;

  CameraPtr generateCameraFromYamlFile(const std::string& filename);
  CameraPtr GenerateCameraFromOption(const CameraOption& options);

 private:
  static std::shared_ptr<CameraFactory> m_instance;
};

}  // namespace camera_models
}  // namespace jarvis
#endif
