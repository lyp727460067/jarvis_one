#include "camera_models/camera_models/camera_factory.h"

#include "memory"
#include "camera_models/camera_models/CataCamera.h"
#include "camera_models/camera_models/EquidistantCamera.h"
#include "camera_models/camera_models/PinholeCamera.h"
#include "camera_models/camera_models/PinholeFullCamera.h"
#include "camera_models/camera_models/ScaramuzzaCamera.h"
#include "ceres/ceres.h"
namespace jarvis {
namespace camera_models {

std::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory() {}

std::shared_ptr<CameraFactory> CameraFactory::instance(void) {
  if (m_instance.get() == 0) {
    m_instance.reset(new CameraFactory);
  }

  return m_instance;
}

CameraPtr CameraFactory::generateCamera(Camera::ModelType modelType,
                                        const std::string& cameraName,
                                        cv::Size imageSize) const {
  switch (modelType) {
    case Camera::KANNALA_BRANDT: {
      EquidistantCameraPtr camera(new EquidistantCamera);

      EquidistantCamera::Parameters params = camera->getParameters();
      params.cameraName() = cameraName;
      params.imageWidth() = imageSize.width;
      params.imageHeight() = imageSize.height;
      camera->setParameters(params);
      return camera;
    }
    case Camera::PINHOLE: {
      PinholeCameraPtr camera(new PinholeCamera);

      PinholeCamera::Parameters params = camera->getParameters();
      params.cameraName() = cameraName;
      params.imageWidth() = imageSize.width;
      params.imageHeight() = imageSize.height;
      camera->setParameters(params);
      return camera;
    }
    case Camera::PINHOLE_FULL: {
      PinholeFullCameraPtr camera(new PinholeFullCamera);

      PinholeFullCamera::Parameters params = camera->getParameters();
      params.cameraName() = cameraName;
      params.imageWidth() = imageSize.width;
      params.imageHeight() = imageSize.height;
      camera->setParameters(params);
      return camera;
    }
    case Camera::SCARAMUZZA: {
      OCAMCameraPtr camera(new OCAMCamera);

      OCAMCamera::Parameters params = camera->getParameters();
      params.cameraName() = cameraName;
      params.imageWidth() = imageSize.width;
      params.imageHeight() = imageSize.height;
      camera->setParameters(params);
      return camera;
    }
    case Camera::MEI:
    default: {
      CataCameraPtr camera(new CataCamera);

      CataCamera::Parameters params = camera->getParameters();
      params.cameraName() = cameraName;
      params.imageWidth() = imageSize.width;
      params.imageHeight() = imageSize.height;
      camera->setParameters(params);
      return camera;
    }
  }
}

CameraPtr CameraFactory::generateCameraFromYamlFile(
    const std::string& filename) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  CHECK(fs.isOpened()) << filename << " Open faied..";
  Camera::ModelType modelType = Camera::MEI;
  if (!fs["model_type"].isNone()) {
    std::string sModelType;
    fs["model_type"] >> sModelType;

    if (sModelType ==  "KANNALA_BRANDT") {
      LOG(INFO) << "start kannala_brandt";
      modelType = Camera::KANNALA_BRANDT;
    } else if (sModelType ==  "mei") {
      modelType = Camera::MEI;
    } else if (sModelType == "scaramuzza") {
      modelType = Camera::SCARAMUZZA;
    } else if (sModelType== "pinhole") {
      modelType = Camera::PINHOLE;
    } else if (sModelType =="PINHOLE_FULL") {
      modelType = Camera::PINHOLE_FULL;
    } else {
      std::cerr << "# ERROR: Unknown camera model: " << sModelType << std::endl;
      return CameraPtr();
    }
  }

  switch (modelType) {
    case Camera::KANNALA_BRANDT: {
      EquidistantCameraPtr camera(new EquidistantCamera);

      EquidistantCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(filename);
      camera->setParameters(params);
      return camera;
    }
    case Camera::PINHOLE: {
      PinholeCameraPtr camera(new PinholeCamera);

      PinholeCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(filename);
      camera->setParameters(params);
      return camera;
    }
    case Camera::PINHOLE_FULL: {
      PinholeFullCameraPtr camera(new PinholeFullCamera);

      PinholeFullCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(filename);
      camera->setParameters(params);
      return camera;
    }
    case Camera::SCARAMUZZA: {
      OCAMCameraPtr camera(new OCAMCamera);

      OCAMCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(filename);
      camera->setParameters(params);
      return camera;
    }
    case Camera::MEI:
    default: {
      CataCameraPtr camera(new CataCamera);

      CataCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(filename);
      camera->setParameters(params);
      return camera;
    }
  }

  return CameraPtr();
}
}  // namespace camera_models

}  // namespace jarvis