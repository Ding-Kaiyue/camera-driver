#include "camera_driver/application/camera_driver_interface.hpp"

#include <memory>

#include "application/camera_driver_service.h"

namespace camera_driver {

std::shared_ptr<CameraDriverInterface> createCameraDriver() {
    return std::make_shared<CameraDriverService>();
}

}  // namespace camera_driver
