#pragma once

#include <memory>
#include <string>

namespace camera_driver {

class CameraDriverInterface {
public:
    virtual ~CameraDriverInterface() = default;

    virtual bool startFromConfigFile(const std::string& config_path) = 0;
    virtual bool isRunning() const = 0;
};

std::shared_ptr<CameraDriverInterface> createCameraDriver();

}  // namespace camera_driver
