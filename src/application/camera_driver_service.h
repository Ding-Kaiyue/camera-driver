#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "camera_driver/application/camera_driver_interface.hpp"

namespace camera_driver {

class CameraDriverService final : public CameraDriverInterface {
public:
    CameraDriverService() = default;
    ~CameraDriverService() override;

    CameraDriverService(const CameraDriverService&) = delete;
    CameraDriverService& operator=(const CameraDriverService&) = delete;

    bool startFromConfigFile(const std::string& config_path) override;
    bool isRunning() const override;

private:
    void stop();

    std::atomic<bool> running_{false};
    std::thread worker_;
};

}  // namespace camera_driver
