#include "application/camera_driver_service.h"

#include <iostream>
#include <utility>

#include "application/camera_driver_application.h"

namespace camera_driver {

CameraDriverService::~CameraDriverService() {
    stop();
}

bool CameraDriverService::startFromConfigFile(const std::string& config_path) {
    if (running_.load()) {
        return true;
    }

    CameraDriverConfig app_cfg;
    if (!loadCameraDriverConfig(config_path, &app_cfg)) {
        std::cerr << "CameraDriverService failed to load config: " << config_path
                  << std::endl;
        return false;
    }
    app_cfg.install_signal_handler = false;

    running_.store(true);
    worker_ = std::thread([this, app_cfg]() {
        const int rc = runCameraDriverApplication(app_cfg);
        if (rc != 0) {
            std::cerr << "CameraDriverService exited with code " << rc << std::endl;
        }
        running_.store(false);
    });
    return true;
}

bool CameraDriverService::isRunning() const {
    return running_.load();
}

void CameraDriverService::stop() {
    if (!worker_.joinable()) {
        return;
    }
    requestCameraDriverStop();
    worker_.join();
    running_.store(false);
}

}  // namespace camera_driver
