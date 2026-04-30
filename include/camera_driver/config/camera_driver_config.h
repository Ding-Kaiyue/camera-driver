#pragma once

#include <string>

#include "camera_driver/config/camera_config.h"
#include "camera_driver/config/esdf_config.h"
#include "camera_driver/config/filter_config.h"

namespace camera_driver {

struct CameraDriverConfig {
    std::string camera_config_path;
    std::string filter_config_path;
    std::string esdf_config_path;
    CameraConfig camera;
    FilterConfig filter;
    EsdfConfig esdf;
    bool install_signal_handler = true;
};

bool loadCameraDriverConfig(const std::string& esdf_config_path,
                            CameraDriverConfig* config);

}  // namespace camera_driver
