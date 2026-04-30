#include "camera_driver/config/camera_driver_config.h"

#include <filesystem>
#include <iostream>
#include <utility>

namespace camera_driver {

bool loadCameraDriverConfig(const std::string& esdf_config_path,
                            CameraDriverConfig* config) {
    if (config == nullptr) {
        return false;
    }

    const std::filesystem::path esdf_path =
        std::filesystem::absolute(std::filesystem::path(esdf_config_path));
    const std::filesystem::path config_dir = esdf_path.parent_path();
    const std::filesystem::path camera_path = config_dir / "camera.yaml";
    const std::filesystem::path filter_path = config_dir / "filter.yaml";

    CameraDriverConfig parsed_config;
    parsed_config.esdf_config_path = esdf_path.string();
    parsed_config.camera_config_path = camera_path.string();
    parsed_config.filter_config_path = filter_path.string();

    if (!loadCameraConfigYaml(parsed_config.camera_config_path, &parsed_config.camera) ||
        !loadFilterConfigYaml(parsed_config.filter_config_path, &parsed_config.filter) ||
        !loadEsdfConfigYaml(parsed_config.esdf_config_path, &parsed_config.esdf)) {
        return false;
    }

    *config = std::move(parsed_config);
    return true;
}

}  // namespace camera_driver
