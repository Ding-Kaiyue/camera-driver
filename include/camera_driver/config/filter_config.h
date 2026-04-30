#pragma once

#include <string>
#include <vector>

namespace camera_driver {

struct FilterConfig {
    std::string file_path;
    bool enable_self_filter = false;
    std::string hardware_config_path;
    std::vector<std::string> arm_mappings;
    double self_padding_m = 0.01;
    bool exclude_robot_base = false;
    double base_radius_m = 0.18;
    double base_half_height_m = 0.20;
    double base_center_z_m = 0.20;
    bool crop_below_world_z_enabled = false;
    double crop_below_world_z_m = 0.0;
};

bool loadFilterConfigYaml(const std::string& file_path, FilterConfig* config);

}  // namespace camera_driver
