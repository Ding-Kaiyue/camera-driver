#include "camera_driver/config/filter_config.h"

#include <filesystem>
#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

namespace camera_driver {
namespace {

template <typename T>
bool readOptionalScalar(const YAML::Node& node, const char* key, T* value) {
    if (value == nullptr || !node[key]) {
        return true;
    }
    try {
        *value = node[key].as<T>();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Invalid key '" << key << "': " << e.what() << "\n";
        return false;
    }
}

bool validateConfig(const FilterConfig& config) {
    return (!config.enable_self_filter || !config.arm_mappings.empty()) &&
           config.self_padding_m >= 0.0 &&
           config.base_radius_m >= 0.0 &&
           config.base_half_height_m >= 0.0;
}

}  // namespace

bool loadFilterConfigYaml(const std::string& file_path, FilterConfig* config) {
    if (config == nullptr) {
        return false;
    }

    const std::filesystem::path filter_config_path =
        std::filesystem::absolute(std::filesystem::path(file_path));

    YAML::Node root;
    try {
        root = YAML::LoadFile(filter_config_path.string());
    } catch (const std::exception& e) {
        std::cerr << "Failed to load filter config: " << filter_config_path.string()
                  << ": " << e.what() << "\n";
        return false;
    }

    const YAML::Node filter = root["filter"];
    if (!filter) {
        std::cerr << "filter.yaml must contain a 'filter' section\n";
        return false;
    }

    FilterConfig parsed_config;
    parsed_config.file_path = filter_config_path.string();

    if (!readOptionalScalar(filter, "enable_self_filter", &parsed_config.enable_self_filter) ||
        !readOptionalScalar(filter, "self_padding_m", &parsed_config.self_padding_m) ||
        !readOptionalScalar(filter, "exclude_robot_base", &parsed_config.exclude_robot_base) ||
        !readOptionalScalar(filter, "base_radius_m", &parsed_config.base_radius_m) ||
        !readOptionalScalar(filter, "base_half_height_m", &parsed_config.base_half_height_m) ||
        !readOptionalScalar(filter, "base_center_z_m", &parsed_config.base_center_z_m) ||
        !readOptionalScalar(filter, "hardware_config", &parsed_config.hardware_config_path)) {
        return false;
    }

    if (filter["arm_mappings"]) {
        if (!filter["arm_mappings"].IsSequence() || filter["arm_mappings"].size() == 0) {
            std::cerr << "filter.arm_mappings must be a non-empty sequence\n";
            return false;
        }
        parsed_config.arm_mappings.clear();
        parsed_config.arm_mappings.reserve(filter["arm_mappings"].size());
        for (std::size_t i = 0; i < filter["arm_mappings"].size(); ++i) {
            parsed_config.arm_mappings.push_back(
                filter["arm_mappings"][i].as<std::string>());
        }
    }

    if (filter["crop_below_world_z_m"]) {
        if (!readOptionalScalar(filter, "crop_below_world_z_m", &parsed_config.crop_below_world_z_m)) {
            return false;
        }
        parsed_config.crop_below_world_z_enabled = true;
    }

    if (!validateConfig(parsed_config)) {
        std::cerr << "Invalid filter configuration in "
                  << filter_config_path.string() << "\n";
        return false;
    }

    *config = std::move(parsed_config);
    return true;
}

}  // namespace camera_driver
