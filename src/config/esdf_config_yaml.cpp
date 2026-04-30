#include "camera_driver/config/esdf_config.h"

#include <filesystem>
#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

namespace camera_driver {
namespace {

template <typename T>
bool readRequiredScalar(const YAML::Node& node,
                        const char* key,
                        const std::string& context,
                        T* value) {
    if (value == nullptr || !node[key]) {
        std::cerr << "Missing key '" << key << "' in " << context << "\n";
        return false;
    }
    try {
        *value = node[key].as<T>();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Invalid key '" << key << "' in " << context
                  << ": " << e.what() << "\n";
        return false;
    }
}

bool readRequiredVec3(const YAML::Node& node,
                      const char* key,
                      const std::string& context,
                      double* x,
                      double* y,
                      double* z) {
    if (x == nullptr || y == nullptr || z == nullptr || !node[key] ||
        !node[key].IsSequence() || node[key].size() != 3) {
        std::cerr << "Missing or invalid 3-vector '" << key << "' in "
                  << context << "\n";
        return false;
    }
    try {
        *x = node[key][0].as<double>();
        *y = node[key][1].as<double>();
        *z = node[key][2].as<double>();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Invalid 3-vector '" << key << "' in " << context
                  << ": " << e.what() << "\n";
        return false;
    }
}

bool validateConfig(const EsdfConfig& config) {
    return config.map.resolution_m > 0.0 &&
           config.map.map_size_x_m > 0.0 &&
           config.map.map_size_y_m > 0.0 &&
           config.map.map_size_z_m > 0.0 &&
           config.map.local_update_range_x_m > 0.0 &&
           config.map.local_update_range_y_m > 0.0 &&
           config.map.local_update_range_z_m > 0.0 &&
           config.map.local_map_margin_cells >= 0 &&
           config.map.unknown_distance_m > 0.0 &&
           !config.visualization.occupancy_topic.empty() &&
           !config.visualization.inflated_occupancy_topic.empty() &&
           !config.visualization.esdf_slice_topic.empty();
}

}  // namespace

bool loadEsdfConfigYaml(const std::string& file_path, EsdfConfig* config) {
    if (config == nullptr) {
        return false;
    }

    const std::filesystem::path esdf_config_path =
        std::filesystem::absolute(std::filesystem::path(file_path));

    YAML::Node root;
    try {
        root = YAML::LoadFile(esdf_config_path.string());
    } catch (const std::exception& e) {
        std::cerr << "Failed to load ESDF config: " << esdf_config_path.string()
                  << ": " << e.what() << "\n";
        return false;
    }

    const YAML::Node esdf = root["esdf"];
    const YAML::Node visualization = root["visualization"];
    if (!esdf || !visualization) {
        std::cerr << "esdf_param.yaml must contain 'esdf' and 'visualization' sections\n";
        return false;
    }

    EsdfConfig parsed_config;
    parsed_config.file_path = esdf_config_path.string();

    if (!readRequiredScalar(esdf, "resolution_m", file_path, &parsed_config.map.resolution_m) ||
        !readRequiredVec3(esdf, "map_size_xyz_m", file_path, &parsed_config.map.map_size_x_m, &parsed_config.map.map_size_y_m, &parsed_config.map.map_size_z_m) ||
        !readRequiredVec3(esdf, "map_origin_xyz_m", file_path, &parsed_config.map.map_origin_x_m, &parsed_config.map.map_origin_y_m, &parsed_config.map.map_origin_z_m) ||
        !readRequiredVec3(esdf, "local_update_range_xyz_m", file_path, &parsed_config.map.local_update_range_x_m, &parsed_config.map.local_update_range_y_m, &parsed_config.map.local_update_range_z_m) ||
        !readRequiredScalar(esdf, "obstacle_inflation_m", file_path, &parsed_config.map.obstacle_inflation_m) ||
        !readRequiredScalar(esdf, "local_map_margin_cells", file_path, &parsed_config.map.local_map_margin_cells) ||
        !readRequiredScalar(esdf, "local_bound_inflate_m", file_path, &parsed_config.map.local_bound_inflate_m) ||
        !readRequiredScalar(esdf, "unknown_distance_m", file_path, &parsed_config.map.unknown_distance_m) ||
        !readRequiredScalar(visualization, "slice_height_m", file_path, &parsed_config.visualization.slice_height_m) ||
        !readRequiredScalar(visualization, "truncate_height_m", file_path, &parsed_config.visualization.truncate_height_m) ||
        !readRequiredScalar(visualization, "occupancy_topic", file_path, &parsed_config.visualization.occupancy_topic) ||
        !readRequiredScalar(visualization, "inflated_occupancy_topic", file_path, &parsed_config.visualization.inflated_occupancy_topic) ||
        !readRequiredScalar(visualization, "esdf_slice_topic", file_path, &parsed_config.visualization.esdf_slice_topic)) {
        return false;
    }

    if (!validateConfig(parsed_config)) {
        std::cerr << "Invalid ESDF configuration in "
                  << esdf_config_path.string() << "\n";
        return false;
    }

    *config = std::move(parsed_config);
    return true;
}

}  // namespace camera_driver
