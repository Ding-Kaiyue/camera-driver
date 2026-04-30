#pragma once

#include <string>

namespace camera_driver {

struct EsdfMapConfig {
    double resolution_m = 0.05;
    double map_size_x_m = 2.5;
    double map_size_y_m = 2.5;
    double map_size_z_m = 2.0;
    double map_origin_x_m = -1.25;
    double map_origin_y_m = -1.25;
    double map_origin_z_m = 0.0;
    double local_update_range_x_m = 1.5;
    double local_update_range_y_m = 1.5;
    double local_update_range_z_m = 1.5;
    double obstacle_inflation_m = 0.0;
    int local_map_margin_cells = 2;
    double local_bound_inflate_m = 0.10;
    double unknown_distance_m = 10000.0;
};

struct EsdfVisualizationConfig {
    double slice_height_m = 0.30;
    double truncate_height_m = 2.0;
    std::string occupancy_topic{"/camera_driver/esdf_occupancy"};
    std::string inflated_occupancy_topic{"/camera_driver/esdf_inflated_occupancy"};
    std::string esdf_slice_topic{"/camera_driver/esdf_slice"};
};

struct EsdfConfig {
    std::string file_path;
    EsdfMapConfig map;
    EsdfVisualizationConfig visualization;
};

bool loadEsdfConfigYaml(const std::string& file_path, EsdfConfig* config);

}  // namespace camera_driver
