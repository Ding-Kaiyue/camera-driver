#include "config_internal.h"

#include <cctype>

namespace realsense_voxblox_bridge {

bool parseIntArg(const std::string& value, int* out) {
    if (out == nullptr) {
        return false;
    }
    try {
        *out = std::stoi(value);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseFloatArg(const std::string& value, float* out) {
    if (out == nullptr) {
        return false;
    }
    try {
        *out = std::stof(value);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseDoubleArg(const std::string& value, double* out) {
    if (out == nullptr) {
        return false;
    }
    try {
        *out = std::stod(value);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseBoolArg(const std::string& value, bool* out) {
    if (out == nullptr) {
        return false;
    }
    if (value == "1" || value == "true" || value == "TRUE" || value == "on" ||
        value == "ON") {
        *out = true;
        return true;
    }
    if (value == "0" || value == "false" || value == "FALSE" ||
        value == "off" || value == "OFF") {
        *out = false;
        return true;
    }
    return false;
}

std::string trim(const std::string& input) {
    size_t begin = 0;
    while (begin < input.size() &&
            std::isspace(static_cast<unsigned char>(input[begin])) != 0) {
        ++begin;
    }

    if (begin == input.size()) {
        return "";
    }

    size_t end = input.size();
    while (end > begin &&
            std::isspace(static_cast<unsigned char>(input[end - 1])) != 0) {
        --end;
    }
    return input.substr(begin, end - begin);
}

std::string normalizeKey(const std::string& key) {
    std::string normalized = trim(key);
    for (char& ch : normalized) {
        if (ch == '-') {
            ch = '_';
        } else {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
    }
    return normalized;
}

bool setAppConfigValue(const std::string& key_raw,
                       const std::string& value,
                       AppConfig* config) {
    if (config == nullptr) {
        return false;
    }

    const std::string key = normalizeKey(key_raw);

    if (key == "width") {
        return parseIntArg(value, &config->width);
    }
    if (key == "height") {
        return parseIntArg(value, &config->height);
    }
    if (key == "fps") {
        return parseIntArg(value, &config->fps);
    }
    if (key == "max_frames") {
        return parseIntArg(value, &config->max_frames);
    }
    if (key == "update_esdf_every_n_frames" || key == "update_esdf_every") {
        return parseIntArg(value, &config->update_esdf_every_n_frames);
    }
    if (key == "sample_stride") {
        return parseIntArg(value, &config->sample_stride);
    }

    if (key == "rs_postprocess") {
        return parseBoolArg(value, &config->rs_postprocess);
    }
    if (key == "rs_hole_fill" || key == "rs_hole_filling_mode") {
        return parseIntArg(value, &config->rs_hole_filling_mode);
    }
    if (key == "rs_emitter" || key == "rs_emitter_enabled") {
        return parseIntArg(value, &config->rs_emitter_enabled);
    }
    if (key == "rs_laser_power") {
        return parseFloatArg(value, &config->rs_laser_power);
    }

    if (key == "voxel_size") {
        return parseFloatArg(value, &config->voxel_size_m);
    }
    if (key == "voxels_per_side") {
        return parseIntArg(value, &config->voxels_per_side);
    }

    if (key == "truncation") {
        return parseFloatArg(value, &config->truncation_distance_m);
    }
    if (key == "min_ray") {
        return parseFloatArg(value, &config->min_ray_length_m);
    }
    if (key == "max_ray") {
        return parseFloatArg(value, &config->max_ray_length_m);
    }
    if (key == "pub_min_ray") {
        return parseFloatArg(value, &config->pub_min_ray_length_m);
    }
    if (key == "pub_max_ray") {
        return parseFloatArg(value, &config->pub_max_ray_length_m);
    }

    if (key == "esdf_max") {
        return parseFloatArg(value, &config->esdf_max_distance_m);
    }
    if (key == "esdf_default") {
        return parseFloatArg(value, &config->esdf_default_distance_m);
    }
    if (key == "esdf_min") {
        return parseFloatArg(value, &config->esdf_min_distance_m);
    }

    if (key == "slice_z") {
        return parseDoubleArg(value, &config->slice_z_m);
    }
    if (key == "output_dir") {
        config->output_dir = value;
        return true;
    }
    if (key == "camera_config") {
        config->camera_config_path = value;
        return true;
    }

    if (key == "ros2_pub") {
        return parseBoolArg(value, &config->ros2_publish_pointcloud);
    }
    if (key == "ros2_topic") {
        config->ros2_topic = value;
        return true;
    }
    if (key == "ros2_frame") {
        config->ros2_frame_id = value;
        return true;
    }

    return false;
}

bool validateConfig(const AppConfig& config) {
    if (config.width <= 0 || config.height <= 0 || config.fps <= 0 ||
        config.voxels_per_side <= 0 || config.sample_stride <= 0 ||
        config.update_esdf_every_n_frames <= 0 || config.voxel_size_m <= 0.0f ||
        config.truncation_distance_m <= 0.0f ||
        config.max_ray_length_m <= config.min_ray_length_m ||
        config.pub_min_ray_length_m < 0.0f ||
        (config.pub_max_ray_length_m > 0.0f &&
        config.pub_max_ray_length_m <= config.pub_min_ray_length_m) ||
        config.esdf_max_distance_m <= 0.0f ||
        config.esdf_default_distance_m <= 0.0f ||
        config.esdf_min_distance_m <= 0.0f || config.ros2_topic.empty() ||
        config.ros2_frame_id.empty() || config.output_dir.empty() ||
        config.camera_config_path.empty() ||
        (config.rs_hole_filling_mode < -1 || config.rs_hole_filling_mode > 2) ||
        (config.rs_emitter_enabled < -1 || config.rs_emitter_enabled > 1)) {
        return false;
    }

    return true;
}

}  // namespace realsense_voxblox_bridge
