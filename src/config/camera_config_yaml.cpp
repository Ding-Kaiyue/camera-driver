#include "camera_driver/config/camera_config.h"

#include <cmath>
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

bool normalizeQuaternion(CameraExtrinsics* extrinsics, const std::string& context) {
    if (extrinsics == nullptr) {
        return false;
    }
    const double q_norm = std::sqrt(extrinsics->qx * extrinsics->qx +
                                    extrinsics->qy * extrinsics->qy +
                                    extrinsics->qz * extrinsics->qz +
                                    extrinsics->qw * extrinsics->qw);
    if (!std::isfinite(q_norm) || q_norm < 1e-12) {
        std::cerr << "Invalid quaternion in " << context << "\n";
        return false;
    }

    extrinsics->qx /= q_norm;
    extrinsics->qy /= q_norm;
    extrinsics->qz /= q_norm;
    extrinsics->qw /= q_norm;
    return true;
}

bool validateConfig(const CameraConfig& config) {
    return config.stream.width > 0 &&
           config.stream.height > 0 &&
           config.stream.fps > 0 &&
           config.stream.sample_stride > 0 &&
           config.stream.min_ray_length_m >= 0.0f &&
           config.stream.max_ray_length_m > config.stream.min_ray_length_m &&
           !config.stream.parent_frame_id.empty() &&
           !config.publish.world_frame_id.empty() &&
           !config.publish.obstacle_pointcloud_topic.empty() &&
           !config.publish.camera_world_pose_topic.empty() &&
           !config.publish.camera_frame_id.empty();
}

}  // namespace

bool loadCameraConfigYaml(const std::string& file_path, CameraConfig* config) {
    if (config == nullptr) {
        return false;
    }

    const std::filesystem::path camera_config_path =
        std::filesystem::absolute(std::filesystem::path(file_path));

    YAML::Node root;
    try {
        root = YAML::LoadFile(camera_config_path.string());
    } catch (const std::exception& e) {
        std::cerr << "Failed to load camera config: " << camera_config_path.string()
                  << ": " << e.what() << "\n";
        return false;
    }

    const YAML::Node camera = root["camera"];
    const YAML::Node realsense = root["realsense"];
    const YAML::Node publish = root["publish"];
    const YAML::Node extrinsics = root["extrinsics"];
    if (!camera || !publish || !extrinsics) {
        std::cerr << "camera.yaml must contain 'camera', 'publish', and 'extrinsics' sections\n";
        return false;
    }

    CameraConfig parsed_config;
    parsed_config.file_path = camera_config_path.string();

    if (!readRequiredScalar(camera, "width", file_path, &parsed_config.stream.width) ||
        !readRequiredScalar(camera, "height", file_path, &parsed_config.stream.height) ||
        !readRequiredScalar(camera, "fps", file_path, &parsed_config.stream.fps) ||
        !readRequiredScalar(camera, "sample_stride", file_path, &parsed_config.stream.sample_stride) ||
        !readRequiredScalar(camera, "min_ray_m", file_path, &parsed_config.stream.min_ray_length_m) ||
        !readRequiredScalar(camera, "max_ray_m", file_path, &parsed_config.stream.max_ray_length_m) ||
        !readRequiredScalar(camera, "parent_frame", file_path, &parsed_config.stream.parent_frame_id) ||
        !readRequiredScalar(publish, "world_frame", file_path, &parsed_config.publish.world_frame_id) ||
        !readRequiredScalar(publish, "obstacle_pointcloud_topic", file_path, &parsed_config.publish.obstacle_pointcloud_topic) ||
        !readRequiredScalar(publish, "camera_world_pose_topic", file_path, &parsed_config.publish.camera_world_pose_topic) ||
        !readRequiredScalar(publish, "camera_frame", file_path, &parsed_config.publish.camera_frame_id)) {
        return false;
    }

    if (!readOptionalScalar(camera, "max_frames", &parsed_config.stream.max_frames) ||
        !readOptionalScalar(realsense, "postprocess", &parsed_config.realsense.postprocess) ||
        !readOptionalScalar(realsense, "hole_fill_mode", &parsed_config.realsense.hole_filling_mode) ||
        !readOptionalScalar(realsense, "laser_power", &parsed_config.realsense.laser_power)) {
        return false;
    }

    if (realsense && realsense["emitter_enabled"]) {
        try {
            parsed_config.realsense.emitter_enabled =
                realsense["emitter_enabled"].as<bool>() ? 1 : 0;
        } catch (...) {
            try {
                parsed_config.realsense.emitter_enabled =
                    realsense["emitter_enabled"].as<int>();
            } catch (const std::exception& e) {
                std::cerr << "Invalid key 'emitter_enabled' in "
                          << camera_config_path.string() << ": " << e.what() << "\n";
                return false;
            }
        }
    }

    const YAML::Node translation = extrinsics["translation_xyz"];
    const YAML::Node quaternion = extrinsics["quaternion_xyzw"];
    if (!translation || !translation.IsSequence() || translation.size() != 3 ||
        !quaternion || !quaternion.IsSequence() || quaternion.size() != 4) {
        std::cerr << "camera.yaml extrinsics must contain translation_xyz[3] and quaternion_xyzw[4]\n";
        return false;
    }

    try {
        parsed_config.extrinsics.tx = translation[0].as<double>();
        parsed_config.extrinsics.ty = translation[1].as<double>();
        parsed_config.extrinsics.tz = translation[2].as<double>();
        parsed_config.extrinsics.qx = quaternion[0].as<double>();
        parsed_config.extrinsics.qy = quaternion[1].as<double>();
        parsed_config.extrinsics.qz = quaternion[2].as<double>();
        parsed_config.extrinsics.qw = quaternion[3].as<double>();
    } catch (const std::exception& e) {
        std::cerr << "Invalid extrinsics values in " << camera_config_path.string()
                  << ": " << e.what() << "\n";
        return false;
    }

    if (!normalizeQuaternion(&parsed_config.extrinsics, camera_config_path.string()) ||
        !validateConfig(parsed_config)) {
        std::cerr << "Invalid camera configuration in "
                  << camera_config_path.string() << "\n";
        return false;
    }

    *config = std::move(parsed_config);
    return true;
}

}  // namespace camera_driver
