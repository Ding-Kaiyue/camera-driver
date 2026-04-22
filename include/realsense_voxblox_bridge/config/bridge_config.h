#ifndef REALSENSE_VOXBLOX_BRIDGE_CONFIG_BRIDGE_CONFIG_H_
#define REALSENSE_VOXBLOX_BRIDGE_CONFIG_BRIDGE_CONFIG_H_

#include <string>

namespace realsense_voxblox_bridge {

struct AppConfig {
    int width = 0;
    int height = 0;
    int fps = 0;

    int max_frames = 0;
    int update_esdf_every_n_frames = 0;
    int sample_stride = 0;

    bool rs_postprocess = false;
    int rs_hole_filling_mode = 0;
    int rs_emitter_enabled = 0;
    float rs_laser_power = 0.0f;

    float voxel_size_m = 0.0f;
    int voxels_per_side = 0;

    float truncation_distance_m = 0.0f;
    float min_ray_length_m = 0.0f;
    float max_ray_length_m = 0.0f;
    float pub_min_ray_length_m = 0.0f;
    float pub_max_ray_length_m = 0.0f;

    float esdf_max_distance_m = 0.0f;
    float esdf_default_distance_m = 0.0f;
    float esdf_min_distance_m = 0.0f;

    double slice_z_m = 0.0;
    std::string output_dir;
    std::string app_config_path;
    std::string camera_config_path;

    bool ros2_publish_pointcloud = false;
    std::string ros2_topic;
    std::string ros2_frame_id;

};

struct CameraExtrinsics {
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
};

bool loadAppConfigYaml(const std::string& file_path, AppConfig* config);
bool parseCameraExtrinsicsYaml(const std::string& file_path,
                               CameraExtrinsics* extrinsics);

}  // namespace realsense_voxblox_bridge

#endif  // REALSENSE_VOXBLOX_BRIDGE_CONFIG_BRIDGE_CONFIG_H_
