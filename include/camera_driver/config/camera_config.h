#pragma once

#include <string>

namespace camera_driver {

struct CameraStreamConfig {
    int width = 0;
    int height = 0;
    int fps = 0;
    int max_frames = 0;
    int sample_stride = 0;
    float min_ray_length_m = 0.0f;
    float max_ray_length_m = 0.0f;
    std::string parent_frame_id{"world"};
};

struct RealSenseConfig {
    bool postprocess = false;
    int hole_filling_mode = 0;
    int emitter_enabled = 0;
    float laser_power = 0.0f;
};

struct CameraPublishConfig {
    std::string world_frame_id{"world"};
    std::string obstacle_pointcloud_topic{"/camera_driver/obstacle_pointcloud"};
    std::string camera_world_pose_topic{"/camera_driver/camera_world_pose"};
    std::string camera_frame_id{"camera_link"};
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

struct CameraConfig {
    std::string file_path;
    CameraStreamConfig stream;
    RealSenseConfig realsense;
    CameraPublishConfig publish;
    CameraExtrinsics extrinsics;
};

bool loadCameraConfigYaml(const std::string& file_path, CameraConfig* config);

}  // namespace camera_driver
