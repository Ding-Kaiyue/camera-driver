#include "camera_driver/point_cloud/point_cloud_build.hpp"

#include <cmath>

namespace camera_driver {

// 相机坐标系，在发布时才乘以T_world_camera转换到世界坐标系
PointCloudBuilder::PointCloudBuilder(
    const CameraDriverConfig& config,
    const rs2_intrinsics& intrinsics,
    float depth_scale)
    : config_(config),
      intrinsics_(intrinsics),
      depth_scale_(depth_scale) {}

Pointcloud PointCloudBuilder::depthToPointcloud(
    const rs2::depth_frame& depth_frame,
    float min_depth_m,
    float max_depth_m) const {
    Pointcloud points_camera;

    const int width = intrinsics_.width;
    const int height = intrinsics_.height;
    const uint16_t* depth_data =
        reinterpret_cast<const uint16_t*>(depth_frame.get_data());

    points_camera.reserve(static_cast<std::size_t>(width) * height /
                          static_cast<std::size_t>(config_.camera.stream.sample_stride *
                                                   config_.camera.stream.sample_stride));

    // pinhole camera model: https://www.intelrealsense.com/depth-camera-pinhole-model/
    for (int v = 0; v < height; v += config_.camera.stream.sample_stride) {
        for (int u = 0; u < width; u += config_.camera.stream.sample_stride) {
            const std::size_t idx = static_cast<std::size_t>(v) * width + u;
            const uint16_t raw = depth_data[idx];
            if (raw == 0u) {
                continue;
            }

            // z = depth_scale * raw
            const float z = static_cast<float>(raw) * depth_scale_;
            if (!std::isfinite(z) || z < min_depth_m) {
                continue;
            }
            if (max_depth_m > 0.0f && z > max_depth_m) {
                continue;
            }

            // x = (u - ppx) * z / fx
            // y = (v - ppy) * z / fy
            const float x = (static_cast<float>(u) - intrinsics_.ppx) * z /
                            intrinsics_.fx;
            const float y = (static_cast<float>(v) - intrinsics_.ppy) * z /
                            intrinsics_.fy;
            points_camera.emplace_back(x, y, z);
        }
    }

    return points_camera;
}

Pointcloud PointCloudBuilder::buildForMapping(
    const rs2::depth_frame& depth_frame) const {
    return depthToPointcloud(
        depth_frame, config_.camera.stream.min_ray_length_m, config_.camera.stream.max_ray_length_m);
}

}  // namespace camera_driver
