#pragma once

#include <librealsense2/rs.hpp>

#include "camera_driver/common/types.hpp"
#include "camera_driver/config/camera_driver_config.h"

namespace camera_driver {

class PointCloudBuilder {
public:
    PointCloudBuilder(const CameraDriverConfig& config,
                      const rs2_intrinsics& intrinsics,
                      float depth_scale);

    Pointcloud buildForMapping(const rs2::depth_frame& depth_frame) const;

private:
    Pointcloud depthToPointcloud(const rs2::depth_frame& depth_frame,
                                 float min_depth_m,
                                 float max_depth_m) const;

    CameraDriverConfig config_;
    rs2_intrinsics intrinsics_{};
    float depth_scale_{0.001f};
};

}  // namespace camera_driver
