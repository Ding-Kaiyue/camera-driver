#pragma once

#include <librealsense2/rs.hpp>

#include "camera_driver/config/camera_driver_config.h"

namespace camera_driver {

class RealSenseDepthSource {
public:
    explicit RealSenseDepthSource(const CameraDriverConfig& config);
    ~RealSenseDepthSource();

    rs2::depth_frame waitForDepthFrame();

    const rs2_intrinsics& intrinsics() const { return intrinsics_; }
    float depthScale() const { return depth_scale_; }

    void stop();

private:
    void configureDepthSensor(rs2::depth_sensor* depth_sensor);

    CameraDriverConfig config_;
    rs2::pipeline pipeline_;
    rs2::pipeline_profile profile_;
    rs2_intrinsics intrinsics_{};
    float depth_scale_{0.001f};

    rs2::spatial_filter spatial_filter_;
    rs2::temporal_filter temporal_filter_;
    rs2::hole_filling_filter hole_filling_filter_;

    bool stopped_{false};
};

}  // namespace camera_driver
