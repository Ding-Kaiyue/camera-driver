#ifndef REALSENSE_VOXBLOX_BRIDGE_REALSENSE_POINTCLOUD_IO_H_
#define REALSENSE_VOXBLOX_BRIDGE_REALSENSE_POINTCLOUD_IO_H_

#include <memory>

#include <librealsense2/rs.hpp>

#include "realsense_voxblox_bridge/config/bridge_config.h"
#include "voxblox/core/common.h"

namespace realsense_voxblox_bridge {

class RealSensePointcloudIo {
public:
    explicit RealSensePointcloudIo(const AppConfig& config);
    ~RealSensePointcloudIo();

    rs2::depth_frame waitForDepthFrame();

    voxblox::Pointcloud makePointcloudForMapping(
        const rs2::depth_frame& depth_frame) const;
    voxblox::Pointcloud makePointcloudForPublish(
        const rs2::depth_frame& depth_frame) const;

    bool publisherEnabled() const { return ros2_publish_enabled_; }
    bool publishUsesMappingFilter() const { return publish_uses_map_filter_; }

    void publish(const voxblox::Pointcloud& points_c,
                const voxblox::Transformation& T_G_C);
    void spinOnce();

    const rs2_intrinsics& intrinsics() const { return intrinsics_; }
    float depthScale() const { return depth_scale_; }

    void stop();

private:
    voxblox::Pointcloud depthToPointcloud(const rs2::depth_frame& depth_frame,
                                            float min_depth_m,
                                            float max_depth_m) const;
    void configureDepthSensor(rs2::depth_sensor* depth_sensor);

    AppConfig config_;
    rs2::pipeline pipeline_;
    rs2::pipeline_profile profile_;
    rs2_intrinsics intrinsics_{};
    float depth_scale_ = 0.001f;

    rs2::spatial_filter spatial_filter_;
    rs2::temporal_filter temporal_filter_;
    rs2::hole_filling_filter hole_filling_filter_;

    bool ros2_publish_enabled_ = false;
    bool publish_uses_map_filter_ = false;
    bool ros2_initialized_here_ = false;
    bool stopped_ = false;

    class RosPublisherImpl;
    std::unique_ptr<RosPublisherImpl> ros_publisher_;
};

}  // namespace realsense_voxblox_bridge

#endif  // REALSENSE_VOXBLOX_BRIDGE_REALSENSE_POINTCLOUD_IO_H_
