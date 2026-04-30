#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "camera_driver/config/camera_driver_config.h"
#include "camera_driver/esdf/esdf_publish.hpp"
#include "camera_driver/esdf/live_esdf_interface.hpp"
#include "camera_driver/esdf/remani_grid_map.hpp"

namespace camera_driver {

class EsdfMapRos final : public LiveEsdfInterface {
public:
    EsdfMapRos(const CameraDriverConfig& config,
               rclcpp::Node::SharedPtr node);

    void updateFromPointcloud(const Pointcloud& points_camera,
                              const Transform& T_world_camera);

    EsdfQueryResult queryDistanceAndGradient(
        const Eigen::Vector3d& position_world) const override;
    bool isInsideMap(const Eigen::Vector3d& position_world) const override;
    std::string describeQueryState(const Eigen::Vector3d& position_world) const;
    int processedFrames() const override;
    std::size_t occupiedVoxelCount() const override;

private:
    void publishVisualization() const;

    CameraDriverConfig config_;
    rclcpp::Node::SharedPtr node_;
    RemaniGridMap core_;
    std::unique_ptr<EsdfPublisher> publisher_;
};

}  // namespace camera_driver
