#include "camera_driver/esdf/esdf_map_ros.hpp"

#include <stdexcept>

namespace camera_driver {

EsdfMapRos::EsdfMapRos(const CameraDriverConfig& config,
                       rclcpp::Node::SharedPtr node)
    : config_(config),
      node_(std::move(node)),
      core_(config.esdf) {
    if (!node_) {
        throw std::invalid_argument("EsdfMapRos requires a valid ROS 2 node.");
    }
    publisher_ = std::make_unique<EsdfPublisher>(config_, node_);

    RCLCPP_INFO(
        node_->get_logger(),
        "REMANI ESDF core enabled: origin=(%.3f, %.3f, %.3f) size=(%.3f, %.3f, %.3f) res=%.3f local_range=(%.3f, %.3f, %.3f) inflate=%.3f",
        config_.esdf.map.map_origin_x_m,
        config_.esdf.map.map_origin_y_m,
        config_.esdf.map.map_origin_z_m,
        config_.esdf.map.map_size_x_m,
        config_.esdf.map.map_size_y_m,
        config_.esdf.map.map_size_z_m,
        config_.esdf.map.resolution_m,
        config_.esdf.map.local_update_range_x_m,
        config_.esdf.map.local_update_range_y_m,
        config_.esdf.map.local_update_range_z_m,
        config_.esdf.map.obstacle_inflation_m);
}

void EsdfMapRos::updateFromPointcloud(const Pointcloud& points_camera,
                                      const Transform& T_world_camera) {
    core_.updateFromPointcloud(points_camera, T_world_camera);
    publishVisualization();
}

EsdfQueryResult EsdfMapRos::queryDistanceAndGradient(
    const Eigen::Vector3d& position_world) const {
    return core_.queryDistanceAndGradient(position_world);
}

bool EsdfMapRos::isInsideMap(const Eigen::Vector3d& position_world) const {
    return core_.isInsideMap(position_world);
}

std::string EsdfMapRos::describeQueryState(
    const Eigen::Vector3d& position_world) const {
    return core_.describeQueryState(position_world);
}

int EsdfMapRos::processedFrames() const {
    return core_.processedFrames();
}

std::size_t EsdfMapRos::occupiedVoxelCount() const {
    return core_.occupiedVoxelCount();
}

void EsdfMapRos::publishVisualization() const {
    if (!publisher_) {
        return;
    }

    const EsdfVisualizationData data = core_.buildVisualizationData();
    publisher_->publishOccupancy(data.occupancy_points);
    publisher_->publishInflatedOccupancy(data.inflated_occupancy_points);
    publisher_->publishEsdfSlice(data.esdf_slice_points);
}

}  // namespace camera_driver
