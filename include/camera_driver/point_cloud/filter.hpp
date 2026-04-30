#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "camera_driver/common/types.hpp"
#include "camera_driver/config/camera_driver_config.h"

namespace camera_driver {

class RobotSelfFilter {
public:
    static std::unique_ptr<RobotSelfFilter> create(
        const CameraDriverConfig& config,
        const rclcpp::Node::SharedPtr& node,
        tf2_ros::Buffer* tf_buffer,
        std::string* error);

    bool enabled() const { return enabled_; }

    std::size_t filterPointcloud(const Transform& T_world_camera,
                                 Pointcloud* points_camera) const;
    std::size_t remapSelfHitsToMaxRange(const Transform& T_world_camera,
                                        float max_range_m,
                                        Pointcloud* points_camera) const;

private:
    struct LinkEllipsoid {
        std::string link_name;
        Eigen::Vector3d center_in_link{Eigen::Vector3d::Zero()};
        Eigen::Vector3d radii{Eigen::Vector3d::Ones()};
        double bounding_radius_sq{0.0};
    };

    struct WorldEllipsoid {
        Eigen::Vector3d center_world{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d rotation_world_link{Eigen::Matrix3d::Identity()};
        Eigen::Vector3d inv_radii_sq{Eigen::Vector3d::Ones()};
        double bounding_radius_sq{0.0};
    };

    RobotSelfFilter() = default;

    bool initialize(const CameraDriverConfig& config, std::string* error);
    bool loadEllipsoidsFromHardwareConfig(const std::string& hardware_config_path,
                                          const std::vector<std::string>& mappings,
                                          const CameraDriverConfig& config,
                                          std::string* error);
    bool buildWorldEllipsoids(std::vector<WorldEllipsoid>* ellipsoids) const;
    bool pointInsideRobot(const Eigen::Vector3d& point_world,
                          const std::vector<WorldEllipsoid>& ellipsoids) const;

    bool enabled_{false};
    std::string world_frame_;
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer* tf_buffer_{nullptr};
    std::vector<LinkEllipsoid> link_ellipsoids_;
};

}  // namespace camera_driver
