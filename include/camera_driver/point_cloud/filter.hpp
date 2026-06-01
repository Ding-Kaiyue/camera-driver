#pragma once

#include <cstddef>
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
                                 const rclcpp::Time& frame_stamp,
                                 Pointcloud* points_camera) const;
    std::size_t remapSelfHitsToMaxRange(const Transform& T_world_camera,
                                        const rclcpp::Time& frame_stamp,
                                        float max_range_m,
                                        Pointcloud* points_camera) const;

private:
    struct LinkEllipsoid {
        enum class Shape {
            Ellipsoid,
            Box,
        };

        std::string link_name;
        Eigen::Vector3d center_in_link{Eigen::Vector3d::Zero()};
        Eigen::Vector3d radii{Eigen::Vector3d::Ones()};
        Shape shape{Shape::Ellipsoid};
        double bounding_radius_sq{0.0};
    };

    struct WorldEllipsoid {
        Eigen::Vector3d center_world{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d rotation_world_link{Eigen::Matrix3d::Identity()};
        Eigen::Vector3d inv_radii_sq{Eigen::Vector3d::Ones()};
        Eigen::Vector3d radii{Eigen::Vector3d::Ones()};
        LinkEllipsoid::Shape shape{LinkEllipsoid::Shape::Ellipsoid};
        double bounding_radius_sq{0.0};
    };

    RobotSelfFilter() = default;

    bool initialize(const CameraDriverConfig& config, std::string* error);
    bool loadEllipsoidsFromHardwareConfig(const std::string& hardware_config_path,
                                          const std::vector<std::string>& mappings,
                                          const CameraDriverConfig& config,
                                          std::string* error);
    bool requiredFramesAvailable() const;
    bool buildWorldEllipsoids(const rclcpp::Time& frame_stamp,
                              std::vector<WorldEllipsoid>* ellipsoids) const;
    bool pointInsideRobot(const Eigen::Vector3d& point_world,
                          const std::vector<WorldEllipsoid>& ellipsoids) const;

    bool enabled_{false};
    std::string world_frame_;
    double tf_wait_timeout_sec_{0.02};
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer* tf_buffer_{nullptr};
    std::vector<LinkEllipsoid> link_ellipsoids_;
    mutable bool tf_frames_ready_{false};
    mutable std::size_t tf_frames_not_ready_count_{0};
    mutable std::size_t tf_lookup_latest_fallback_count_{0};
    mutable std::size_t tf_lookup_failure_count_{0};
};

}  // namespace camera_driver
