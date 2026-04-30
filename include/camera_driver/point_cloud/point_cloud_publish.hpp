#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "camera_driver/common/types.hpp"
#include "camera_driver/config/camera_driver_config.h"

namespace camera_driver {

class PointCloudPublisher {
public:
    using RgbColors = std::vector<std::uint32_t>;

    PointCloudPublisher(const CameraDriverConfig& config,
                        rclcpp::Node::SharedPtr node);

    bool enabled() const { return enabled_; }
    void publish(const Pointcloud& points_camera,
                 const Transform& T_world_camera,
                 const RgbColors* colors = nullptr);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    CameraDriverConfig config_;
    bool enabled_{false};
};

}  // namespace camera_driver
