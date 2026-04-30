#pragma once

#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "camera_driver/config/camera_driver_config.h"

namespace camera_driver {

class EsdfPublisher {
public:
    EsdfPublisher(const CameraDriverConfig& config,
                  const rclcpp::Node::SharedPtr& node);
    
    // 发布原始占据体素
    void publishOccupancy(const std::vector<Eigen::Vector3f>& points) const;
    // 发布膨胀后的占据体素
    void publishInflatedOccupancy(const std::vector<Eigen::Vector3f>& points) const;
    // 发布ESDF切片，包含距离信息
    void publishEsdfSlice(const std::vector<Eigen::Vector4f>& points) const;

private:
    // 辅助函数，用于发布仅包含XYZ坐标的点云
    void publishXYZCloud(
        const std::vector<Eigen::Vector3f>& points,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const;
    // 辅助函数，用于发布包含XYZ坐标和ESDF数值的点云
    void publishXYZICloud(
        const std::vector<Eigen::Vector4f>& points,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const;

    CameraDriverConfig config_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_inflate_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
};

}  // namespace camera_driver
