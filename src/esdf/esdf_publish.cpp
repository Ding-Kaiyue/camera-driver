#include "camera_driver/esdf/esdf_publish.hpp"

#include <cstring>

#include <sensor_msgs/msg/point_field.hpp>

namespace camera_driver {

EsdfPublisher::EsdfPublisher(const CameraDriverConfig& config,
                             const rclcpp::Node::SharedPtr& node)
    : config_(config), node_(node) {
    if (!node_) {
        return;
    }

    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    occupancy_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.esdf.visualization.occupancy_topic, qos);
    occupancy_inflate_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.esdf.visualization.inflated_occupancy_topic, qos);
    esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.esdf.visualization.esdf_slice_topic, qos);
}

void EsdfPublisher::publishOccupancy(
    const std::vector<Eigen::Vector3f>& points) const {
    publishXYZCloud(points, occupancy_pub_);
}

void EsdfPublisher::publishInflatedOccupancy(
    const std::vector<Eigen::Vector3f>& points) const {
    publishXYZCloud(points, occupancy_inflate_pub_);
}

void EsdfPublisher::publishEsdfSlice(
    const std::vector<Eigen::Vector4f>& points) const {
    publishXYZICloud(points, esdf_pub_);
}

void EsdfPublisher::publishXYZCloud(
    const std::vector<Eigen::Vector3f>& points,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const {
    if (!node_ || !publisher) {
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = config_.camera.publish.world_frame_id;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(points.size());
    msg.is_bigendian = false;
    msg.is_dense = false;
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.point_step = 12;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(static_cast<std::size_t>(msg.row_step));

    for (std::size_t i = 0; i < points.size(); ++i) {
        uint8_t* ptr = msg.data.data() + i * msg.point_step;
        const float x = points[i].x();
        const float y = points[i].y();
        const float z = points[i].z();
        std::memcpy(ptr + 0, &x, sizeof(float));
        std::memcpy(ptr + 4, &y, sizeof(float));
        std::memcpy(ptr + 8, &z, sizeof(float));
    }

    publisher->publish(msg);
}

void EsdfPublisher::publishXYZICloud(
    const std::vector<Eigen::Vector4f>& points,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const {
    if (!node_ || !publisher) {
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = config_.camera.publish.world_frame_id;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(points.size());
    msg.is_bigendian = false;
    msg.is_dense = false;
    msg.fields.resize(4);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[3].count = 1;
    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(static_cast<std::size_t>(msg.row_step));

    for (std::size_t i = 0; i < points.size(); ++i) {
        uint8_t* ptr = msg.data.data() + i * msg.point_step;
        const float x = points[i].x();
        const float y = points[i].y();
        const float z = points[i].z();
        const float intensity = points[i].w();  // 在普通激光点云里，它表示反射强度；在ESDF切片里，它表示距离信息
        std::memcpy(ptr + 0, &x, sizeof(float));
        std::memcpy(ptr + 4, &y, sizeof(float));
        std::memcpy(ptr + 8, &z, sizeof(float));
        std::memcpy(ptr + 12, &intensity, sizeof(float));
    }

    publisher->publish(msg);
}

}  // namespace camera_driver
