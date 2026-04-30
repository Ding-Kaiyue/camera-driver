#include "camera_driver/point_cloud/point_cloud_publish.hpp"

#include <cstring>
#include <stdexcept>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace camera_driver {
namespace {

constexpr std::uint32_t kDefaultRgb = 0x00FFFFFFu;

}  // namespace

PointCloudPublisher::PointCloudPublisher(const CameraDriverConfig& config,
                                         rclcpp::Node::SharedPtr node)
    : node_(std::move(node)), config_(config) {
    if (!node_) {
        throw std::runtime_error(
            "PointCloudPublisher requires a valid ROS 2 node.");
    }

    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.camera.publish.obstacle_pointcloud_topic, qos);
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        config_.camera.publish.camera_world_pose_topic, qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    enabled_ = true;
}

void PointCloudPublisher::publish(
    const Pointcloud& points_camera,
    const Transform& T_world_camera,
    const RgbColors* colors) {
    if (!enabled_ || !node_ || points_camera.empty()) {
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = config_.camera.publish.world_frame_id;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(points_camera.size());
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
    msg.fields[3].name = "rgb";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
    msg.fields[3].count = 1;

    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(static_cast<std::size_t>(msg.row_step));

    for (std::size_t i = 0; i < points_camera.size(); ++i) {
        const Eigen::Vector3d point_world =
            T_world_camera * points_camera[i].cast<double>();
        const float x = static_cast<float>(point_world.x());
        const float y = static_cast<float>(point_world.y());
        const float z = static_cast<float>(point_world.z());
        const std::uint32_t rgb =
            (colors != nullptr && i < colors->size()) ? (*colors)[i] : kDefaultRgb;

        uint8_t* ptr = msg.data.data() + i * msg.point_step;
        std::memcpy(ptr + 0, &x, sizeof(float));
        std::memcpy(ptr + 4, &y, sizeof(float));
        std::memcpy(ptr + 8, &z, sizeof(float));
        std::memcpy(ptr + 12, &rgb, sizeof(rgb));
    }

    pointcloud_pub_->publish(msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg.header;
    const Eigen::Quaterniond q(T_world_camera.rotation());
    pose_msg.pose.position.x = T_world_camera.translation().x();
    pose_msg.pose.position.y = T_world_camera.translation().y();
    pose_msg.pose.position.z = T_world_camera.translation().z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = msg.header;
    tf_msg.child_frame_id = config_.camera.publish.camera_frame_id;
    tf_msg.transform.translation.x = T_world_camera.translation().x();
    tf_msg.transform.translation.y = T_world_camera.translation().y();
    tf_msg.transform.translation.z = T_world_camera.translation().z();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace camera_driver
