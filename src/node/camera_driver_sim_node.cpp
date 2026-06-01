#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <controller_interfaces/srv/query_distance_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "camera_driver/common/types.hpp"
#include "camera_driver/config/camera_driver_config.h"
#include "camera_driver/esdf/esdf_map_ros.hpp"
#include "camera_driver/point_cloud/filter.hpp"
#include "camera_driver/point_cloud/point_cloud_publish.hpp"

namespace {

constexpr const char* kEsdfQueryServiceName = "/camera_driver/query_distance_field";

camera_driver::Transform toTransform(
    const geometry_msgs::msg::TransformStamped& tf_msg) {
    Eigen::Quaterniond rotation(tf_msg.transform.rotation.w,
                                tf_msg.transform.rotation.x,
                                tf_msg.transform.rotation.y,
                                tf_msg.transform.rotation.z);
    if (rotation.norm() < 1e-8) {
        rotation = Eigen::Quaterniond::Identity();
    } else {
        rotation.normalize();
    }

    camera_driver::Transform transform = camera_driver::Transform::Identity();
    transform.linear() = rotation.toRotationMatrix();
    transform.translation() =
        Eigen::Vector3d(tf_msg.transform.translation.x,
                        tf_msg.transform.translation.y,
                        tf_msg.transform.translation.z);
    return transform;
}

geometry_msgs::msg::Vector3 toVector3Msg(const Eigen::Vector3d& value) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = value.x();
    msg.y = value.y();
    msg.z = value.z();
    return msg;
}

std::size_t cropPointcloudByWorldZ(
    const camera_driver::Transform& T_world_camera,
    const double min_z_world_m,
    camera_driver::Pointcloud* points_camera) {
    if (points_camera == nullptr || points_camera->empty()) {
        return 0u;
    }

    camera_driver::Pointcloud filtered;
    filtered.reserve(points_camera->size());
    std::size_t removed_points = 0u;
    for (const camera_driver::Point& point_camera : *points_camera) {
        const Eigen::Vector3d point_world =
            T_world_camera * point_camera.cast<double>();
        if (!point_world.allFinite() || point_world.z() < min_z_world_m) {
            ++removed_points;
            continue;
        }
        filtered.push_back(point_camera);
    }
    *points_camera = std::move(filtered);
    return removed_points;
}

class GazeboEsdfBridge final {
public:
    GazeboEsdfBridge(
        const camera_driver::CameraDriverConfig& config,
        rclcpp::Node::SharedPtr node)
        : node_(std::move(node)),
          config_(config),
          tf_buffer_(node_->get_clock()),
          tf_listener_(tf_buffer_) {
        node_->declare_parameter<std::string>(
            "input_pointcloud_topic", "/gazebo_depth_camera/points");
        node_->declare_parameter<std::string>(
            "world_frame", config_.camera.publish.world_frame_id);
        node_->declare_parameter<std::string>(
            "camera_frame", config_.camera.publish.camera_frame_id);
        node_->declare_parameter<double>("min_z_world_m", 0.10);
        node_->declare_parameter<bool>(
            "enable_self_filter", config_.filter.enable_self_filter);

        input_pointcloud_topic_ =
            node_->get_parameter("input_pointcloud_topic").as_string();
        config_.camera.publish.world_frame_id =
            node_->get_parameter("world_frame").as_string();
        config_.camera.publish.camera_frame_id =
            node_->get_parameter("camera_frame").as_string();
        config_.camera.stream.parent_frame_id =
            config_.camera.publish.camera_frame_id;
        config_.filter.crop_below_world_z_enabled = true;
        config_.filter.crop_below_world_z_m =
            node_->get_parameter("min_z_world_m").as_double();
        config_.filter.enable_self_filter =
            node_->get_parameter("enable_self_filter").as_bool();

        esdf_map_ = std::make_shared<camera_driver::EsdfMapRos>(
            config_, node_);
        pointcloud_publisher_ =
            std::make_unique<camera_driver::PointCloudPublisher>(
                config_, node_);

        if (config_.filter.enable_self_filter) {
            std::string error;
            self_filter_ = camera_driver::RobotSelfFilter::create(
                config_, node_, &tf_buffer_, &error);
            if (!self_filter_) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Gazebo ESDF self-filter disabled: %s",
                    error.c_str());
            }
        }

        esdf_query_service_ =
            node_->create_service<controller_interfaces::srv::QueryDistanceField>(
                kEsdfQueryServiceName,
                [this](
                    const std::shared_ptr<
                        controller_interfaces::srv::QueryDistanceField::Request>
                        request,
                    std::shared_ptr<
                        controller_interfaces::srv::QueryDistanceField::Response>
                        response) {
                    handleEsdfQuery(request, response);
                });

        pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_pointcloud_topic_,
            rclcpp::SensorDataQoS().keep_last(2),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                onPointcloud(msg);
            });

        RCLCPP_INFO(
            node_->get_logger(),
            "Gazebo ESDF bridge ready: input=%s output_cloud=%s service=%s world=%s camera_frame=%s",
            input_pointcloud_topic_.c_str(),
            config_.camera.publish.obstacle_pointcloud_topic.c_str(),
            kEsdfQueryServiceName,
            config_.camera.publish.world_frame_id.c_str(),
            config_.camera.publish.camera_frame_id.c_str());
    }

private:
    void handleEsdfQuery(
        const std::shared_ptr<
            controller_interfaces::srv::QueryDistanceField::Request> request,
        std::shared_ptr<
            controller_interfaces::srv::QueryDistanceField::Response> response) {
        if (!response) {
            return;
        }
        response->success = false;
        response->map_ready = static_cast<bool>(esdf_map_);
        response->message = "map_not_ready";
        if (!esdf_map_) {
            return;
        }

        const std::size_t query_count = request ? request->positions.size() : 0u;
        response->observed.resize(query_count, false);
        response->distance_valid.resize(query_count, false);
        response->gradient_valid.resize(query_count, false);
        response->distances.resize(query_count, 0.0);
        response->gradients.resize(query_count);

        for (std::size_t i = 0; i < query_count; ++i) {
            const auto& position = request->positions[i];
            const Eigen::Vector3d p_world(position.x, position.y, position.z);
            const camera_driver::EsdfQueryResult query =
                esdf_map_->queryDistanceAndGradient(p_world);
            response->observed[i] = query.observed;
            response->distance_valid[i] = query.distance_valid;
            response->gradient_valid[i] = query.gradient_valid;
            response->distances[i] = query.distance;
            response->gradients[i] = toVector3Msg(query.gradient);
            if (!query.observed || !query.distance_valid) {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    2000,
                    "Gazebo ESDF invalid query: %s",
                    esdf_map_->describeQueryState(p_world).c_str());
            }
        }

        response->success = true;
        response->map_ready = true;
        response->message = "ok";
    }

    void onPointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!msg || !esdf_map_) {
            return;
        }

        const std::string source_frame = msg->header.frame_id.empty()
                                             ? config_.camera.publish.camera_frame_id
                                             : msg->header.frame_id;
        camera_driver::Transform T_world_camera =
            camera_driver::Transform::Identity();
        try {
            const geometry_msgs::msg::TransformStamped tf_msg =
                tf_buffer_.lookupTransform(
                    config_.camera.publish.world_frame_id,
                    source_frame,
                    tf2::TimePointZero,
                    std::chrono::milliseconds(50));
            T_world_camera = toTransform(tf_msg);
        } catch (const std::exception& e) {
            ++tf_failure_count_;
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                2000,
                "Waiting for TF %s <- %s: %s",
                config_.camera.publish.world_frame_id.c_str(),
                source_frame.c_str(),
                e.what());
            return;
        }

        camera_driver::Pointcloud points_camera;
        points_camera.reserve(
            static_cast<std::size_t>(msg->width) *
            static_cast<std::size_t>(msg->height));

        try {
            sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
            for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
                const Eigen::Vector3f p(*it_x, *it_y, *it_z);
                if (!p.allFinite()) {
                    continue;
                }
                const float range = p.norm();
                if (range < config_.camera.stream.min_ray_length_m ||
                    range > config_.camera.stream.max_ray_length_m) {
                    continue;
                }
                points_camera.emplace_back(p);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                2000,
                "Failed to parse Gazebo pointcloud: %s",
                e.what());
            return;
        }

        std::size_t cropped = 0u;
        if (config_.filter.crop_below_world_z_enabled) {
            cropped = cropPointcloudByWorldZ(
                T_world_camera,
                config_.filter.crop_below_world_z_m,
                &points_camera);
        }

        std::size_t self_filtered = 0u;
        if (self_filter_ && self_filter_->enabled()) {
            self_filtered = self_filter_->filterPointcloud(
                T_world_camera,
                rclcpp::Time(msg->header.stamp),
                &points_camera);
        }

        if (points_camera.empty()) {
            return;
        }

        esdf_map_->updateFromPointcloud(points_camera, T_world_camera);
        pointcloud_publisher_->publish(points_camera, T_world_camera);

        ++frame_count_;
        if (frame_count_ <= 5 || (frame_count_ % 30) == 0) {
            RCLCPP_INFO(
                node_->get_logger(),
                "Gazebo ESDF frame=%d points=%zu cropped=%zu self_filtered=%zu occupied=%zu tf_failures=%zu",
                frame_count_,
                points_camera.size(),
                cropped,
                self_filtered,
                esdf_map_->occupiedVoxelCount(),
                tf_failure_count_);
        }
    }

    rclcpp::Node::SharedPtr node_;
    camera_driver::CameraDriverConfig config_;
    std::string input_pointcloud_topic_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<camera_driver::EsdfMapRos> esdf_map_;
    std::unique_ptr<camera_driver::PointCloudPublisher> pointcloud_publisher_;
    std::unique_ptr<camera_driver::RobotSelfFilter> self_filter_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Service<controller_interfaces::srv::QueryDistanceField>::SharedPtr
        esdf_query_service_;
    int frame_count_{0};
    std::size_t tf_failure_count_{0};
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::string config_path;
    if (argc >= 2) {
        config_path = argv[1];
    } else {
        config_path =
            ament_index_cpp::get_package_share_directory("camera_driver") +
            "/config/esdf_param.yaml";
    }

    camera_driver::CameraDriverConfig config;
    if (!camera_driver::loadCameraDriverConfig(config_path, &config)) {
        std::cerr << "camera_driver_sim failed to load config: "
                  << config_path << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<rclcpp::Node>("camera_driver_sim");
    auto bridge = std::make_shared<GazeboEsdfBridge>(config, node);
    (void)bridge;
    rclcpp::spin(node);

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
