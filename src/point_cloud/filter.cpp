#include "camera_driver/point_cloud/filter.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>

namespace camera_driver {
namespace {

std::string joinMappings(const std::vector<std::string>& mappings) {
    std::ostringstream oss;
    for (std::size_t i = 0; i < mappings.size(); ++i) {
        if (i > 0) {
            oss << ',';
        }
        oss << mappings[i];
    }
    return oss.str();
}

Eigen::Vector3d transformPoint(
    const geometry_msgs::msg::TransformStamped& tf_msg,
    const Eigen::Vector3d& point_in_link) {
    Eigen::Quaterniond q(tf_msg.transform.rotation.w,
                         tf_msg.transform.rotation.x,
                         tf_msg.transform.rotation.y,
                         tf_msg.transform.rotation.z);
    if (q.norm() < 1e-8) {
        q = Eigen::Quaterniond::Identity();
    } else {
        q.normalize();
    }
    return q.toRotationMatrix() * point_in_link +
           Eigen::Vector3d(tf_msg.transform.translation.x,
                           tf_msg.transform.translation.y,
                           tf_msg.transform.translation.z);
}

Eigen::Matrix3d transformRotation(
    const geometry_msgs::msg::TransformStamped& tf_msg) {
    Eigen::Quaterniond q(tf_msg.transform.rotation.w,
                         tf_msg.transform.rotation.x,
                         tf_msg.transform.rotation.y,
                         tf_msg.transform.rotation.z);
    if (q.norm() < 1e-8) {
        q = Eigen::Quaterniond::Identity();
    } else {
        q.normalize();
    }
    return q.toRotationMatrix();
}

std::string defaultHardwareConfigPath() {
    return ament_index_cpp::get_package_share_directory("arm_controller") +
           "/config/hardware_config.yaml";
}

}  // namespace

// 创建过滤器
std::unique_ptr<RobotSelfFilter> RobotSelfFilter::create(
    const CameraDriverConfig& config,
    const rclcpp::Node::SharedPtr& node,
    tf2_ros::Buffer* tf_buffer,
    std::string* error) {
    auto filter = std::unique_ptr<RobotSelfFilter>(new RobotSelfFilter());
    filter->node_ = node;
    filter->tf_buffer_ = tf_buffer;
    if (!filter->initialize(config, error)) {
        return nullptr;
    }
    return filter;
}

bool RobotSelfFilter::initialize(const CameraDriverConfig& config,
                                 std::string* error) {
    enabled_ = config.filter.enable_self_filter;
    world_frame_ = config.camera.publish.world_frame_id;
    if (!enabled_) {
        return true;
    }
    if (!node_) {
        if (error != nullptr) {
            *error = "RobotSelfFilter requires a valid ROS node.";
        }
        return false;
    }
    if (tf_buffer_ == nullptr) {
        if (error != nullptr) {
            *error = "RobotSelfFilter requires a valid TF buffer.";
        }
        return false;
    }

    std::string hardware_config_path = config.filter.hardware_config_path;
    if (hardware_config_path.empty()) {
        try {
            hardware_config_path = defaultHardwareConfigPath();
        } catch (const std::exception& e) {
            if (error != nullptr) {
                *error = std::string("Resolve default hardware config failed: ") +
                         e.what();
            }
            return false;
        }
    }

    const std::vector<std::string>& mappings = config.filter.arm_mappings;
    if (mappings.empty()) {
        if (error != nullptr) {
            *error = "RobotSelfFilter has no mappings configured.";
        }
        return false;
    }

    if (!loadEllipsoidsFromHardwareConfig(
            hardware_config_path, mappings, config, error)) {
        return false;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "[PointCloud] Robot self-filter enabled: world_frame=%s mappings=%s ellipsoids=%zu padding=%.4f base_exclusion=%s config=%s",
        world_frame_.c_str(),
        joinMappings(config.filter.arm_mappings).c_str(),
        link_ellipsoids_.size(),
        config.filter.self_padding_m,
        config.filter.exclude_robot_base ? "true" : "false",
        hardware_config_path.c_str());
    return true;
}

// 从硬件配置加载椭球
bool RobotSelfFilter::loadEllipsoidsFromHardwareConfig(
    const std::string& hardware_config_path,
    const std::vector<std::string>& mappings,
    const CameraDriverConfig& config,
    std::string* error) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(hardware_config_path);
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = std::string("Load hardware config failed: ") + e.what();
        }
        return false;
    }

    const YAML::Node hardware = root["hardware"];
    if (!hardware || !hardware.IsMap()) {
        if (error != nullptr) {
            *error = "Missing 'hardware' map in hardware config.";
        }
        return false;
    }

    link_ellipsoids_.clear();
    for (const std::string& mapping : mappings) {
        const YAML::Node mapping_node = hardware[mapping];
        if (!mapping_node || !mapping_node.IsMap()) {
            if (error != nullptr) {
                *error = "Mapping '" + mapping + "' not found in hardware config.";
            }
            return false;
        }

        const YAML::Node collision_spheres = mapping_node["collision_spheres"];
        if (!collision_spheres || !collision_spheres.IsSequence()) {
            if (error != nullptr) {
                *error = "Mapping '" + mapping + "' has no collision_spheres.";
            }
            return false;
        }

        for (std::size_t i = 0; i < collision_spheres.size(); ++i) {
            const YAML::Node item = collision_spheres[i];
            if (!item.IsMap() || !item["link_name"] || !item["center"]) {
                continue;
            }

            const YAML::Node center = item["center"];
            if (!center.IsSequence() || center.size() != 3) {
                continue;
            }

            LinkEllipsoid ellipsoid;
            ellipsoid.link_name = item["link_name"].as<std::string>();
            ellipsoid.center_in_link = Eigen::Vector3d(center[0].as<double>(),
                                                       center[1].as<double>(),
                                                       center[2].as<double>());

            if (item["radii"] && item["radii"].IsSequence() &&
                item["radii"].size() == 3) {
                const YAML::Node radii = item["radii"];
                ellipsoid.radii = Eigen::Vector3d(radii[0].as<double>(),
                                                  radii[1].as<double>(),
                                                  radii[2].as<double>());
            } else if (item["radius"]) {
                ellipsoid.radii =
                    Eigen::Vector3d::Constant(item["radius"].as<double>());
            } else {
                continue;
            }

            ellipsoid.radii =
                ellipsoid.radii.cwiseMax(Eigen::Vector3d::Constant(1e-4)) +
                Eigen::Vector3d::Constant(std::max(0.0, config.filter.self_padding_m));
            const double bounding_radius = ellipsoid.radii.maxCoeff();
            ellipsoid.bounding_radius_sq = bounding_radius * bounding_radius;
            link_ellipsoids_.push_back(std::move(ellipsoid));
        }

        if (config.filter.exclude_robot_base) {
            const YAML::Node frame_id = mapping_node["frame_id"];
            if (!frame_id || !frame_id.IsScalar()) {
                if (error != nullptr) {
                    *error = "Mapping '" + mapping +
                             "' has no frame_id for base exclusion.";
                }
                return false;
            }

            LinkEllipsoid base_ellipsoid;
            base_ellipsoid.link_name = frame_id.as<std::string>();
            base_ellipsoid.center_in_link =
                Eigen::Vector3d(0.0, 0.0, config.filter.base_center_z_m);
            base_ellipsoid.radii = Eigen::Vector3d(
                std::max(1e-4, config.filter.base_radius_m),
                std::max(1e-4, config.filter.base_radius_m),
                std::max(1e-4, config.filter.base_half_height_m));
            const double bounding_radius = base_ellipsoid.radii.maxCoeff();
            base_ellipsoid.bounding_radius_sq = bounding_radius * bounding_radius;
            link_ellipsoids_.push_back(std::move(base_ellipsoid));
        }
    }

    enabled_ = !link_ellipsoids_.empty();
    if (!enabled_ && error != nullptr) {
        *error = "RobotSelfFilter loaded zero ellipsoids.";
    }
    return enabled_;
}

// 构建实际系椭球
bool RobotSelfFilter::buildWorldEllipsoids(
    std::vector<WorldEllipsoid>* ellipsoids) const {
    if (ellipsoids == nullptr || !node_ || tf_buffer_ == nullptr) {
        return false;
    }
    ellipsoids->clear();
    ellipsoids->reserve(link_ellipsoids_.size());

    for (const LinkEllipsoid& link_ellipsoid : link_ellipsoids_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_->lookupTransform(
                world_frame_, link_ellipsoid.link_name, tf2::TimePointZero);
        } catch (const std::exception&) {
            return false;
        }

        WorldEllipsoid ellipsoid;
        ellipsoid.center_world = transformPoint(tf_msg, link_ellipsoid.center_in_link);
        ellipsoid.rotation_world_link = transformRotation(tf_msg);
        ellipsoid.inv_radii_sq = link_ellipsoid.radii.cwiseProduct(link_ellipsoid.radii)
                                     .cwiseInverse();
        ellipsoid.bounding_radius_sq = link_ellipsoid.bounding_radius_sq;
        ellipsoids->push_back(std::move(ellipsoid));
    }

    return true;
}

bool RobotSelfFilter::pointInsideRobot(
    const Eigen::Vector3d& point_world,
    const std::vector<WorldEllipsoid>& ellipsoids) const {
    for (const WorldEllipsoid& ellipsoid : ellipsoids) {
        const Eigen::Vector3d delta_world = point_world - ellipsoid.center_world;
        if (delta_world.squaredNorm() > ellipsoid.bounding_radius_sq) {
            continue;
        }

        const Eigen::Vector3d delta_link =
            ellipsoid.rotation_world_link.transpose() * delta_world;
        const double norm_value =
            delta_link.cwiseProduct(delta_link).dot(ellipsoid.inv_radii_sq);
        if (norm_value <= 1.0) {
            return true;
        }
    }
    return false;
}

std::size_t RobotSelfFilter::filterPointcloud(
    const Transform& T_world_camera,
    Pointcloud* points_camera) const {
    if (!enabled_ || points_camera == nullptr || points_camera->empty()) {
        return 0u;
    }

    std::vector<WorldEllipsoid> ellipsoids;
    if (!buildWorldEllipsoids(&ellipsoids)) {
        return 0u;
    }

    Pointcloud filtered_points;
    filtered_points.reserve(points_camera->size());
    std::size_t removed_points = 0u;

    for (const Point& point_camera : *points_camera) {
        const Eigen::Vector3d point_world =
            T_world_camera * point_camera.cast<double>();
        if (pointInsideRobot(point_world, ellipsoids)) {
            ++removed_points;
            continue;
        }
        filtered_points.push_back(point_camera);
    }

    *points_camera = std::move(filtered_points);
    return removed_points;
}

std::size_t RobotSelfFilter::remapSelfHitsToMaxRange(
    const Transform& T_world_camera,
    float max_range_m,
    Pointcloud* points_camera) const {
    if (!enabled_ || points_camera == nullptr || points_camera->empty() ||
        !(max_range_m > 0.0f)) {
        return 0u;
    }

    std::vector<WorldEllipsoid> ellipsoids;
    if (!buildWorldEllipsoids(&ellipsoids)) {
        return 0u;
    }

    const Eigen::Vector3d camera_world = T_world_camera.translation();
    std::size_t remapped_points = 0u;

    for (Point& point_camera : *points_camera) {
        const Eigen::Vector3d point_world = T_world_camera * point_camera.cast<double>();
        if (!pointInsideRobot(point_world, ellipsoids)) {
            continue;
        }

        Eigen::Vector3d direction_world = point_world - camera_world;
        const double direction_norm = direction_world.norm();
        if (!(direction_norm > 1e-6)) {
            continue;
        }
        direction_world /= direction_norm;
        const Eigen::Vector3d remapped_world =
            camera_world + direction_world * static_cast<double>(max_range_m);
        const Eigen::Vector3d remapped_camera =
            T_world_camera.inverse() * remapped_world;
        point_camera = remapped_camera.cast<float>();
        ++remapped_points;
    }

    return remapped_points;
}

}  // namespace camera_driver
