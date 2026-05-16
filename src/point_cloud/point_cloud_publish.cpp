#include "camera_driver/point_cloud/point_cloud_publish.hpp"

#include <cstring>
#include <cmath>
#include <queue>
#include <set>
#include <stdexcept>
#include <unordered_map>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace camera_driver {
namespace {

constexpr std::uint32_t kDefaultRgb = 0x00FFFFFFu;

struct CellKey {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const CellKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    bool operator<(const CellKey& other) const {
        if (x != other.x) {
            return x < other.x;
        }
        if (y != other.y) {
            return y < other.y;
        }
        return z < other.z;
    }
};

struct CellKeyHash {
    std::size_t operator()(const CellKey& key) const {
        std::size_t seed = std::hash<int>{}(key.x);
        seed ^= std::hash<int>{}(key.y) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        seed ^= std::hash<int>{}(key.z) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        return seed;
    }
};

struct CellAccum {
    Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
    int count{0};
    std::uint32_t rgb{kDefaultRgb};
};

struct VoxelPoint {
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    std::uint32_t rgb{kDefaultRgb};
};

using AccumulatorMap = std::unordered_map<CellKey, CellAccum, CellKeyHash>;
using VoxelMap = std::unordered_map<CellKey, VoxelPoint, CellKeyHash>;

CellKey toCellKey(const Eigen::Vector3d& p, const double voxel_size) {
    return CellKey{
        static_cast<int>(std::floor(p.x() / voxel_size)),
        static_cast<int>(std::floor(p.y() / voxel_size)),
        static_cast<int>(std::floor(p.z() / voxel_size)),
    };
}

int countOccupiedNeighbors(
    const CellKey& key,
    const AccumulatorMap& accumulators,
    const int radius_cells,
    const int min_count) {
    int neighbor_count = 0;
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const CellKey neighbor_key{key.x + dx, key.y + dy, key.z + dz};
                if (accumulators.find(neighbor_key) == accumulators.end()) {
                    continue;
                }
                ++neighbor_count;
                if (neighbor_count >= min_count) {
                    return neighbor_count;
                }
            }
        }
    }
    return neighbor_count;
}

VoxelMap filterSmallClusters(const VoxelMap& voxels, const int min_cluster_cells) {
    if (min_cluster_cells <= 1 ||
        voxels.size() < static_cast<std::size_t>(min_cluster_cells)) {
        return voxels;
    }

    VoxelMap filtered;
    filtered.reserve(voxels.size());
    std::set<CellKey> visited;
    std::vector<CellKey> component;
    std::queue<CellKey> frontier;
    for (const auto& [seed_key, seed_point] : voxels) {
        (void)seed_point;
        if (visited.find(seed_key) != visited.end()) {
            continue;
        }
        component.clear();
        frontier.push(seed_key);
        visited.insert(seed_key);
        while (!frontier.empty()) {
            const CellKey key = frontier.front();
            frontier.pop();
            component.push_back(key);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        const CellKey neighbor_key{key.x + dx, key.y + dy, key.z + dz};
                        if (visited.find(neighbor_key) != visited.end() ||
                            voxels.find(neighbor_key) == voxels.end()) {
                            continue;
                        }
                        visited.insert(neighbor_key);
                        frontier.push(neighbor_key);
                    }
                }
            }
        }
        if (component.size() < static_cast<std::size_t>(min_cluster_cells)) {
            continue;
        }
        for (const auto& key : component) {
            const auto it = voxels.find(key);
            if (it != voxels.end()) {
                filtered.emplace(key, it->second);
            }
        }
    }
    return filtered;
}

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

    const double voxel_size = std::max(1.0e-3, config_.camera.publish.obstacle_voxel_size_m);
    const int isolated_min_neighbor_count =
        std::max(0, config_.camera.publish.obstacle_isolated_min_neighbor_count);
    const int isolated_neighbor_radius_cells =
        std::max(1, config_.camera.publish.obstacle_isolated_neighbor_radius_cells);
    const int min_cluster_cell_count =
        std::max(1, config_.camera.publish.obstacle_min_cluster_cell_count);

    AccumulatorMap accumulators;
    accumulators.reserve(points_camera.size());
    for (std::size_t i = 0; i < points_camera.size(); ++i) {
        const Eigen::Vector3d point_world =
            T_world_camera * points_camera[i].cast<double>();
        if (!point_world.allFinite()) {
            continue;
        }
        const CellKey key = toCellKey(point_world, voxel_size);
        CellAccum& cell = accumulators[key];
        cell.sum += point_world;
        ++cell.count;
        if (colors != nullptr && i < colors->size()) {
            cell.rgb = (*colors)[i];
        }
    }

    VoxelMap voxels;
    voxels.reserve(accumulators.size());
    for (const auto& [key, cell] : accumulators) {
        if (cell.count <= 0) {
            continue;
        }
        if (isolated_min_neighbor_count > 0 &&
            countOccupiedNeighbors(
                key,
                accumulators,
                isolated_neighbor_radius_cells,
                isolated_min_neighbor_count) < isolated_min_neighbor_count) {
            continue;
        }
        VoxelPoint voxel;
        voxel.point = cell.sum / static_cast<double>(cell.count);
        voxel.rgb = cell.rgb;
        voxels.emplace(key, voxel);
    }
    voxels = filterSmallClusters(voxels, min_cluster_cell_count);

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = config_.camera.publish.world_frame_id;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(voxels.size());
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

    std::size_t i = 0;
    for (const auto& [key, voxel] : voxels) {
        (void)key;
        const float x = static_cast<float>(voxel.point.x());
        const float y = static_cast<float>(voxel.point.y());
        const float z = static_cast<float>(voxel.point.z());
        const std::uint32_t rgb = voxel.rgb;

        uint8_t* ptr = msg.data.data() + i * msg.point_step;
        std::memcpy(ptr + 0, &x, sizeof(float));
        std::memcpy(ptr + 4, &y, sizeof(float));
        std::memcpy(ptr + 8, &z, sizeof(float));
        std::memcpy(ptr + 12, &rgb, sizeof(rgb));
        ++i;
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
