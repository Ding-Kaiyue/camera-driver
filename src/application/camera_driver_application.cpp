#include "application/camera_driver_application.h"

#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

#include <Eigen/Geometry>
#include <controller_interfaces/srv/query_distance_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "camera/realsense_depth_source.h"
#include "camera_driver/common/types.hpp"
#include "camera_driver/esdf/esdf_map_ros.hpp"
#include "camera_driver/esdf/live_esdf_registry.hpp"
#include "camera_driver/point_cloud/filter.hpp"
#include "camera_driver/point_cloud/point_cloud_build.hpp"
#include "camera_driver/point_cloud/point_cloud_publish.hpp"

namespace camera_driver {
namespace {

std::atomic<bool> g_should_exit{false};
constexpr const char* kEsdfQueryServiceName = "/camera_driver/query_distance_field";

void onSignal(int) {
    g_should_exit.store(true);
}

Transform makeTransformationFromExtrinsics(
    const CameraConfig& camera_config) {
    Eigen::Quaterniond rotation(camera_config.extrinsics.qw,
                                camera_config.extrinsics.qx,
                                camera_config.extrinsics.qy,
                                camera_config.extrinsics.qz);
    rotation.normalize();

    Transform transform = Transform::Identity();
    transform.linear() = rotation.toRotationMatrix();
    transform.translation() = Eigen::Vector3d(camera_config.extrinsics.tx,
                                              camera_config.extrinsics.ty,
                                              camera_config.extrinsics.tz);
    return transform;
}

Transform toTransform(const geometry_msgs::msg::TransformStamped& tf_msg) {
    Eigen::Quaterniond rotation(tf_msg.transform.rotation.w,
                                tf_msg.transform.rotation.x,
                                tf_msg.transform.rotation.y,
                                tf_msg.transform.rotation.z);
    rotation.normalize();

    Transform transform = Transform::Identity();
    transform.linear() = rotation.toRotationMatrix();
    transform.translation() = Eigen::Vector3d(tf_msg.transform.translation.x,
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

class ScopedRosExecutor {
public:
    ScopedRosExecutor() = default;

    ~ScopedRosExecutor() { stop(); }

    void start(const rclcpp::Node::SharedPtr& node) {
        if (!node || running_) {
            return;
        }
        node_ = node;
        executor_.add_node(node_);
        spin_thread_ = std::thread([this]() { executor_.spin(); });
        running_ = true;
    }

    void stop() {
        if (!running_) {
            return;
        }
        executor_.cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        if (node_) {
            executor_.remove_node(node_);
            node_.reset();
        }
        running_ = false;
    }

private:
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Node::SharedPtr node_;
    std::thread spin_thread_;
    bool running_{false};
};

std::size_t cropPointcloudByWorldZ(const Transform& T_world_camera,
                                   double min_z_world_m,
                                   Pointcloud* points_camera) {
    if (points_camera == nullptr || points_camera->empty()) {
        return 0u;
    }

    Pointcloud filtered_points;
    filtered_points.reserve(points_camera->size());
    std::size_t removed_points = 0u;

    for (const Point& point_camera : *points_camera) {
        const Eigen::Vector3d point_world =
            T_world_camera * point_camera.cast<double>();
        if (point_world.z() < min_z_world_m) {
            ++removed_points;
            continue;
        }
        filtered_points.push_back(point_camera);
    }

    *points_camera = std::move(filtered_points);
    return removed_points;
}

}  // namespace

int runCameraDriverApplication(const CameraDriverConfig& app_cfg,
                               const rclcpp::Node::SharedPtr& node) {
    if (!node) {
        std::cerr << "runCameraDriverApplication requires a valid ROS 2 node."
                  << std::endl;
        return 2;
    }

    std::cout << "Loaded camera_driver config set:\n"
              << "  camera: " << app_cfg.camera_config_path << "\n"
              << "  filter: " << app_cfg.filter_config_path << "\n"
              << "  esdf:   " << app_cfg.esdf_config_path << std::endl;
    g_should_exit.store(false);
    if (app_cfg.install_signal_handler) {
        std::signal(SIGINT, onSignal);
    }

    try {
        const Transform T_parent_camera =
            makeTransformationFromExtrinsics(app_cfg.camera);

        std::cout << "Loaded camera extrinsics from " << app_cfg.camera_config_path << "\n"
                  << "  parent_frame = " << app_cfg.camera.stream.parent_frame_id << "\n"
                  << "  world_frame  = " << app_cfg.camera.publish.world_frame_id << "\n"
                  << "  t = [" << app_cfg.camera.extrinsics.tx << ", "
                  << app_cfg.camera.extrinsics.ty << ", " << app_cfg.camera.extrinsics.tz << "]\n"
                  << "  q(x,y,z,w) = [" << app_cfg.camera.extrinsics.qx << ", "
                  << app_cfg.camera.extrinsics.qy << ", " << app_cfg.camera.extrinsics.qz << ", "
                  << app_cfg.camera.extrinsics.qw << "]" << std::endl;

        const bool need_tf_compose =
            app_cfg.camera.stream.parent_frame_id != app_cfg.camera.publish.world_frame_id;
        const bool need_tf_listener = need_tf_compose || app_cfg.filter.enable_self_filter;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener;
        if (need_tf_listener) {
            tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
            tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer, node, false);
            if (need_tf_compose) {
                std::cout << "Composing pose by TF each frame:\n"
                          << "  target(frame): " << app_cfg.camera.publish.world_frame_id << "\n"
                          << "  source(frame): " << app_cfg.camera.stream.parent_frame_id
                          << std::endl;
            }
        }

        auto pointcloud_publisher = std::make_unique<PointCloudPublisher>(app_cfg, node);
        RealSenseDepthSource depth_source(app_cfg);
        PointCloudBuilder pointcloud_builder(
            app_cfg, depth_source.intrinsics(), depth_source.depthScale());

        std::shared_ptr<EsdfMapRos> esdf_grid_map =
            std::make_shared<EsdfMapRos>(app_cfg, node);
        setLiveEsdfMap(esdf_grid_map);

        auto esdf_query_service =
            node->create_service<controller_interfaces::srv::QueryDistanceField>(
                kEsdfQueryServiceName,
                [esdf_grid_map, node](
                    const std::shared_ptr<controller_interfaces::srv::QueryDistanceField::Request> request,
                    std::shared_ptr<controller_interfaces::srv::QueryDistanceField::Response> response) {
                    if (!response) {
                        return;
                    }
                    response->success = false;
                    response->map_ready = static_cast<bool>(esdf_grid_map);
                    response->message = "map_not_ready";
                    if (!esdf_grid_map) {
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
                        const Eigen::Vector3d p_world(
                            position.x,
                            position.y,
                            position.z);
                        const EsdfQueryResult query =
                            esdf_grid_map->queryDistanceAndGradient(p_world);
                        response->observed[i] = query.observed;
                        response->distance_valid[i] = query.distance_valid;
                        response->gradient_valid[i] = query.gradient_valid;
                        response->distances[i] = query.distance;
                        response->gradients[i] = toVector3Msg(query.gradient);
                        if (!query.observed || !query.distance_valid) {
                            RCLCPP_WARN_THROTTLE(
                                node->get_logger(),
                                *node->get_clock(),
                                2000,
                                "camera_driver ESDF invalid query: %s",
                                esdf_grid_map->describeQueryState(p_world).c_str());
                        }
                    }

                    response->success = true;
                    response->map_ready = true;
                    response->message = "ok";
                });
        (void)esdf_query_service;
        RCLCPP_INFO(
            node->get_logger(),
            "camera_driver ESDF query service ready: %s",
            kEsdfQueryServiceName);

        ScopedRosExecutor ros_executor;
        ros_executor.start(node);
        RCLCPP_INFO(
            node->get_logger(),
            "camera_driver ROS executor thread started for TF/service callbacks.");

        std::unique_ptr<RobotSelfFilter> self_filter;
        if (app_cfg.filter.enable_self_filter) {
            std::string self_filter_error;
            self_filter = RobotSelfFilter::create(
                app_cfg, node, tf_buffer.get(), &self_filter_error);
            if (!self_filter) {
                std::cerr << "Robot self-filter init failed: "
                          << self_filter_error << std::endl;
                clearLiveEsdfMap();
                return 1;
            }
        }

        int frame_count = 0;
        std::size_t last_points = 0u;
        int dropped_depth_frames = 0;
        int skipped_no_tf_frames = 0;
        std::size_t self_filtered_mapping_total = 0u;
        std::size_t mapping_z_cropped_total = 0u;
        Transform T_world_camera = T_parent_camera;
        bool has_world_pose = !need_tf_compose;
        std::cout << "[camera_driver] entering main capture loop" << std::endl;

        while (!g_should_exit.load()) {
            if (frame_count < 5) {
                std::cout << "[camera_driver] loop iteration begin"
                          << " frame_count=" << frame_count
                          << " dropped_depth_frames=" << dropped_depth_frames
                          << std::endl;
            }
            if (app_cfg.camera.stream.max_frames > 0 && frame_count >= app_cfg.camera.stream.max_frames) {
                break;
            }

            if (need_tf_compose) {
                try {
                    const geometry_msgs::msg::TransformStamped tf_msg =
                        tf_buffer->lookupTransform(app_cfg.camera.publish.world_frame_id,
                                                   app_cfg.camera.stream.parent_frame_id,
                                                   tf2::TimePointZero);
                    const Transform T_world_parent = toTransform(tf_msg);
                    T_world_camera = T_world_parent * T_parent_camera;
                    has_world_pose = true;
                } catch (const std::exception& e) {
                    ++skipped_no_tf_frames;
                    if (skipped_no_tf_frames % 30 == 0) {
                        std::cout << "[warn] waiting TF "
                                  << app_cfg.camera.publish.world_frame_id << " <- "
                                  << app_cfg.camera.stream.parent_frame_id
                                  << " error: " << e.what() << std::endl;
                    }
                    if (!has_world_pose) {
                        continue;
                    }
                }
            }

            if (frame_count < 5) {
                std::cout << "[camera_driver] waiting for depth frame" << std::endl;
            }
            const rs2::depth_frame depth_frame = depth_source.waitForDepthFrame();
            if (frame_count < 5) {
                std::cout << "[camera_driver] waitForDepthFrame returned"
                          << " valid=" << (depth_frame ? "true" : "false")
                          << std::endl;
            }
            if (!depth_frame) {
                ++dropped_depth_frames;
                if (dropped_depth_frames <= 5 || (dropped_depth_frames % 30) == 0) {
                    std::cout << "[camera_driver] no valid depth frame"
                              << " dropped_depth_frames=" << dropped_depth_frames
                              << " tf_wait_skipped=" << skipped_no_tf_frames
                              << std::endl;
                }
                continue;
            }

            if (frame_count < 5) {
                std::cout << "[camera_driver] building pointcloud" << std::endl;
            }
            Pointcloud points_for_mapping =
                pointcloud_builder.buildForMapping(depth_frame);
            if (frame_count < 5) {
                std::cout << "[camera_driver] buildForMapping done"
                          << " points=" << points_for_mapping.size()
                          << std::endl;
            }
            if (app_cfg.filter.crop_below_world_z_enabled) {
                if (frame_count < 5) {
                    std::cout << "[camera_driver] cropPointcloudByWorldZ begin" << std::endl;
                }
                mapping_z_cropped_total += cropPointcloudByWorldZ(
                    T_world_camera,
                    app_cfg.filter.crop_below_world_z_m,
                    &points_for_mapping);
                if (frame_count < 5) {
                    std::cout << "[camera_driver] cropPointcloudByWorldZ done"
                              << " points=" << points_for_mapping.size()
                              << " total_cropped=" << mapping_z_cropped_total
                              << std::endl;
                }
            }
            if (self_filter && self_filter->enabled()) {
                if (frame_count < 5) {
                    std::cout << "[camera_driver] self_filter begin" << std::endl;
                }
                self_filtered_mapping_total += self_filter->filterPointcloud(
                    T_world_camera,
                    &points_for_mapping);
                if (frame_count < 5) {
                    std::cout << "[camera_driver] self_filter done"
                              << " points=" << points_for_mapping.size()
                              << " total_self_filtered=" << self_filtered_mapping_total
                              << std::endl;
                }
            }

            if (frame_count < 5) {
                std::cout << "[camera_driver] esdf update begin" << std::endl;
            }
            esdf_grid_map->updateFromPointcloud(points_for_mapping, T_world_camera);
            if (frame_count < 5) {
                std::cout << "[camera_driver] esdf update done" << std::endl;
            }

            last_points = points_for_mapping.size();
            if (frame_count < 5) {
                std::cout << "[camera_driver] publish begin"
                          << " last_points=" << last_points
                          << std::endl;
            }
            pointcloud_publisher->publish(points_for_mapping, T_world_camera);
            if (frame_count < 5) {
                std::cout << "[camera_driver] publish done" << std::endl;
            }

            ++frame_count;

            if (frame_count <= 5 || (frame_count % 30) == 0) {
                std::cout << "[camera_driver] heartbeat"
                          << " frames=" << frame_count
                          << " tf_wait_skipped=" << skipped_no_tf_frames
                          << " dropped_depth_frames=" << dropped_depth_frames
                          << " mapping_z_cropped_total=" << mapping_z_cropped_total
                          << " self_filtered_mapping_total=" << self_filtered_mapping_total
                          << " last_points=" << last_points
                          << " esdf_frames=" << esdf_grid_map->processedFrames()
                          << " esdf_occupied_voxels=" << esdf_grid_map->occupiedVoxelCount()
                          << std::endl;
            }
        }

        clearLiveEsdfMap();
        esdf_grid_map.reset();
        depth_source.stop();

        std::cout << "\nDone.\n"
                  << "  frames total:         " << frame_count << "\n"
                  << "  tf-wait skipped:      " << skipped_no_tf_frames << "\n"
                  << "  dropped depth frames: " << dropped_depth_frames << "\n"
                  << "  mapping z-cropped:    " << mapping_z_cropped_total << "\n"
                  << "  self-filtered mapping:" << self_filtered_mapping_total << "\n"
                  << "  last points/frame:    " << last_points << std::endl;

        pointcloud_publisher.reset();
        tf_listener.reset();
        tf_buffer.reset();

        return 0;
    } catch (const rs2::error& e) {
        clearLiveEsdfMap();
        std::cerr << "RealSense error: " << e.what() << "\n"
                  << "  function: " << e.get_failed_function() << "\n"
                  << "  args: " << e.get_failed_args() << std::endl;
        return 3;
    } catch (const std::exception& e) {
        clearLiveEsdfMap();
        std::cerr << "Exception: " << e.what() << std::endl;
        return 4;
    }
}

int runCameraDriverApplication(const CameraDriverConfig& app_cfg) {
    bool ros2_initialized_here = false;
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
        ros2_initialized_here = true;
    }

    const auto node = rclcpp::Node::make_shared("camera_driver");
    const int rc = runCameraDriverApplication(app_cfg, node);

    if (ros2_initialized_here && rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return rc;
}

int runCameraDriverFromConfigFile(const std::string& app_config_path,
                                  const rclcpp::Node::SharedPtr& node) {
    CameraDriverConfig app_cfg;
    if (!loadCameraDriverConfig(app_config_path, &app_cfg)) {
        return 1;
    }
    return runCameraDriverApplication(app_cfg, node);
}

int runCameraDriverFromConfigFile(const std::string& app_config_path) {
    CameraDriverConfig app_cfg;
    if (!loadCameraDriverConfig(app_config_path, &app_cfg)) {
        return 1;
    }
    return runCameraDriverApplication(app_cfg);
}

void requestCameraDriverStop() {
    g_should_exit.store(true);
}

}  // namespace camera_driver
