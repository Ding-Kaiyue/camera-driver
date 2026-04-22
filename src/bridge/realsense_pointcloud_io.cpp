#include "realsense_voxblox_bridge/realsense_pointcloud_io.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace realsense_voxblox_bridge {
namespace {

void setOptionIfSupported(const rs2::options& options,
                          rs2_option option,
                          float value,
                          const char* option_name) {
    if (!options.supports(option)) {
        return;
    }

    const rs2::option_range range = options.get_option_range(option);
    const float clamped = std::min(range.max, std::max(range.min, value));
    options.set_option(option, clamped);
    std::cout << "Set " << option_name << " = " << clamped << std::endl;
}

}  // namespace

class RealSensePointcloudIo::RosPublisherImpl {
public:
    explicit RosPublisherImpl(const AppConfig& config) {
        node_ = rclcpp::Node::make_shared("realsense_voxblox_bridge");
        const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            config.ros2_topic, qos);
    }

    void publish(const voxblox::Pointcloud& points_c,
                const voxblox::Transformation& T_G_C,
                const std::string& frame_id) {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = frame_id;
        msg.height = 1;
        msg.width = static_cast<uint32_t>(points_c.size());
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
        msg.data.resize(static_cast<size_t>(msg.row_step));

        for (size_t i = 0; i < points_c.size(); ++i) {
        const voxblox::Point point_g = T_G_C * points_c[i];
        const float x = point_g.x();
        const float y = point_g.y();
        const float z = point_g.z();

        uint8_t* ptr = msg.data.data() + i * msg.point_step;
        std::memcpy(ptr + 0, &x, sizeof(float));
        std::memcpy(ptr + 4, &y, sizeof(float));
        std::memcpy(ptr + 8, &z, sizeof(float));
        }

        publisher_->publish(msg);
    }

    void spinOnce() { rclcpp::spin_some(node_); }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

RealSensePointcloudIo::RealSensePointcloudIo(const AppConfig& config)
    : config_(config) {
    rs2::config rs_cfg;
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, config_.width, config_.height,
                        RS2_FORMAT_Z16, config_.fps);

    profile_ = pipeline_.start(rs_cfg);
    rs2::depth_sensor depth_sensor =
        profile_.get_device().first<rs2::depth_sensor>();
    depth_scale_ = depth_sensor.get_depth_scale();

    rs2::video_stream_profile depth_vsp =
        profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    intrinsics_ = depth_vsp.get_intrinsics();

    configureDepthSensor(&depth_sensor);

    if (config_.rs_hole_filling_mode >= 0) {
        hole_filling_filter_.set_option(
            RS2_OPTION_HOLES_FILL,
            static_cast<float>(config_.rs_hole_filling_mode));
    }

    publish_uses_map_filter_ =
        std::abs(config_.pub_min_ray_length_m - config_.min_ray_length_m) < 1e-6f &&
        std::abs(config_.pub_max_ray_length_m - config_.max_ray_length_m) < 1e-6f;

    if (config_.ros2_publish_pointcloud) {
        if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
        ros2_initialized_here_ = true;
        }
        ros_publisher_ = std::make_unique<RosPublisherImpl>(config_);
        ros2_publish_enabled_ = true;
        std::cout << "ROS2 pointcloud publish enabled:\n"
                << "  topic: " << config_.ros2_topic << "\n"
                << "  frame: " << config_.ros2_frame_id << std::endl;
    }

    std::cout << "RealSense depth started: " << intrinsics_.width << "x"
                << intrinsics_.height << " @" << config_.fps << " fps"
                << std::endl;
    std::cout << "Depth scale: " << depth_scale_ << " m/unit" << std::endl;
}

RealSensePointcloudIo::~RealSensePointcloudIo() { stop(); }

rs2::depth_frame RealSensePointcloudIo::waitForDepthFrame() {
    const rs2::frameset frames = pipeline_.wait_for_frames();
    rs2::frame depth = frames.get_depth_frame();

    if (config_.rs_postprocess && depth) {
        depth = spatial_filter_.process(depth);
        depth = temporal_filter_.process(depth);
        if (config_.rs_hole_filling_mode >= 0) {
        depth = hole_filling_filter_.process(depth);
        }
    }

    return depth.as<rs2::depth_frame>();
}

voxblox::Pointcloud RealSensePointcloudIo::depthToPointcloud(
    const rs2::depth_frame& depth_frame, float min_depth_m,
    float max_depth_m) const {
    voxblox::Pointcloud points_c;

    const int width = intrinsics_.width;
    const int height = intrinsics_.height;
    const uint16_t* depth_data =
        reinterpret_cast<const uint16_t*>(depth_frame.get_data());

    points_c.reserve(static_cast<size_t>(width) * height /
                    static_cast<size_t>(config_.sample_stride *
                                        config_.sample_stride));

    for (int v = 0; v < height; v += config_.sample_stride) {
        for (int u = 0; u < width; u += config_.sample_stride) {
        const size_t idx = static_cast<size_t>(v) * width + u;
        const uint16_t raw = depth_data[idx];
        if (raw == 0u) {
            continue;
        }

        const float z = static_cast<float>(raw) * depth_scale_;
        if (!std::isfinite(z) || z < min_depth_m) {
            continue;
        }
        if (max_depth_m > 0.0f && z > max_depth_m) {
            continue;
        }

        const float x = (static_cast<float>(u) - intrinsics_.ppx) * z /
                        intrinsics_.fx;
        const float y = (static_cast<float>(v) - intrinsics_.ppy) * z /
                        intrinsics_.fy;
        points_c.emplace_back(x, y, z);
        }
    }

    return points_c;
}

voxblox::Pointcloud RealSensePointcloudIo::makePointcloudForMapping(
    const rs2::depth_frame& depth_frame) const {
    return depthToPointcloud(depth_frame, config_.min_ray_length_m,
                           config_.max_ray_length_m);
}

voxblox::Pointcloud RealSensePointcloudIo::makePointcloudForPublish(
    const rs2::depth_frame& depth_frame) const {
    if (publish_uses_map_filter_) {
        return makePointcloudForMapping(depth_frame);
    }
    return depthToPointcloud(depth_frame, config_.pub_min_ray_length_m,
                            config_.pub_max_ray_length_m);
}

void RealSensePointcloudIo::publish(const voxblox::Pointcloud& points_c,
                                    const voxblox::Transformation& T_G_C) {
    if (!ros2_publish_enabled_ || !ros_publisher_ || points_c.empty()) {
        return;
    }
    ros_publisher_->publish(points_c, T_G_C, config_.ros2_frame_id);
}

void RealSensePointcloudIo::spinOnce() {
    if (!ros2_publish_enabled_ || !ros_publisher_) {
        return;
    }
    ros_publisher_->spinOnce();
}

void RealSensePointcloudIo::configureDepthSensor(rs2::depth_sensor* depth_sensor) {
    if (depth_sensor == nullptr) {
        return;
    }

    if (config_.rs_emitter_enabled >= 0) {
        setOptionIfSupported(*depth_sensor, RS2_OPTION_EMITTER_ENABLED,
                            static_cast<float>(config_.rs_emitter_enabled),
                            "emitter_enabled");
    }

    if (config_.rs_laser_power >= 0.0f) {
        setOptionIfSupported(*depth_sensor, RS2_OPTION_LASER_POWER,
                            config_.rs_laser_power, "laser_power");
    }
}

void RealSensePointcloudIo::stop() {
    if (stopped_) {
        return;
    }
    stopped_ = true;

    pipeline_.stop();
    if (ros2_initialized_here_ && rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

}  // namespace realsense_voxblox_bridge
