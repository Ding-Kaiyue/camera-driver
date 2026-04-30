#include "camera/realsense_depth_source.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

namespace camera_driver {
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

RealSenseDepthSource::RealSenseDepthSource(const CameraDriverConfig& config)
    : config_(config) {
    rs2::config rs_cfg;
    rs_cfg.enable_stream(RS2_STREAM_DEPTH,
                         config_.camera.stream.width,
                         config_.camera.stream.height,
                         RS2_FORMAT_Z16,
                         config_.camera.stream.fps);

    profile_ = pipeline_.start(rs_cfg);
    rs2::depth_sensor depth_sensor =
        profile_.get_device().first<rs2::depth_sensor>();
    depth_scale_ = depth_sensor.get_depth_scale();

    rs2::video_stream_profile depth_vsp =
        profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    intrinsics_ = depth_vsp.get_intrinsics();

    configureDepthSensor(&depth_sensor);

    if (config_.camera.realsense.hole_filling_mode >= 0) {
        hole_filling_filter_.set_option(
            RS2_OPTION_HOLES_FILL,
            static_cast<float>(config_.camera.realsense.hole_filling_mode));
    }

    std::cout << "RealSense depth started: " << intrinsics_.width << "x"
              << intrinsics_.height << " @" << config_.camera.stream.fps << " fps"
              << std::endl;
    std::cout << "Depth scale: " << depth_scale_ << " m/unit" << std::endl;
}

RealSenseDepthSource::~RealSenseDepthSource() {
    stop();
}

rs2::depth_frame RealSenseDepthSource::waitForDepthFrame() {
    static int consecutive_timeouts = 0;
    rs2::frameset frames;
    const auto start_time = std::chrono::steady_clock::now();
    while (!pipeline_.poll_for_frames(&frames)) {
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed_ms.count() >= 1000) {
            ++consecutive_timeouts;
            if (consecutive_timeouts <= 5 || (consecutive_timeouts % 30) == 0) {
                std::cout << "[camera_driver] waitForDepthFrame timeout after 1000 ms"
                          << " (consecutive_timeouts=" << consecutive_timeouts << ")"
                          << std::endl;
            }
            return rs2::frame().as<rs2::depth_frame>();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (!frames) {
        ++consecutive_timeouts;
        if (consecutive_timeouts <= 5 || (consecutive_timeouts % 30) == 0) {
            std::cout << "[camera_driver] poll_for_frames returned without a valid frameset"
                      << " (consecutive_timeouts=" << consecutive_timeouts << ")"
                      << std::endl;
        }
        return rs2::frame().as<rs2::depth_frame>();
    }

    if (consecutive_timeouts > 0) {
        std::cout << "[camera_driver] depth stream recovered after "
                  << consecutive_timeouts << " timeout(s)" << std::endl;
        consecutive_timeouts = 0;
    }
    rs2::frame depth = frames.get_depth_frame();

    if (config_.camera.realsense.postprocess && depth) {
        depth = spatial_filter_.process(depth);
        depth = temporal_filter_.process(depth);
        if (config_.camera.realsense.hole_filling_mode >= 0) {
            depth = hole_filling_filter_.process(depth);
        }
    }

    return depth.as<rs2::depth_frame>();
}

void RealSenseDepthSource::configureDepthSensor(rs2::depth_sensor* depth_sensor) {
    if (depth_sensor == nullptr) {
        return;
    }

    if (config_.camera.realsense.emitter_enabled >= 0) {
        setOptionIfSupported(*depth_sensor,
                             RS2_OPTION_EMITTER_ENABLED,
                             static_cast<float>(config_.camera.realsense.emitter_enabled),
                             "emitter_enabled");
    }

    if (config_.camera.realsense.laser_power >= 0.0f) {
        setOptionIfSupported(*depth_sensor,
                             RS2_OPTION_LASER_POWER,
                             config_.camera.realsense.laser_power,
                             "laser_power");
    }
}

void RealSenseDepthSource::stop() {
    if (stopped_) {
        return;
    }
    stopped_ = true;
    pipeline_.stop();
}

}  // namespace camera_driver
