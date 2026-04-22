#include "realsense_voxblox_bridge/bridge_app.h"

#include <atomic>
#include <csignal>
#include <iostream>
#include <utility>

#include <librealsense2/rs.hpp>

#include "realsense_voxblox_bridge/realsense_pointcloud_io.h"
#include "realsense_voxblox_bridge/voxblox_mapper.h"

namespace realsense_voxblox_bridge {
namespace {

std::atomic<bool> g_should_exit{false};

void onSignal(int) { g_should_exit.store(true); }

}  // namespace

int runBridge(const AppConfig& app_cfg) {
    std::cout << "Loaded runtime config: " << app_cfg.app_config_path << std::endl;
    std::signal(SIGINT, onSignal);

    try {
        CameraExtrinsics camera_extrinsics;
        if (!parseCameraExtrinsicsYaml(app_cfg.camera_config_path,
                                    &camera_extrinsics)) {
        return 1;
        }

        voxblox::Rotation rotation(
            static_cast<voxblox::FloatingPoint>(camera_extrinsics.qw),
            static_cast<voxblox::FloatingPoint>(camera_extrinsics.qx),
            static_cast<voxblox::FloatingPoint>(camera_extrinsics.qy),
            static_cast<voxblox::FloatingPoint>(camera_extrinsics.qz));
        rotation.normalize();

        const voxblox::Point translation(
            static_cast<voxblox::FloatingPoint>(camera_extrinsics.tx),
            static_cast<voxblox::FloatingPoint>(camera_extrinsics.ty),
            static_cast<voxblox::FloatingPoint>(camera_extrinsics.tz));
        const voxblox::Transformation T_G_C(rotation, translation);

        std::cout << "Loaded T_G_C from " << app_cfg.camera_config_path << "\n"
                << "  t = [" << camera_extrinsics.tx << ", "
                << camera_extrinsics.ty << ", " << camera_extrinsics.tz << "]\n"
                << "  q(x,y,z,w) = [" << camera_extrinsics.qx << ", "
                << camera_extrinsics.qy << ", " << camera_extrinsics.qz << ", "
                << camera_extrinsics.qw << "]" << std::endl;

        RealSensePointcloudIo pointcloud_io(app_cfg);
        VoxbloxMapper mapper(app_cfg);

        int frame_count = 0;
        size_t last_points = 0u;
        int dropped_depth_frames = 0;

        while (!g_should_exit.load()) {
        if (app_cfg.max_frames > 0 && frame_count >= app_cfg.max_frames) {
            break;
        }

        const rs2::depth_frame depth_frame = pointcloud_io.waitForDepthFrame();
        if (!depth_frame) {
            ++dropped_depth_frames;
            continue;
        }

        voxblox::Pointcloud points_for_mapping;
        const bool need_mapping_points =
            mapper.enabled() ||
            (pointcloud_io.publisherEnabled() &&
            pointcloud_io.publishUsesMappingFilter()) ||
            !pointcloud_io.publisherEnabled();
        if (need_mapping_points) {
            points_for_mapping = pointcloud_io.makePointcloudForMapping(depth_frame);
        }

        if (pointcloud_io.publisherEnabled()) {
            if (pointcloud_io.publishUsesMappingFilter()) {
            last_points = points_for_mapping.size();
            pointcloud_io.publish(points_for_mapping, T_G_C);
            } else {
            voxblox::Pointcloud points_for_publish =
                pointcloud_io.makePointcloudForPublish(depth_frame);
            last_points = points_for_publish.size();
            pointcloud_io.publish(points_for_publish, T_G_C);
            }
            pointcloud_io.spinOnce();
        } else {
            last_points = points_for_mapping.size();
        }

        if (mapper.enabled()) {
            if (points_for_mapping.empty()) {
            points_for_mapping = pointcloud_io.makePointcloudForMapping(depth_frame);
            }
            mapper.submit(T_G_C, std::move(points_for_mapping));
        }

        ++frame_count;

        if (frame_count % 30 == 0) {
            std::cout << "frames=" << frame_count
                    << " mapper_processed=" << mapper.processedFrames()
                    << " mapper_integrated=" << mapper.integratedFrames()
                    << " mapper_skipped=" << mapper.skippedFrames()
                    << " mapper_dropped=" << mapper.droppedFrames()
                    << " dropped_depth_frames=" << dropped_depth_frames
                    << " last_points=" << last_points << std::endl;
            }
        }

        pointcloud_io.stop();
        mapper.stop();

        std::cout << "\nDone.\n"
                << "  frames total:         " << frame_count << "\n"
                << "  dropped depth frames: " << dropped_depth_frames << "\n"
                << "  mapper processed:     " << mapper.processedFrames() << "\n"
                << "  mapper integrated:    " << mapper.integratedFrames() << "\n"
                << "  mapper skipped:       " << mapper.skippedFrames() << "\n"
                << "  mapper queue-dropped: " << mapper.droppedFrames() << "\n"
                << "  last points/frame:    " << last_points << std::endl;

        const bool save_ok = mapper.saveOutputs();
        return save_ok ? 0 : 2;
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << "\n"
                << "  function: " << e.get_failed_function() << "\n"
                << "  args: " << e.get_failed_args() << std::endl;
        return 3;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 4;
    }
}

int runBridgeFromConfigFile(const std::string& app_config_path) {
    AppConfig app_cfg;
    if (!loadAppConfigYaml(app_config_path, &app_cfg)) {
        return 1;
    }
    return runBridge(app_cfg);
}

}  // namespace realsense_voxblox_bridge
