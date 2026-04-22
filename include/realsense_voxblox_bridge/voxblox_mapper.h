#ifndef REALSENSE_VOXBLOX_BRIDGE_VOXBLOX_MAPPER_H_
#define REALSENSE_VOXBLOX_BRIDGE_VOXBLOX_MAPPER_H_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Core>

#include "realsense_voxblox_bridge/config/bridge_config.h"
#include "voxblox/api/esdf_pipeline.h"
#include "voxblox/core/common.h"

namespace realsense_voxblox_bridge {

class VoxbloxMapper {
public:
    explicit VoxbloxMapper(const AppConfig& config);
    ~VoxbloxMapper();

    bool enabled() const { return static_cast<bool>(pipeline_); }

    bool submit(const voxblox::Transformation& T_G_C,
                voxblox::Pointcloud points_for_integration);
    void stop();
    bool isInsideMap(const Eigen::Vector3d& position) const;
    double getDistance(const Eigen::Vector3d& position) const;
    Eigen::Vector3d getGradient(const Eigen::Vector3d& position) const;

    bool saveOutputs();

    int integratedFrames() const { return integrated_frames_.load(); }
    int skippedFrames() const { return skipped_frames_.load(); }
    int droppedFrames() const { return dropped_frames_.load(); }
    int processedFrames() const { return processed_frames_.load(); }

private:
    struct QueueEntry {
        voxblox::Transformation T_G_C;
        voxblox::Pointcloud points;
    };

    void workerLoop();
    void updateEsdf();
    bool exportEsdfSliceCsv(const voxblox::EsdfMap& esdf_map,
                            double slice_z_m,
                            const std::string& csv_path) const;

    AppConfig config_;
    std::unique_ptr<voxblox::EsdfPipeline> pipeline_;
    std::thread worker_;
    mutable std::mutex pipeline_mutex_;

    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<QueueEntry> queue_;
    bool stop_requested_ = false;

    std::atomic<int> integrated_frames_{0};
    std::atomic<int> skipped_frames_{0};
    std::atomic<int> dropped_frames_{0};
    std::atomic<int> processed_frames_{0};
};

}  // namespace realsense_voxblox_bridge

#endif  // REALSENSE_VOXBLOX_BRIDGE_VOXBLOX_MAPPER_H_
