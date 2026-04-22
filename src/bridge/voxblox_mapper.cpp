#include "realsense_voxblox_bridge/voxblox_mapper.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <utility>

#include "voxblox/io/layer_io.h"

namespace realsense_voxblox_bridge {
namespace {

constexpr size_t kMaxQueueSize = 2;

}  // namespace

VoxbloxMapper::VoxbloxMapper(const AppConfig& config) : config_(config) {
    voxblox::EsdfPipeline::Config map_cfg;
    map_cfg.tsdf_map_config.tsdf_voxel_size = config_.voxel_size_m;
    map_cfg.tsdf_map_config.tsdf_voxels_per_side =
        static_cast<size_t>(config_.voxels_per_side);
    map_cfg.esdf_map_config.esdf_voxel_size = config_.voxel_size_m;
    map_cfg.esdf_map_config.esdf_voxels_per_side =
        static_cast<size_t>(config_.voxels_per_side);

    map_cfg.tsdf_integrator_type = voxblox::TsdfIntegratorType::kFast;
    map_cfg.tsdf_integrator_config.default_truncation_distance =
        config_.truncation_distance_m;
    map_cfg.tsdf_integrator_config.min_ray_length_m = config_.min_ray_length_m;
    map_cfg.tsdf_integrator_config.max_ray_length_m = config_.max_ray_length_m;

    map_cfg.esdf_integrator_config.max_distance_m = config_.esdf_max_distance_m;
    map_cfg.esdf_integrator_config.default_distance_m =
        config_.esdf_default_distance_m;
    map_cfg.esdf_integrator_config.min_distance_m = config_.esdf_min_distance_m;

    pipeline_ = std::make_unique<voxblox::EsdfPipeline>(map_cfg);
    worker_ = std::thread(&VoxbloxMapper::workerLoop, this);
}

VoxbloxMapper::~VoxbloxMapper() { stop(); }

bool VoxbloxMapper::submit(const voxblox::Transformation& T_G_C,
                           voxblox::Pointcloud points_for_integration) {
    if (!pipeline_ || points_for_integration.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stop_requested_) {
        return false;
    }

    if (queue_.size() >= kMaxQueueSize) {
        queue_.pop_front();
        ++dropped_frames_;
    }

    queue_.push_back(QueueEntry{T_G_C, std::move(points_for_integration)});
    queue_cv_.notify_one();
    return true;
}

bool VoxbloxMapper::isInsideMap(const Eigen::Vector3d& position) const {
    if (!pipeline_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    return pipeline_->isObserved(position);
}

double VoxbloxMapper::getDistance(const Eigen::Vector3d& position) const {
    if (!pipeline_) {
        return -1.0;
    }
    double distance = 0.0;
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    if (!pipeline_->getDistance(position, &distance, true)) {
        return -1.0;
    }
    return distance;
}

Eigen::Vector3d VoxbloxMapper::getGradient(const Eigen::Vector3d& position) const {
    if (!pipeline_) {
        return Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    if (!pipeline_->getGradient(position, &gradient, true)) {
        return Eigen::Vector3d::Zero();
    }
    return gradient;
}

void VoxbloxMapper::updateEsdf() {
    if (!pipeline_) {
        return;
    }
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    pipeline_->updateEsdf();
}

void VoxbloxMapper::workerLoop() {
    if (!pipeline_) {
        return;
    }

    while (true) {
        QueueEntry task;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() { return stop_requested_ || !queue_.empty(); });

            if (stop_requested_ && queue_.empty()) {
                break;
            }

            task = std::move(queue_.front());
            queue_.pop_front();
        }

        bool integrated = false;
        {
            std::lock_guard<std::mutex> lock(pipeline_mutex_);
            integrated = pipeline_->integratePointcloud(task.T_G_C, task.points);
        }
        if (integrated) {
            ++integrated_frames_;
        } else {
            ++skipped_frames_;
        }

        const int processed = ++processed_frames_;
        if (processed % config_.update_esdf_every_n_frames == 0) {
            updateEsdf();
        }
    }
}

void VoxbloxMapper::stop() {
    if (!pipeline_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (stop_requested_) {
            return;
        }
        stop_requested_ = true;
    }
    queue_cv_.notify_all();

    if (worker_.joinable()) {
        worker_.join();
    }

    updateEsdf();
}

bool VoxbloxMapper::saveOutputs() {
    if (!pipeline_) {
        return true;
    }

    stop();

    std::filesystem::create_directories(config_.output_dir);

    const std::string tsdf_path = config_.output_dir + "/tsdf.voxblox";
    const std::string esdf_path = config_.output_dir + "/esdf.voxblox";
    const std::string csv_path = config_.output_dir + "/esdf_slice.csv";

    bool save_tsdf_ok = false;
    bool save_esdf_ok = false;
    bool save_slice_ok = false;
    {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);
        save_tsdf_ok = voxblox::io::SaveLayer(
            pipeline_->getTsdfMapPtr()->getTsdfLayer(), tsdf_path, true);
        save_esdf_ok = voxblox::io::SaveLayer(
            pipeline_->getEsdfMapPtr()->getEsdfLayer(), esdf_path, true);
        save_slice_ok = exportEsdfSliceCsv(*pipeline_->getEsdfMapPtr(),
                                           config_.slice_z_m, csv_path);
    }

    std::cout << "  TSDF file:         " << tsdf_path
                << (save_tsdf_ok ? " [ok]" : " [failed]") << "\n"
                << "  ESDF file:         " << esdf_path
                << (save_esdf_ok ? " [ok]" : " [failed]") << "\n"
                << "  ESDF slice csv:    " << csv_path
                << (save_slice_ok ? " [ok]" : " [failed]") << std::endl;

    return save_tsdf_ok && save_esdf_ok && save_slice_ok;
}

bool VoxbloxMapper::exportEsdfSliceCsv(const voxblox::EsdfMap& esdf_map,
                                       double slice_z_m,
                                       const std::string& csv_path) const {
    std::ofstream out(csv_path);
    if (!out.is_open()) {
        return false;
    }

    out << "x,y,z,distance\n";
    const voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer =
        esdf_map.getEsdfLayerConstPtr();
    if (esdf_layer == nullptr) {
        return false;
    }

    voxblox::BlockIndexList blocks;
    esdf_layer->getAllAllocatedBlocks(&blocks);

    for (const voxblox::BlockIndex& index : blocks) {
        const voxblox::Block<voxblox::EsdfVoxel>& block =
            esdf_layer->getBlockByIndex(index);
        const size_t voxels_per_block =
            block.voxels_per_side() * block.voxels_per_side() *
            block.voxels_per_side();

        for (size_t linear_idx = 0; linear_idx < voxels_per_block; ++linear_idx) {
        const voxblox::Point coord =
            block.computeCoordinatesFromLinearIndex(linear_idx);
        const voxblox::EsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_idx);

        if (!voxel.observed) {
            continue;
        }

        if (std::abs(static_cast<double>(coord.z()) - slice_z_m) >
            static_cast<double>(block.voxel_size())) {
            continue;
        }

        out << coord.x() << ',' << coord.y() << ',' << coord.z() << ','
            << voxel.distance << '\n';
        }
    }

    return true;
}

}  // namespace realsense_voxblox_bridge
