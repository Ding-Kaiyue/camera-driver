#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "camera_driver/common/types.hpp"
#include "camera_driver/config/esdf_config.h"
#include "camera_driver/esdf/esdf_types.hpp"

namespace camera_driver {

struct EsdfVisualizationData {
    std::vector<Eigen::Vector3f> occupancy_points;
    std::vector<Eigen::Vector3f> inflated_occupancy_points;
    std::vector<Eigen::Vector4f> esdf_slice_points;
};

class RemaniGridMap {
public:
    explicit RemaniGridMap(const EsdfConfig& config);

    void updateFromPointcloud(const Pointcloud& points_camera,
                              const Transform& T_world_camera);

    EsdfQueryResult queryDistanceAndGradient(
        const Eigen::Vector3d& position_world) const;
    bool isInsideMap(const Eigen::Vector3d& position_world) const;
    std::string describeQueryState(const Eigen::Vector3d& position_world) const;
    int processedFrames() const { return processed_frames_.load(); }
    std::size_t occupiedVoxelCount() const;
    EsdfVisualizationData buildVisualizationData() const;

private:
    template <typename FGet, typename FSet>
    void fillESDF(FGet f_get_val, FSet f_set_val, int start, int end);

    bool isInMapUnlocked(const Eigen::Vector3d& position_world) const;
    bool isInMapUnlocked(const Eigen::Vector3i& index) const;
    bool isObservedVoxelUnlocked(const Eigen::Vector3i& index) const;
    bool isWithinObservedBoundsUnlocked(const Eigen::Vector3d& position_world) const;
    bool isWithinObservedBoundsUnlocked(const Eigen::Vector3i& index) const;
    void posToIndexUnlocked(const Eigen::Vector3d& position_world,
                            Eigen::Vector3i& index) const;
    void indexToPosUnlocked(const Eigen::Vector3i& index,
                            Eigen::Vector3d& position_world) const;
    int toAddressUnlocked(const Eigen::Vector3i& index) const;
    int toAddressUnlocked(int x, int y, int z) const;
    void boundIndexUnlocked(Eigen::Vector3i& index) const;

    void clearRegionUnlocked(const Eigen::Vector3i& min_index,
                             const Eigen::Vector3i& max_index);
    void clearDistanceRegionUnlocked(const Eigen::Vector3i& min_index,
                                     const Eigen::Vector3i& max_index);
    void clearStaleRegionUnlocked(const Eigen::Vector3i& keep_min,
                                  const Eigen::Vector3i& keep_max);
    int cacheOccupancyObservationUnlocked(const Eigen::Vector3i& index,
                                          int occ,
                                          const Eigen::Vector3i& min_index,
                                          const Eigen::Vector3i& max_index);
    void flushOccupancyUpdatesUnlocked(const Eigen::Vector3i& integration_min,
                                       const Eigen::Vector3i& integration_max);
    void inflateOccupiedUnlocked(const std::vector<Eigen::Vector3i>& occupied_indices,
                                 const Eigen::Vector3i& min_index,
                                 const Eigen::Vector3i& max_index);
    void updateESDF3dUnlocked(const Eigen::Vector3i& min_index,
                              const Eigen::Vector3i& max_index);
    double getDistanceVoxelUnlocked(const Eigen::Vector3i& index) const;
    bool collectSurroundingDistancesUnlocked(const Eigen::Vector3d& position_world,
                                             double values[2][2][2],
                                             Eigen::Vector3d& diff) const;
    EsdfVisualizationData buildVisualizationDataUnlocked() const;

    EsdfConfig config_;
    mutable std::mutex mutex_;

    double resolution_{0.05};
    double resolution_inv_{20.0};
    Eigen::Vector3d map_origin_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d map_size_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d map_min_boundary_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d map_max_boundary_{Eigen::Vector3d::Zero()};
    Eigen::Vector3i map_voxel_num_{Eigen::Vector3i::Zero()};
    Eigen::Vector3d local_update_range_{Eigen::Vector3d::Constant(1.5)};
    int local_map_margin_cells_{2};
    int local_bound_inflate_cells_{0};
    int inflation_steps_{0};

    double p_hit_{0.70};
    double p_miss_{0.35};
    double p_min_{0.12};
    double p_max_{0.97};
    double p_occ_{0.80};
    double prob_hit_log_{0.0};
    double prob_miss_log_{0.0};
    double clamp_min_log_{0.0};
    double clamp_max_log_{0.0};
    double min_occupancy_log_{0.0};
    double unknown_flag_{0.01};
    double unknown_distance_m_{10000.0};

    bool has_observed_bounds_{false};
    Eigen::Vector3i observed_min_index_{Eigen::Vector3i::Zero()};
    Eigen::Vector3i observed_max_index_{Eigen::Vector3i::Zero()};
    bool has_last_update_bounds_{false};
    Eigen::Vector3i last_update_min_index_{Eigen::Vector3i::Zero()};
    Eigen::Vector3i last_update_max_index_{Eigen::Vector3i::Zero()};

    std::vector<double> occupancy_buffer_;
    std::vector<char> occupancy_buffer_inflate_;
    std::vector<char> occupancy_buffer_neg_;
    std::vector<double> tmp_buffer1_;
    std::vector<double> tmp_buffer2_;
    std::vector<double> distance_buffer_;
    std::vector<double> distance_buffer_neg_;
    std::vector<double> distance_buffer_all_;

    std::vector<short> count_hit_;
    std::vector<short> count_hit_and_miss_;
    std::vector<int> flag_traverse_;
    std::vector<int> flag_rayend_;
    std::vector<Eigen::Vector3i> cache_voxels_;
    int raycast_num_{0};

    std::atomic<int> processed_frames_{0};
};

}  // namespace camera_driver
