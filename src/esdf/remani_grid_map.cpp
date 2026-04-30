#include "camera_driver/esdf/remani_grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace camera_driver {
namespace {

double logit(double p) {
    return std::log(p / (1.0 - p));
}

double intbound(double s, double ds) {
    if (ds < 0.0) {
        return intbound(-s, -ds);
    }
    const double s_mod = std::fmod(s, 1.0);
    const double normalized = (s_mod < 0.0) ? (s_mod + 1.0) : s_mod;
    return (1.0 - normalized) / ds;
}

class GridRayCaster {
public:
    bool setInput(const Eigen::Vector3d& start_scaled,
                  const Eigen::Vector3d& end_scaled) {
        start_ = start_scaled;
        end_ = end_scaled;

        x_ = static_cast<int>(std::floor(start_.x()));
        y_ = static_cast<int>(std::floor(start_.y()));
        z_ = static_cast<int>(std::floor(start_.z()));
        end_x_ = static_cast<int>(std::floor(end_.x()));
        end_y_ = static_cast<int>(std::floor(end_.y()));
        end_z_ = static_cast<int>(std::floor(end_.z()));

        const Eigen::Vector3d diff = end_ - start_;
        step_x_ = (diff.x() > 0.0) ? 1 : (diff.x() < 0.0 ? -1 : 0);
        step_y_ = (diff.y() > 0.0) ? 1 : (diff.y() < 0.0 ? -1 : 0);
        step_z_ = (diff.z() > 0.0) ? 1 : (diff.z() < 0.0 ? -1 : 0);

        const double inf = std::numeric_limits<double>::infinity();
        t_delta_x_ = (step_x_ == 0) ? inf : std::abs(1.0 / diff.x());
        t_delta_y_ = (step_y_ == 0) ? inf : std::abs(1.0 / diff.y());
        t_delta_z_ = (step_z_ == 0) ? inf : std::abs(1.0 / diff.z());

        t_max_x_ = (step_x_ == 0) ? inf : intbound(start_.x(), diff.x());
        t_max_y_ = (step_y_ == 0) ? inf : intbound(start_.y(), diff.y());
        t_max_z_ = (step_z_ == 0) ? inf : intbound(start_.z(), diff.z());

        valid_ = (x_ != end_x_) || (y_ != end_y_) || (z_ != end_z_);
        return valid_;
    }

    bool step(Eigen::Vector3i* voxel) {
        if (!valid_ || voxel == nullptr) {
            return false;
        }

        if (t_max_x_ < t_max_y_) {
            if (t_max_x_ < t_max_z_) {
                x_ += step_x_;
                t_max_x_ += t_delta_x_;
            } else {
                z_ += step_z_;
                t_max_z_ += t_delta_z_;
            }
        } else {
            if (t_max_y_ < t_max_z_) {
                y_ += step_y_;
                t_max_y_ += t_delta_y_;
            } else {
                z_ += step_z_;
                t_max_z_ += t_delta_z_;
            }
        }

        if (x_ == end_x_ && y_ == end_y_ && z_ == end_z_) {
            return false;
        }

        *voxel = Eigen::Vector3i(x_, y_, z_);
        return true;
    }

private:
    Eigen::Vector3d start_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d end_{Eigen::Vector3d::Zero()};
    int x_{0};
    int y_{0};
    int z_{0};
    int end_x_{0};
    int end_y_{0};
    int end_z_{0};
    int step_x_{0};
    int step_y_{0};
    int step_z_{0};
    double t_max_x_{0.0};
    double t_max_y_{0.0};
    double t_max_z_{0.0};
    double t_delta_x_{0.0};
    double t_delta_y_{0.0};
    double t_delta_z_{0.0};
    bool valid_{false};
};

Eigen::Vector3d clipPointToMapBoundary(const Eigen::Vector3d& point_world,
                                       const Eigen::Vector3d& camera_world,
                                       const Eigen::Vector3d& map_min,
                                       const Eigen::Vector3d& map_max) {
    const Eigen::Vector3d diff = point_world - camera_world;
    double min_t = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 3; ++i) {
        if (std::abs(diff(i)) < 1e-12) {
            continue;
        }

        const double t1 = (map_max(i) - camera_world(i)) / diff(i);
        if (t1 > 0.0 && t1 < min_t) {
            min_t = t1;
        }

        const double t2 = (map_min(i) - camera_world(i)) / diff(i);
        if (t2 > 0.0 && t2 < min_t) {
            min_t = t2;
        }
    }

    if (!std::isfinite(min_t)) {
        return camera_world;
    }
    return camera_world + (min_t - 1e-3) * diff;
}

bool indexWithinBounds(const Eigen::Vector3i& index,
                       const Eigen::Vector3i& min_index,
                       const Eigen::Vector3i& max_index) {
    return index.x() >= min_index.x() && index.y() >= min_index.y() &&
           index.z() >= min_index.z() && index.x() <= max_index.x() &&
           index.y() <= max_index.y() && index.z() <= max_index.z();
}

}  // namespace

RemaniGridMap::RemaniGridMap(const EsdfConfig& config)
    : config_(config) {
    resolution_ = std::max(1e-3, config_.map.resolution_m);
    resolution_inv_ = 1.0 / resolution_;
    map_size_ = Eigen::Vector3d(
        std::max(resolution_, config_.map.map_size_x_m),
        std::max(resolution_, config_.map.map_size_y_m),
        std::max(resolution_, config_.map.map_size_z_m));
    map_origin_ = Eigen::Vector3d(config_.map.map_origin_x_m,
                                  config_.map.map_origin_y_m,
                                  config_.map.map_origin_z_m);
    map_min_boundary_ = map_origin_;
    map_max_boundary_ = map_origin_ + map_size_;
    local_update_range_ = Eigen::Vector3d(
        std::max(resolution_, config_.map.local_update_range_x_m),
        std::max(resolution_, config_.map.local_update_range_y_m),
        std::max(resolution_, config_.map.local_update_range_z_m));
    local_map_margin_cells_ = std::max(0, config_.map.local_map_margin_cells);
    local_bound_inflate_cells_ = static_cast<int>(
        std::ceil(std::max(0.0, config_.map.local_bound_inflate_m) / resolution_));
    inflation_steps_ = static_cast<int>(
        std::ceil(std::max(0.0, config_.map.obstacle_inflation_m) / resolution_));
    unknown_distance_m_ = std::max(1.0, config_.map.unknown_distance_m);

    prob_hit_log_ = logit(p_hit_);
    prob_miss_log_ = logit(p_miss_);
    clamp_min_log_ = logit(p_min_);
    clamp_max_log_ = logit(p_max_);
    min_occupancy_log_ = logit(p_occ_);

    for (int i = 0; i < 3; ++i) {
        map_voxel_num_(i) =
            std::max(1, static_cast<int>(std::ceil(map_size_(i) / resolution_)));
    }

    const int buffer_size =
        map_voxel_num_.x() * map_voxel_num_.y() * map_voxel_num_.z();
    occupancy_buffer_.assign(buffer_size, clamp_min_log_ - unknown_flag_);
    occupancy_buffer_inflate_.assign(buffer_size, 0);
    occupancy_buffer_neg_.assign(buffer_size, 0);
    tmp_buffer1_.assign(buffer_size, 0.0);
    tmp_buffer2_.assign(buffer_size, 0.0);
    distance_buffer_.assign(buffer_size, unknown_distance_m_);
    distance_buffer_neg_.assign(buffer_size, unknown_distance_m_);
    distance_buffer_all_.assign(buffer_size, unknown_distance_m_);
    count_hit_.assign(buffer_size, 0);
    count_hit_and_miss_.assign(buffer_size, 0);
    flag_traverse_.assign(buffer_size, -1);
    flag_rayend_.assign(buffer_size, -1);
    cache_voxels_.reserve(32768);
}

std::size_t RemaniGridMap::occupiedVoxelCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<std::size_t>(std::count(
        occupancy_buffer_inflate_.begin(),
        occupancy_buffer_inflate_.end(),
        static_cast<char>(1)));
}

bool RemaniGridMap::isInMapUnlocked(const Eigen::Vector3d& position_world) const {
    return position_world.x() >= map_min_boundary_.x() + 1e-4 &&
           position_world.y() >= map_min_boundary_.y() + 1e-4 &&
           position_world.z() >= map_min_boundary_.z() + 1e-4 &&
           position_world.x() <= map_max_boundary_.x() - 1e-4 &&
           position_world.y() <= map_max_boundary_.y() - 1e-4 &&
           position_world.z() <= map_max_boundary_.z() - 1e-4;
}

bool RemaniGridMap::isInMapUnlocked(const Eigen::Vector3i& index) const {
    return index.x() >= 0 && index.y() >= 0 && index.z() >= 0 &&
           index.x() < map_voxel_num_.x() &&
           index.y() < map_voxel_num_.y() &&
           index.z() < map_voxel_num_.z();
}

bool RemaniGridMap::isObservedVoxelUnlocked(const Eigen::Vector3i& index) const {
    if (!isInMapUnlocked(index)) {
        return false;
    }
    return occupancy_buffer_[toAddressUnlocked(index)] >= clamp_min_log_;
}

bool RemaniGridMap::isWithinObservedBoundsUnlocked(
    const Eigen::Vector3d& position_world) const {
    if (!has_observed_bounds_) {
        return false;
    }
    Eigen::Vector3i index;
    posToIndexUnlocked(position_world, index);
    return isWithinObservedBoundsUnlocked(index);
}

bool RemaniGridMap::isWithinObservedBoundsUnlocked(
    const Eigen::Vector3i& index) const {
    if (!has_observed_bounds_) {
        return false;
    }
    return indexWithinBounds(index, observed_min_index_, observed_max_index_);
}

bool RemaniGridMap::isInsideMap(const Eigen::Vector3d& position_world) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isInMapUnlocked(position_world) ||
        !isWithinObservedBoundsUnlocked(position_world)) {
        return false;
    }
    double values[2][2][2];
    Eigen::Vector3d diff = Eigen::Vector3d::Zero();
    return collectSurroundingDistancesUnlocked(position_world, values, diff);
}

std::string RemaniGridMap::describeQueryState(
    const Eigen::Vector3d& position_world) const {
    std::lock_guard<std::mutex> lock(mutex_);

    auto fmtVec3i = [](const Eigen::Vector3i& v) {
        std::ostringstream oss;
        oss << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
        return oss.str();
    };
    auto fmtVec3d = [](const Eigen::Vector3d& v) {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(4);
        oss << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
        return oss.str();
    };

    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(4);
    oss << "p=" << fmtVec3d(position_world)
        << " processed_frames=" << processed_frames_.load();

    if (!isInMapUnlocked(position_world)) {
        oss << " reason=outside_map"
            << " map_min=" << fmtVec3d(map_min_boundary_)
            << " map_max=" << fmtVec3d(map_max_boundary_);
        return oss.str();
    }

    Eigen::Vector3i point_index;
    posToIndexUnlocked(position_world, point_index);
    oss << " point_idx=" << fmtVec3i(point_index);

    if (!has_observed_bounds_) {
        oss << " reason=no_observed_bounds";
        return oss.str();
    }

    oss << " observed_min=" << fmtVec3i(observed_min_index_)
        << " observed_max=" << fmtVec3i(observed_max_index_);

    if (!isWithinObservedBoundsUnlocked(point_index)) {
        oss << " reason=outside_observed_bounds";
        return oss.str();
    }

    const Eigen::Vector3d shifted =
        position_world - 0.5 * resolution_ * Eigen::Vector3d::Ones();
    Eigen::Vector3i base_index;
    posToIndexUnlocked(shifted, base_index);
    oss << " interp_base=" << fmtVec3i(base_index);

    if (base_index.x() < 0 || base_index.y() < 0 || base_index.z() < 0 ||
        base_index.x() >= map_voxel_num_.x() - 1 ||
        base_index.y() >= map_voxel_num_.y() - 1 ||
        base_index.z() >= map_voxel_num_.z() - 1) {
        oss << " reason=interpolation_base_outside_map";
        return oss.str();
    }

    for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 2; ++y) {
            for (int z = 0; z < 2; ++z) {
                const Eigen::Vector3i current =
                    base_index + Eigen::Vector3i(x, y, z);
                if (!isWithinObservedBoundsUnlocked(current)) {
                    oss << " reason=interpolation_neighbor_outside_observed_bounds"
                        << " neighbor_idx=" << fmtVec3i(current);
                    return oss.str();
                }
                const double distance =
                    distance_buffer_all_[toAddressUnlocked(current)];
                if (!std::isfinite(distance)) {
                    oss << " reason=interpolation_neighbor_distance_not_finite"
                        << " neighbor_idx=" << fmtVec3i(current)
                        << " distance=" << distance;
                    return oss.str();
                }
            }
        }
    }

    double values[2][2][2];
    Eigen::Vector3d diff = Eigen::Vector3d::Zero();
    if (!collectSurroundingDistancesUnlocked(position_world, values, diff)) {
        oss << " reason=collect_surrounding_distances_failed";
        return oss.str();
    }

    oss << " reason=ok";
    return oss.str();
}

void RemaniGridMap::posToIndexUnlocked(const Eigen::Vector3d& position_world,
                                       Eigen::Vector3i& index) const {
    for (int i = 0; i < 3; ++i) {
        index(i) = static_cast<int>(std::floor(
            (position_world(i) - map_origin_(i)) * resolution_inv_));
    }
}

void RemaniGridMap::indexToPosUnlocked(const Eigen::Vector3i& index,
                                       Eigen::Vector3d& position_world) const {
    for (int i = 0; i < 3; ++i) {
        position_world(i) =
            (static_cast<double>(index(i)) + 0.5) * resolution_ + map_origin_(i);
    }
}

int RemaniGridMap::toAddressUnlocked(const Eigen::Vector3i& index) const {
    return toAddressUnlocked(index.x(), index.y(), index.z());
}

int RemaniGridMap::toAddressUnlocked(int x, int y, int z) const {
    return x * map_voxel_num_.y() * map_voxel_num_.z() +
           y * map_voxel_num_.z() + z;
}

void RemaniGridMap::boundIndexUnlocked(Eigen::Vector3i& index) const {
    index.x() = std::max(0, std::min(index.x(), map_voxel_num_.x() - 1));
    index.y() = std::max(0, std::min(index.y(), map_voxel_num_.y() - 1));
    index.z() = std::max(0, std::min(index.z(), map_voxel_num_.z() - 1));
}

void RemaniGridMap::clearRegionUnlocked(const Eigen::Vector3i& min_index,
                                        const Eigen::Vector3i& max_index) {
    for (int x = min_index.x(); x <= max_index.x(); ++x) {
        for (int y = min_index.y(); y <= max_index.y(); ++y) {
            for (int z = min_index.z(); z <= max_index.z(); ++z) {
                const int address = toAddressUnlocked(x, y, z);
                occupancy_buffer_[address] = clamp_min_log_ - unknown_flag_;
                occupancy_buffer_inflate_[address] = 0;
                occupancy_buffer_neg_[address] = 0;
                distance_buffer_[address] = unknown_distance_m_;
                distance_buffer_neg_[address] = unknown_distance_m_;
                distance_buffer_all_[address] = unknown_distance_m_;
                tmp_buffer1_[address] = 0.0;
                tmp_buffer2_[address] = 0.0;
                count_hit_[address] = 0;
                count_hit_and_miss_[address] = 0;
            }
        }
    }
}

void RemaniGridMap::clearDistanceRegionUnlocked(const Eigen::Vector3i& min_index,
                                                const Eigen::Vector3i& max_index) {
    for (int x = min_index.x(); x <= max_index.x(); ++x) {
        for (int y = min_index.y(); y <= max_index.y(); ++y) {
            for (int z = min_index.z(); z <= max_index.z(); ++z) {
                const int address = toAddressUnlocked(x, y, z);
                occupancy_buffer_inflate_[address] = 0;
                occupancy_buffer_neg_[address] = 0;
                distance_buffer_[address] = unknown_distance_m_;
                distance_buffer_neg_[address] = unknown_distance_m_;
                distance_buffer_all_[address] = unknown_distance_m_;
                tmp_buffer1_[address] = 0.0;
                tmp_buffer2_[address] = 0.0;
                count_hit_[address] = 0;
                count_hit_and_miss_[address] = 0;
            }
        }
    }
}

void RemaniGridMap::clearStaleRegionUnlocked(const Eigen::Vector3i& keep_min,
                                             const Eigen::Vector3i& keep_max) {
    if (!has_last_update_bounds_) {
        return;
    }

    const int margin = local_map_margin_cells_ + local_bound_inflate_cells_;
    Eigen::Vector3i clear_min =
        last_update_min_index_ - Eigen::Vector3i::Constant(margin);
    Eigen::Vector3i clear_max =
        last_update_max_index_ + Eigen::Vector3i::Constant(margin);
    boundIndexUnlocked(clear_min);
    boundIndexUnlocked(clear_max);

    for (int x = clear_min.x(); x <= clear_max.x(); ++x) {
        for (int y = clear_min.y(); y <= clear_max.y(); ++y) {
            for (int z = clear_min.z(); z <= clear_max.z(); ++z) {
                const Eigen::Vector3i index(x, y, z);
                if (indexWithinBounds(index, keep_min, keep_max)) {
                    continue;
                }
                const int address = toAddressUnlocked(index);
                occupancy_buffer_[address] = clamp_min_log_ - unknown_flag_;
                occupancy_buffer_inflate_[address] = 0;
                occupancy_buffer_neg_[address] = 0;
                distance_buffer_[address] = unknown_distance_m_;
                distance_buffer_neg_[address] = unknown_distance_m_;
                distance_buffer_all_[address] = unknown_distance_m_;
                tmp_buffer1_[address] = 0.0;
                tmp_buffer2_[address] = 0.0;
                count_hit_[address] = 0;
                count_hit_and_miss_[address] = 0;
            }
        }
    }
}

int RemaniGridMap::cacheOccupancyObservationUnlocked(
    const Eigen::Vector3i& index,
    int occ,
    const Eigen::Vector3i& min_index,
    const Eigen::Vector3i& max_index) {
    if ((occ != 0 && occ != 1) || !isInMapUnlocked(index) ||
        !indexWithinBounds(index, min_index, max_index)) {
        return -1;
    }

    const int address = toAddressUnlocked(index);
    ++count_hit_and_miss_[address];
    if (count_hit_and_miss_[address] == 1) {
        cache_voxels_.push_back(index);
    }
    if (occ == 1) {
        ++count_hit_[address];
    }
    return address;
}

void RemaniGridMap::flushOccupancyUpdatesUnlocked(
    const Eigen::Vector3i& integration_min,
    const Eigen::Vector3i& integration_max) {
    for (const Eigen::Vector3i& index : cache_voxels_) {
        const int address = toAddressUnlocked(index);
        const int hit_count = count_hit_[address];
        const int total_count = count_hit_and_miss_[address];
        if (total_count <= 0) {
            continue;
        }

        const double log_odds_update =
            (hit_count >= (total_count - hit_count)) ? prob_hit_log_ : prob_miss_log_;

        count_hit_[address] = 0;
        count_hit_and_miss_[address] = 0;

        if (log_odds_update >= 0.0 && occupancy_buffer_[address] >= clamp_max_log_) {
            continue;
        }
        if (log_odds_update <= 0.0 && occupancy_buffer_[address] <= clamp_min_log_) {
            occupancy_buffer_[address] = clamp_min_log_;
            continue;
        }

        if (!indexWithinBounds(index, integration_min, integration_max)) {
            occupancy_buffer_[address] = clamp_min_log_;
        }

        occupancy_buffer_[address] = std::min(
            std::max(occupancy_buffer_[address] + log_odds_update, clamp_min_log_),
            clamp_max_log_);
    }
    cache_voxels_.clear();
}

void RemaniGridMap::inflateOccupiedUnlocked(
    const std::vector<Eigen::Vector3i>& occupied_indices,
    const Eigen::Vector3i& min_index,
    const Eigen::Vector3i& max_index) {
    for (const auto& index : occupied_indices) {
        for (int dx = -inflation_steps_; dx <= inflation_steps_; ++dx) {
            for (int dy = -inflation_steps_; dy <= inflation_steps_; ++dy) {
                for (int dz = -inflation_steps_; dz <= inflation_steps_; ++dz) {
                    const Eigen::Vector3i inflated = index + Eigen::Vector3i(dx, dy, dz);
                    if (!isInMapUnlocked(inflated) ||
                        !indexWithinBounds(inflated, min_index, max_index)) {
                        continue;
                    }
                    occupancy_buffer_inflate_[toAddressUnlocked(inflated)] = 1;
                }
            }
        }
    }
}

template <typename FGet, typename FSet>
void RemaniGridMap::fillESDF(FGet f_get_val, FSet f_set_val, int start, int end) {
    std::vector<int> v(static_cast<std::size_t>(end + 1), 0);
    std::vector<double> z(static_cast<std::size_t>(end + 2), 0.0);

    int k = start;
    v[static_cast<std::size_t>(start)] = start;
    z[static_cast<std::size_t>(start)] = -std::numeric_limits<double>::max();
    z[static_cast<std::size_t>(start + 1)] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; ++q) {
        ++k;
        double s = 0.0;
        do {
            --k;
            s = ((f_get_val(q) + q * q) -
                 (f_get_val(v[static_cast<std::size_t>(k)]) +
                  v[static_cast<std::size_t>(k)] * v[static_cast<std::size_t>(k)])) /
                (2.0 * q - 2.0 * v[static_cast<std::size_t>(k)]);
        } while (s <= z[static_cast<std::size_t>(k)]);

        ++k;
        v[static_cast<std::size_t>(k)] = q;
        z[static_cast<std::size_t>(k)] = s;
        z[static_cast<std::size_t>(k + 1)] = std::numeric_limits<double>::max();
    }

    k = start;
    for (int q = start; q <= end; ++q) {
        while (z[static_cast<std::size_t>(k + 1)] < q) {
            ++k;
        }
        const double value =
            (q - v[static_cast<std::size_t>(k)]) *
                (q - v[static_cast<std::size_t>(k)]) +
            f_get_val(v[static_cast<std::size_t>(k)]);
        f_set_val(q, value);
    }
}

void RemaniGridMap::updateESDF3dUnlocked(const Eigen::Vector3i& min_index,
                                         const Eigen::Vector3i& max_index) {
    const double inf = std::numeric_limits<double>::max();

    for (int x = min_index.x(); x <= max_index.x(); ++x) {
        for (int y = min_index.y(); y <= max_index.y(); ++y) {
            fillESDF(
                [&](int z) {
                    return occupancy_buffer_inflate_[toAddressUnlocked(x, y, z)] == 1
                               ? 0.0
                               : inf;
                },
                [&](int z, double value) {
                    tmp_buffer1_[toAddressUnlocked(x, y, z)] = value;
                },
                min_index.z(),
                max_index.z());
        }
    }

    for (int x = min_index.x(); x <= max_index.x(); ++x) {
        for (int z = min_index.z(); z <= max_index.z(); ++z) {
            fillESDF(
                [&](int y) { return tmp_buffer1_[toAddressUnlocked(x, y, z)]; },
                [&](int y, double value) {
                    tmp_buffer2_[toAddressUnlocked(x, y, z)] = value;
                },
                min_index.y(),
                max_index.y());
        }
    }

    for (int y = min_index.y(); y <= max_index.y(); ++y) {
        for (int z = min_index.z(); z <= max_index.z(); ++z) {
            fillESDF(
                [&](int x) { return tmp_buffer2_[toAddressUnlocked(x, y, z)]; },
                [&](int x, double value) {
                    distance_buffer_[toAddressUnlocked(x, y, z)] =
                        resolution_ * std::sqrt(std::max(0.0, value));
                },
                min_index.x(),
                max_index.x());
        }
    }

    for (int x = min_index.x(); x <= max_index.x(); ++x) {
        for (int y = min_index.y(); y <= max_index.y(); ++y) {
            for (int z = min_index.z(); z <= max_index.z(); ++z) {
                const int address = toAddressUnlocked(x, y, z);
                occupancy_buffer_neg_[address] =
                    occupancy_buffer_inflate_[address] == 0 ? 1 : 0;
            }
        }
    }

    for (int x = min_index.x(); x <= max_index.x(); ++x) {
        for (int y = min_index.y(); y <= max_index.y(); ++y) {
            fillESDF(
                [&](int z) {
                    return occupancy_buffer_neg_[toAddressUnlocked(x, y, z)] == 1
                               ? 0.0
                               : inf;
                },
                [&](int z, double value) {
                    tmp_buffer1_[toAddressUnlocked(x, y, z)] = value;
                },
                min_index.z(),
                max_index.z());
        }
    }

    for (int x = min_index.x(); x <= max_index.x(); ++x) {
        for (int z = min_index.z(); z <= max_index.z(); ++z) {
            fillESDF(
                [&](int y) { return tmp_buffer1_[toAddressUnlocked(x, y, z)]; },
                [&](int y, double value) {
                    tmp_buffer2_[toAddressUnlocked(x, y, z)] = value;
                },
                min_index.y(),
                max_index.y());
        }
    }

    for (int y = min_index.y(); y <= max_index.y(); ++y) {
        for (int z = min_index.z(); z <= max_index.z(); ++z) {
            fillESDF(
                [&](int x) { return tmp_buffer2_[toAddressUnlocked(x, y, z)]; },
                [&](int x, double value) {
                    distance_buffer_neg_[toAddressUnlocked(x, y, z)] =
                        resolution_ * std::sqrt(std::max(0.0, value));
                },
                min_index.x(),
                max_index.x());
        }
    }

    for (int x = min_index.x(); x <= max_index.x(); ++x) {
        for (int y = min_index.y(); y <= max_index.y(); ++y) {
            for (int z = min_index.z(); z <= max_index.z(); ++z) {
                const int address = toAddressUnlocked(x, y, z);
                distance_buffer_all_[address] = distance_buffer_[address];
                if (distance_buffer_neg_[address] > 0.0) {
                    distance_buffer_all_[address] +=
                        (-distance_buffer_neg_[address] + resolution_);
                }
            }
        }
    }
}

double RemaniGridMap::getDistanceVoxelUnlocked(const Eigen::Vector3i& index) const {
    Eigen::Vector3i bounded = index;
    boundIndexUnlocked(bounded);
    return distance_buffer_all_[toAddressUnlocked(bounded)];
}

bool RemaniGridMap::collectSurroundingDistancesUnlocked(
    const Eigen::Vector3d& position_world,
    double values[2][2][2],
    Eigen::Vector3d& diff) const {
    const Eigen::Vector3d shifted =
        position_world - 0.5 * resolution_ * Eigen::Vector3d::Ones();
    Eigen::Vector3i base_index;
    posToIndexUnlocked(shifted, base_index);

    if (base_index.x() < 0 || base_index.y() < 0 || base_index.z() < 0 ||
        base_index.x() >= map_voxel_num_.x() - 1 ||
        base_index.y() >= map_voxel_num_.y() - 1 ||
        base_index.z() >= map_voxel_num_.z() - 1) {
        return false;
    }

    Eigen::Vector3d base_position;
    indexToPosUnlocked(base_index, base_position);
    diff = (position_world - base_position) * resolution_inv_;
    diff = diff.cwiseMax(0.0).cwiseMin(1.0);

    for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 2; ++y) {
            for (int z = 0; z < 2; ++z) {
                const Eigen::Vector3i current = base_index + Eigen::Vector3i(x, y, z);
                if (!isWithinObservedBoundsUnlocked(current)) {
                    return false;
                }
                values[x][y][z] = getDistanceVoxelUnlocked(current);
            }
        }
    }
    return true;
}

EsdfQueryResult RemaniGridMap::queryDistanceAndGradient(
    const Eigen::Vector3d& position_world) const {
    EsdfQueryResult result;

    std::lock_guard<std::mutex> lock(mutex_);
    if (!isInMapUnlocked(position_world) ||
        !isWithinObservedBoundsUnlocked(position_world)) {
        return result;
    }

    double values[2][2][2];
    Eigen::Vector3d diff = Eigen::Vector3d::Zero();
    if (!collectSurroundingDistancesUnlocked(position_world, values, diff)) {
        return result;
    }

    const double v00 = (1.0 - diff.x()) * values[0][0][0] +
                       diff.x() * values[1][0][0];
    const double v01 = (1.0 - diff.x()) * values[0][0][1] +
                       diff.x() * values[1][0][1];
    const double v10 = (1.0 - diff.x()) * values[0][1][0] +
                       diff.x() * values[1][1][0];
    const double v11 = (1.0 - diff.x()) * values[0][1][1] +
                       diff.x() * values[1][1][1];
    const double v0 = (1.0 - diff.y()) * v00 + diff.y() * v10;
    const double v1 = (1.0 - diff.y()) * v01 + diff.y() * v11;

    result.distance = (1.0 - diff.z()) * v0 + diff.z() * v1;
    result.valid = true;
    result.observed = true;
    result.distance_valid = std::isfinite(result.distance);

    result.gradient.z() = (v1 - v0) * resolution_inv_;
    result.gradient.y() =
        ((1.0 - diff.z()) * (v10 - v00) + diff.z() * (v11 - v01)) *
        resolution_inv_;
    result.gradient.x() =
        (1.0 - diff.z()) * (1.0 - diff.y()) *
            (values[1][0][0] - values[0][0][0]) +
        (1.0 - diff.z()) * diff.y() *
            (values[1][1][0] - values[0][1][0]) +
        diff.z() * (1.0 - diff.y()) *
            (values[1][0][1] - values[0][0][1]) +
        diff.z() * diff.y() *
            (values[1][1][1] - values[0][1][1]);
    result.gradient.x() *= resolution_inv_;
    result.gradient_valid = result.gradient.allFinite();

    if (result.distance >= unknown_distance_m_ * 0.5) {
        result.gradient = Eigen::Vector3d::Zero();
        result.gradient_valid = true;
    }

    return result;
}

EsdfVisualizationData RemaniGridMap::buildVisualizationDataUnlocked() const {
    EsdfVisualizationData data;
    if (!has_last_update_bounds_) {
        return data;
    }

    Eigen::Vector3i publish_min =
        last_update_min_index_ - Eigen::Vector3i::Constant(local_map_margin_cells_);
    Eigen::Vector3i publish_max =
        last_update_max_index_ + Eigen::Vector3i::Constant(local_map_margin_cells_);
    boundIndexUnlocked(publish_min);
    boundIndexUnlocked(publish_max);

    for (int x = publish_min.x(); x <= publish_max.x(); ++x) {
        for (int y = publish_min.y(); y <= publish_max.y(); ++y) {
            for (int z = publish_min.z(); z <= publish_max.z(); ++z) {
                Eigen::Vector3d position_world;
                indexToPosUnlocked(Eigen::Vector3i(x, y, z), position_world);
                if (position_world.z() > config_.visualization.truncate_height_m) {
                    continue;
                }

                const int address = toAddressUnlocked(x, y, z);
                if (occupancy_buffer_[address] >= min_occupancy_log_) {
                    data.occupancy_points.emplace_back(position_world.cast<float>());
                }
                if (occupancy_buffer_inflate_[address] == 1) {
                    data.inflated_occupancy_points.emplace_back(
                        position_world.cast<float>());
                }
            }
        }
    }

    constexpr double kMinDistance = -3.0;
    constexpr double kMaxDistance = 3.0;
    for (int x = publish_min.x(); x <= publish_max.x(); ++x) {
        for (int y = publish_min.y(); y <= publish_max.y(); ++y) {
            Eigen::Vector3d position_world;
            indexToPosUnlocked(Eigen::Vector3i(x, y, publish_min.z()),
                               position_world);
            position_world.z() = config_.visualization.slice_height_m;
            if (!isInMapUnlocked(position_world) ||
                !isWithinObservedBoundsUnlocked(position_world)) {
                continue;
            }

            double values[2][2][2];
            Eigen::Vector3d diff = Eigen::Vector3d::Zero();
            if (!collectSurroundingDistancesUnlocked(position_world, values, diff)) {
                continue;
            }

            const double v00 = (1.0 - diff.x()) * values[0][0][0] +
                               diff.x() * values[1][0][0];
            const double v01 = (1.0 - diff.x()) * values[0][0][1] +
                               diff.x() * values[1][0][1];
            const double v10 = (1.0 - diff.x()) * values[0][1][0] +
                               diff.x() * values[1][1][0];
            const double v11 = (1.0 - diff.x()) * values[0][1][1] +
                               diff.x() * values[1][1][1];
            const double v0 = (1.0 - diff.y()) * v00 + diff.y() * v10;
            const double v1 = (1.0 - diff.y()) * v01 + diff.y() * v11;
            const double distance = (1.0 - diff.z()) * v0 + diff.z() * v1;
            if (!std::isfinite(distance)) {
                continue;
            }

            const double clamped =
                std::max(kMinDistance, std::min(kMaxDistance, distance));
            data.esdf_slice_points.emplace_back(
                static_cast<float>(position_world.x()),
                static_cast<float>(position_world.y()),
                static_cast<float>(position_world.z()),
                static_cast<float>((clamped - kMinDistance) /
                                   (kMaxDistance - kMinDistance)));
        }
    }

    return data;
}

EsdfVisualizationData RemaniGridMap::buildVisualizationData() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buildVisualizationDataUnlocked();
}

void RemaniGridMap::updateFromPointcloud(const Pointcloud& points_camera,
                                         const Transform& T_world_camera) {
    if (points_camera.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    const Eigen::Vector3d camera_world = T_world_camera.translation();
    if (!isInMapUnlocked(camera_world)) {
        return;
    }

    Eigen::Vector3i center_index;
    posToIndexUnlocked(camera_world, center_index);
    boundIndexUnlocked(center_index);

    Eigen::Vector3i integration_min = center_index;
    Eigen::Vector3i integration_max = center_index;
    Eigen::Vector3i local_min = center_index;
    Eigen::Vector3i local_max = center_index;
    for (int axis = 0; axis < 3; ++axis) {
        const int delta = static_cast<int>(
            std::ceil(local_update_range_(axis) / resolution_));
        integration_min(axis) -= delta;
        integration_max(axis) += delta;
        local_min(axis) -= delta + local_bound_inflate_cells_;
        local_max(axis) += delta + local_bound_inflate_cells_;
    }
    boundIndexUnlocked(integration_min);
    boundIndexUnlocked(integration_max);
    boundIndexUnlocked(local_min);
    boundIndexUnlocked(local_max);

    clearStaleRegionUnlocked(local_min, local_max);
    clearDistanceRegionUnlocked(local_min, local_max);

    cache_voxels_.clear();
    ++raycast_num_;

    GridRayCaster raycaster;
    for (const Point& point_camera : points_camera) {
        const Eigen::Vector3d point_world =
            T_world_camera * point_camera.cast<double>();
        if (!point_world.allFinite()) {
            continue;
        }

        Eigen::Vector3d endpoint_world = point_world;
        bool endpoint_is_hit = true;
        if (!isInMapUnlocked(endpoint_world)) {
            endpoint_world = clipPointToMapBoundary(
                endpoint_world,
                camera_world,
                map_min_boundary_,
                map_max_boundary_);
            endpoint_is_hit = false;
        }
        if (!endpoint_world.allFinite()) {
            continue;
        }

        Eigen::Vector3i endpoint_index;
        posToIndexUnlocked(endpoint_world, endpoint_index);
        if (!isInMapUnlocked(endpoint_index)) {
            boundIndexUnlocked(endpoint_index);
        }

        const int endpoint_address = cacheOccupancyObservationUnlocked(
            endpoint_index,
            endpoint_is_hit ? 1 : 0,
            local_min,
            local_max);
        if (endpoint_address != -1) {
            if (flag_rayend_[endpoint_address] == raycast_num_) {
                continue;
            }
            flag_rayend_[endpoint_address] = raycast_num_;
        }

        if (!raycaster.setInput(camera_world * resolution_inv_,
                                endpoint_world * resolution_inv_)) {
            continue;
        }

        Eigen::Vector3i traversed_index;
        while (raycaster.step(&traversed_index)) {
            const int address = cacheOccupancyObservationUnlocked(
                traversed_index,
                0,
                local_min,
                local_max);
            if (address == -1) {
                continue;
            }
            if (flag_traverse_[address] == raycast_num_) {
                break;
            }
            flag_traverse_[address] = raycast_num_;
        }
    }

    flushOccupancyUpdatesUnlocked(integration_min, integration_max);

    std::vector<Eigen::Vector3i> occupied_indices;
    occupied_indices.reserve(cache_voxels_.size());
    for (int x = local_min.x(); x <= local_max.x(); ++x) {
        for (int y = local_min.y(); y <= local_max.y(); ++y) {
            for (int z = local_min.z(); z <= local_max.z(); ++z) {
                const Eigen::Vector3i index(x, y, z);
                if (occupancy_buffer_[toAddressUnlocked(index)] >= min_occupancy_log_) {
                    occupied_indices.push_back(index);
                }
            }
        }
    }

    inflateOccupiedUnlocked(occupied_indices, local_min, local_max);
    updateESDF3dUnlocked(local_min, local_max);

    has_last_update_bounds_ = true;
    last_update_min_index_ = local_min;
    last_update_max_index_ = local_max;
    has_observed_bounds_ = true;
    observed_min_index_ = local_min;
    observed_max_index_ = local_max;

    ++processed_frames_;
}

}  // namespace camera_driver
