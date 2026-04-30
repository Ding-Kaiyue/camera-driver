#pragma once

#include <Eigen/Core>

namespace camera_driver {

struct EsdfQueryResult {
    bool valid{false};
    bool observed{false};
    bool distance_valid{false};
    bool gradient_valid{false};
    double distance{0.0};
    Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
};

}  // namespace camera_driver
