#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace camera_driver {

using Point = Eigen::Vector3f;
using Pointcloud = std::vector<Point>;
using Transform = Eigen::Isometry3d;

}  // namespace camera_driver
