#pragma once

#include <cstddef>

#include <Eigen/Core>

#include "camera_driver/esdf/esdf_types.hpp"

namespace camera_driver {

class LiveEsdfInterface {
public:
    virtual ~LiveEsdfInterface() = default;

    virtual EsdfQueryResult queryDistanceAndGradient(
        const Eigen::Vector3d& position_world) const = 0;       // 查询 distance 和 gradient
    virtual bool isInsideMap(const Eigen::Vector3d& position_world) const = 0;  // 判断点是否在地图内
    virtual int processedFrames() const = 0;    // 返回处理过的点云帧数
    virtual std::size_t occupiedVoxelCount() const = 0; // 返回占据体素数量
};

}  // namespace camera_driver
