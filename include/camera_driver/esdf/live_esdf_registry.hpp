/**
 * 全局注册表接口，用于存储和访问当前的 Live ESDF 地图实例。
 * camera_driver 在运行时把当前的 Live ESDF 地图实例注册到这个全局注册表里，其他模块（如路径规划）可以通过这个接口访问当前的 ESDF 地图。
 * 这种设计避免了直接在模块之间传递 ESDF 地图实例的复杂性，同时也允许在未来支持多个 ESDF 地图实例（例如不同分辨率或不同区域的地图）。
 * 需要注意的是，这个接口只提供了访问当前 ESDF 地图实例的功能，并不负责 ESDF 地图实例的创建、更新或销毁。
 * 这些操作应该由 camera_driver 内部的 ESDF 地图管理模块来处理。
 */
#pragma once

#include <memory>

namespace camera_driver {

class LiveEsdfInterface;

std::shared_ptr<const LiveEsdfInterface> getLiveEsdfMap();
void setLiveEsdfMap(std::shared_ptr<const LiveEsdfInterface> map);
void clearLiveEsdfMap();

}  // namespace camera_driver
