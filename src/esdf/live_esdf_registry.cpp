#include "camera_driver/esdf/live_esdf_interface.hpp"
#include "camera_driver/esdf/live_esdf_registry.hpp"

#include <mutex>

namespace camera_driver {
namespace {

std::mutex g_live_esdf_mutex;
std::shared_ptr<const LiveEsdfInterface> g_live_esdf_map;

}  // namespace

std::shared_ptr<const LiveEsdfInterface> getLiveEsdfMap() {
    std::lock_guard<std::mutex> lock(g_live_esdf_mutex);
    return g_live_esdf_map;
}

void setLiveEsdfMap(std::shared_ptr<const LiveEsdfInterface> map) {
    std::lock_guard<std::mutex> lock(g_live_esdf_mutex);
    g_live_esdf_map = std::move(map);
}

void clearLiveEsdfMap() {
    std::lock_guard<std::mutex> lock(g_live_esdf_mutex);
    g_live_esdf_map.reset();
}

}  // namespace camera_driver
