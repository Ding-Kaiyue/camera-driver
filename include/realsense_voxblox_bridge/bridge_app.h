#ifndef REALSENSE_VOXBLOX_BRIDGE_BRIDGE_APP_H_
#define REALSENSE_VOXBLOX_BRIDGE_BRIDGE_APP_H_

#include "realsense_voxblox_bridge/config/bridge_config.h"

namespace realsense_voxblox_bridge {

int runBridge(const AppConfig& app_cfg);
int runBridgeFromConfigFile(const std::string& app_config_path = "config/bridge.yaml");

}  // namespace realsense_voxblox_bridge

#endif  // REALSENSE_VOXBLOX_BRIDGE_BRIDGE_APP_H_
