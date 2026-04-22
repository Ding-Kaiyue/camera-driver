#ifndef REALSENSE_VOXBLOX_BRIDGE_CONFIG_INTERNAL_H_
#define REALSENSE_VOXBLOX_BRIDGE_CONFIG_INTERNAL_H_

#include <string>

#include "realsense_voxblox_bridge/config/bridge_config.h"

namespace realsense_voxblox_bridge {

bool parseIntArg(const std::string& value, int* out);
bool parseFloatArg(const std::string& value, float* out);
bool parseDoubleArg(const std::string& value, double* out);
bool parseBoolArg(const std::string& value, bool* out);

std::string trim(const std::string& input);
std::string normalizeKey(const std::string& key);

bool setAppConfigValue(const std::string& key_raw,
                       const std::string& value,
                       AppConfig* config);
bool validateConfig(const AppConfig& config);

}  // namespace realsense_voxblox_bridge

#endif  // REALSENSE_VOXBLOX_BRIDGE_CONFIG_INTERNAL_H_
