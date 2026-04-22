#include <glog/logging.h>

#include "realsense_voxblox_bridge/bridge_app.h"

int main(int, char** argv) {
    google::InitGoogleLogging(argv[0]);
    return realsense_voxblox_bridge::runBridgeFromConfigFile("config/bridge.yaml");
}
