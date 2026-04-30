#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "camera_driver/application/camera_driver_interface.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <esdf_param_path> [run_seconds]\n";
        return 1;
    }

    const std::string config_path = argv[1];
    int run_seconds = 5;
    if (argc > 2) {
        run_seconds = std::max(1, std::atoi(argv[2]));
    }

    auto camera_driver_app = camera_driver::createCameraDriver();
    if (!camera_driver_app) {
        std::cerr << "Failed to create camera_driver interface.\n";
        return 1;
    }

    if (!camera_driver_app->startFromConfigFile(config_path)) {
        std::cerr << "Failed to start camera_driver with config: "
                  << config_path << "\n";
        return 2;
    }

    std::cout << "camera_driver started. Running for " << run_seconds
              << " seconds...\n";
    std::this_thread::sleep_for(std::chrono::seconds(run_seconds));
    std::cout << "Done. camera_driver will stop automatically when interface is destroyed.\n";
    return 0;
}
