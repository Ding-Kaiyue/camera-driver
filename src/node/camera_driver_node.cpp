#include <iostream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "application/camera_driver_application.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::string config_path;
    if (argc >= 2) {
        config_path = argv[1];
    } else {
        config_path =
            ament_index_cpp::get_package_share_directory("camera_driver") +
            "/config/esdf_param.yaml";
    }

    const auto node = rclcpp::Node::make_shared("camera_driver");
    const int rc = camera_driver::runCameraDriverFromConfigFile(config_path, node);

    if (rc != 0) {
        std::cerr << "camera_driver node exited with code " << rc << std::endl;
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return rc;
}
