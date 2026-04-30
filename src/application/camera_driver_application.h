#pragma once

#include <string>

#include <rclcpp/node.hpp>

#include "camera_driver/config/camera_driver_config.h"

namespace camera_driver {

int runCameraDriverApplication(const CameraDriverConfig& config,
                               const rclcpp::Node::SharedPtr& node);
int runCameraDriverApplication(const CameraDriverConfig& config);
int runCameraDriverFromConfigFile(const std::string& esdf_config_path,
                                  const rclcpp::Node::SharedPtr& node);
int runCameraDriverFromConfigFile(
    const std::string& esdf_config_path = "config/esdf_param.yaml");
void requestCameraDriverStop();

}  // namespace camera_driver
