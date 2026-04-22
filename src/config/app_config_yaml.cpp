#include "realsense_voxblox_bridge/config/bridge_config.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include "config_internal.h"

namespace realsense_voxblox_bridge {

namespace {

bool parseAppConfigYamlFile(const std::string& file_path, AppConfig* config) {
    if (config == nullptr) {
        return false;
    }

    std::ifstream in(file_path);
    if (!in.is_open()) {
        std::cerr << "Failed to open runtime config: " << file_path << "\n";
        return false;
    }

    std::string line;
    int line_num = 0;
    while (std::getline(in, line)) {
        ++line_num;
        const size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        const size_t colon_pos = line.find(':');
        if (colon_pos == std::string::npos) {
            std::cerr << "Invalid line in runtime config " << file_path << ":"
                        << line_num << ": " << line << "\n";
            return false;
        }

        const std::string key = trim(line.substr(0, colon_pos));
        const std::string value = trim(line.substr(colon_pos + 1));
        if (key.empty() || value.empty()) {
            continue;
        }

        if (!setAppConfigValue(key, value, config)) {
            std::cerr << "Unknown or invalid config item in " << file_path << ":"
                        << line_num << ": " << key << "\n";
            return false;
        }
    }

    return true;
}

}  // namespace

bool loadAppConfigYaml(const std::string& file_path, AppConfig* config) {
    if (config == nullptr) {
        return false;
    }
    if (!std::filesystem::exists(file_path)) {
        std::cerr << "Runtime config not found: " << file_path << "\n";
        return false;
    }

    AppConfig parsed_config;
    if (!parseAppConfigYamlFile(file_path, &parsed_config)) {
        return false;
    }
    if (!validateConfig(parsed_config)) {
        std::cerr << "Invalid runtime config values in: " << file_path << "\n";
        return false;
    }

    parsed_config.app_config_path = file_path;
    *config = std::move(parsed_config);
    return true;
}

}  // namespace realsense_voxblox_bridge
