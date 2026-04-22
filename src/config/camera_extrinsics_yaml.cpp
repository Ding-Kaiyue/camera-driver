#include "realsense_voxblox_bridge/config/bridge_config.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "config_internal.h"

namespace realsense_voxblox_bridge {

bool parseCameraExtrinsicsYaml(const std::string& file_path,
                               CameraExtrinsics* extrinsics) {
    if (extrinsics == nullptr) {
        return false;
    }

    std::ifstream in(file_path);
    if (!in.is_open()) {
        std::cerr << "Failed to open camera extrinsic config: " << file_path
                << "\n";
        return false;
    }

    bool has_tx = false;
    bool has_ty = false;
    bool has_tz = false;
    bool has_qx = false;
    bool has_qy = false;
    bool has_qz = false;
    bool has_qw = false;

    std::string line;
    while (std::getline(in, line)) {
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
            continue;
        }

        const std::string key = trim(line.substr(0, colon_pos));
        const std::string value_str = trim(line.substr(colon_pos + 1));
        if (key.empty() || value_str.empty()) {
            continue;
        }

        double value = 0.0;
        if (!parseDoubleArg(value_str, &value)) {
            std::cerr << "Invalid numeric value in " << file_path << ": " << line
                        << "\n";
            return false;
        }

        if (key == "tx") {
            extrinsics->tx = value;
            has_tx = true;
        } else if (key == "ty") {
            extrinsics->ty = value;
            has_ty = true;
        } else if (key == "tz") {
            extrinsics->tz = value;
            has_tz = true;
        } else if (key == "qx") {
            extrinsics->qx = value;
            has_qx = true;
        } else if (key == "qy") {
            extrinsics->qy = value;
            has_qy = true;
        } else if (key == "qz") {
            extrinsics->qz = value;
            has_qz = true;
        } else if (key == "qw") {
            extrinsics->qw = value;
            has_qw = true;
        }
    }

    if (!(has_tx && has_ty && has_tz && has_qx && has_qy && has_qz && has_qw)) {
        std::cerr << "Missing keys in " << file_path
                << ". Required: tx ty tz qx qy qz qw\n";
        return false;
    }

    const double q_norm = std::sqrt(extrinsics->qx * extrinsics->qx +
                                    extrinsics->qy * extrinsics->qy +
                                    extrinsics->qz * extrinsics->qz +
                                    extrinsics->qw * extrinsics->qw);
    if (!std::isfinite(q_norm) || q_norm < 1e-12) {
        std::cerr << "Invalid quaternion in " << file_path << "\n";
        return false;
    }

    extrinsics->qx /= q_norm;
    extrinsics->qy /= q_norm;
    extrinsics->qz /= q_norm;
    extrinsics->qw /= q_norm;

    return true;
}

}  // namespace realsense_voxblox_bridge
