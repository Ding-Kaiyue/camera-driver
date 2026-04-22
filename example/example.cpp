#include <cstdlib>
#include <iostream>
#include <string>

#include <Eigen/Core>

#include "voxblox/core/esdf_map.h"
#include "voxblox/io/layer_io.h"

namespace {

struct Args {
    std::string esdf_path = "output/esdf.voxblox";
    Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d p2 = Eigen::Vector3d::Zero();
    bool interpolate = true;
};

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog
            << " [--esdf <path>] [--p1 x y z] [--p2 x y z] [--interpolate 0|1]\n"
            << "Example:\n"
            << "  " << prog
            << " --esdf output/esdf.voxblox --p1 0.4 0.1 0.2 --p2 0.6 -0.1 0.2\n";
}

bool parseDouble(const std::string& s, double* out) {
    if (out == nullptr) {
        return false;
    }
    try {
        *out = std::stod(s);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseInt(const std::string& s, int* out) {
    if (out == nullptr) {
        return false;
    }
    try {
        *out = std::stoi(s);
        return true;
    } catch (...) {
        return false;
    }
}

bool parsePointArg(int argc, char** argv, int* i, Eigen::Vector3d* point) {
    if (i == nullptr || point == nullptr) {
        return false;
    }
    if (*i + 3 >= argc) {
        return false;
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!parseDouble(argv[*i + 1], &x) || !parseDouble(argv[*i + 2], &y) ||
        !parseDouble(argv[*i + 3], &z)) {
        return false;
    }

    *point = Eigen::Vector3d(x, y, z);
    *i += 3;
    return true;
}

bool parseArgs(int argc, char** argv, Args* args) {
    if (args == nullptr) {
        return false;
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            return false;
        }
        if (arg == "--esdf") {
            if (i + 1 >= argc) {
                return false;
            }
            args->esdf_path = argv[++i];
        } else if (arg == "--p1") {
            if (!parsePointArg(argc, argv, &i, &args->p1)) {
                return false;
            }
        } else if (arg == "--p2") {
            if (!parsePointArg(argc, argv, &i, &args->p2)) {
                return false;
            }
        } else if (arg == "--interpolate") {
            if (i + 1 >= argc) {
                return false;
            }
            int value = 0;
            if (!parseInt(argv[++i], &value) || (value != 0 && value != 1)) {
                return false;
            }
            args->interpolate = (value == 1);
        } else {
            std::cerr << "Unknown arg: " << arg << "\n";
            return false;
        }
    }
    return true;
}

void queryPoint(const voxblox::EsdfMap& map,
                const Eigen::Vector3d& point,
                bool interpolate,
                const std::string& label) {
    double distance = 0.0;
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    const bool ok_distance =
        map.getDistanceAtPosition(point, interpolate, &distance);
    const bool ok_distance_gradient = map.getDistanceAndGradientAtPosition(
        point, interpolate, &distance, &gradient);

    std::cout << label << " = [" << point.x() << ", " << point.y() << ", "
                << point.z() << "]\n";
    if (!ok_distance) {
        std::cout << "  distance: unavailable (point not observed / outside ESDF)\n";
        return;
    }

    std::cout << "  distance: " << distance << " m\n";
    if (!ok_distance_gradient) {
        std::cout << "  gradient: unavailable (insufficient local ESDF support)\n";
        return;
    }

    std::cout << "  gradient: [" << gradient.x() << ", " << gradient.y() << ", "
                << gradient.z() << "]\n";
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parseArgs(argc, argv, &args)) {
        printUsage(argv[0]);
        return 1;
    }

    voxblox::Layer<voxblox::EsdfVoxel>::Ptr esdf_layer;
    if (!voxblox::io::LoadLayer<voxblox::EsdfVoxel>(args.esdf_path,
                                                    &esdf_layer) ||
        !esdf_layer) {
        std::cerr << "Failed to load ESDF layer from: " << args.esdf_path << "\n";
        return 2;
    }

    voxblox::EsdfMap esdf_map(esdf_layer);

    std::cout << "Loaded ESDF: " << args.esdf_path << "\n"
                << "interpolate: " << (args.interpolate ? "true" : "false")
                << "\n";

    queryPoint(esdf_map, args.p1, args.interpolate, "p1");
    queryPoint(esdf_map, args.p2, args.interpolate, "p2");

    return 0;
}
