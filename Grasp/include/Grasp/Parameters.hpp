#pragma once

#include <string>

#include <Eigen/Geometry>

namespace Grasp {

struct EnvParameters {
    std::string scene_file; // scene xml file
    std::string eef; // end-effector name
    std::string eef_preshape; // default end-effector pose
};
}
