#pragma once

#include <string>

#include <Eigen/Geometry>

namespace Grasp {

struct EnvParameters {
    std::string scene_file; // scene xml file
    std::string object; // target object
    std::string eef; // end-effector name
    std::string eef_preshape; // default end-effector pose

    EnvParameters() {
        scene_file = "";
        object = "";
        eef = "";
        eef_preshape = "";
    }

    EnvParameters(
        const std::string& _scene_file,
        const std::string& _object,
        const std::string& _eef,
        const std::string& _eef_preshape
    ) {
        scene_file = _scene_file;
        object = _object;
        eef = _eef;
        eef_preshape = _eef_preshape;
    }
};
}
