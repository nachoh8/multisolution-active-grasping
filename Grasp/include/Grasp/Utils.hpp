#pragma once

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Geometry>

#include "CoordSys.hpp"
#include "Parameters.hpp"

namespace Grasp {

inline int var_to_idx(const std::string& var) {
    if (var == "x") {
        return CARTESIAN_VARS::TRANS_X;
    } else if (var == "y") {
        return CARTESIAN_VARS::TRANS_Y;
    } else if ( var == "z") {
        return CARTESIAN_VARS::TRANS_Z;
    } else if ( var == "rx") {
        return CARTESIAN_VARS::ROT_ROLL;
    } else if ( var == "ry") {
        return CARTESIAN_VARS::ROT_PITCH;
    } else if ( var == "rz") {
        return CARTESIAN_VARS::ROT_YAW;
    } else {
        return -1;
    }
}

bool queryToCartesian(const std::vector<double>& query, Eigen::Vector3f& xyz, Eigen::Vector3f& rpy);

bool loadEnvParametersFile(const std::string& json_file, EnvParameters& params);

}
