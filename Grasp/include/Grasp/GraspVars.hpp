#pragma once

namespace Grasp {

enum CARTESIAN_VARS {
    TRANS_X = 0,
    TRANS_Y,
    TRANS_Z,
    ROT_ROLL,
    ROT_PITCH,
    ROT_YAW
};

const std::size_t CARTESIAN_VARS_SIZE = 6;

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
    } /*else if (var == "theta") {
        return Grasp::SPHERICAL_VARS::TRANS_THETA;
    } else if (var == "phi") {
        return Grasp::SPHERICAL_VARS::TRANS_PHI;
    } else if ( var == "rho") {
        return Grasp::SPHERICAL_VARS::TRANS_RHO;
    } else if ( var == "rx") {
        return Grasp::SPHERICAL_VARS::R_ROLL;
    } else if ( var == "ry") {
        return Grasp::SPHERICAL_VARS::R_PITCH;
    } else if ( var == "rz") {
        return Grasp::SPHERICAL_VARS::R_YAW;
    }*/ else {
        return -1;
    }
}

// const unsigned int NUM_GRASP_VARS = 6;

}
