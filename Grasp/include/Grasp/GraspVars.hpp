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

enum SPHERICAL_VARS
{
    TRANS_THETA = 0,
    TRANS_PHI,
    TRANS_RHO,
    R_ROLL,
    R_PITCH,
    R_YAW
};

const unsigned int NUM_GRASP_VARS = 6;

}
