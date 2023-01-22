#pragma once

namespace Grasp {

enum CARTESIAN_VARS {
    TRANS_X = 0,
    TRANS_Y,
    TRANS_Z,
    ROT_ROLL,
    ROT_PITCH,
    ROT_YAW,
    AMPLITUDE1,
    AMPLITUDE2
};

const std::size_t CARTESIAN_VEC_SIZE = 6;

}
