#pragma once

#include <string>
#include <vector>
#include <boost/property_tree/json_parser.hpp>

#define MAX_ERROR_POS_DEFAULT 5.0f
#define MAX_ERROR_ORI_DEFAULT 0.04f
#define JACOBIAN_STEP_SIZE_DEFAULT 0.3f
#define JACOBIAN_MAX_LOOPS_DEFAULT 100
#define CSPACE_PATH_STEP_SIZE_DEFAULT 0.04f
#define CSPACE_COL_STEP_SIZE_DEFAULT 0.08f

namespace pt = boost::property_tree;

namespace Grasp {

struct GraspPlannerIKParams {
    std::string scene;
    std::string reachability;
    std::string eef;
    std::string eef_preshape;
    std::string rns;
    std::vector<std::string> robot_cols;

    float max_error_pos, max_error_ori;
    float jacobian_step_size;
    int jacobian_max_loops;
    float cspace_path_step_size, cspace_col_step_size;

    GraspPlannerIKParams() {}
};

bool load_GraspPlannerIKParams(const std::string& json, GraspPlannerIKParams& params);

bool load_GraspPlannerIKParams(pt::ptree root, GraspPlannerIKParams& params);

}
