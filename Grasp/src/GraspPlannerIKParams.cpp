#include "../include/Grasp/GraspPlannerIKParams.hpp"

#include <iostream>

#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;

namespace Grasp {

bool load_GraspPlannerIKParams(const std::string& json, GraspPlannerIKParams& params) {
    pt::ptree root;
    try {
        pt::read_json(json, root);

    } catch(std::exception & e) {
        std::cout << "Error loading GraspPlannerIKParams: " << e.what() << std::endl;

        return false;
    }

    return load_GraspPlannerIKParams(root, params);
}

bool load_GraspPlannerIKParams(pt::ptree root, GraspPlannerIKParams& params) {
    try {
        params.scene = root.get<std::string>("scene");
        params.reachability = root.get<std::string>("reachability", "");
        params.eef = root.get<std::string>("eef");
        params.eef_preshape = root.get<std::string>("eef_preshape", "");
        params.rns = root.get<std::string>("rns");

        for (pt::ptree::value_type &v_col : root.get_child("robot_cols"))
        {
            std::string col = v_col.second.get_value<std::string>();
            params.robot_cols.push_back(col);
        }

        params.max_error_pos = root.get<float>("max_error_pos", MAX_ERROR_POS_DEFAULT);
        params.max_error_ori = root.get<float>("max_error_ori", MAX_ERROR_ORI_DEFAULT);

        params.jacobian_step_size = root.get<float>("jacobian_step_size", JACOBIAN_STEP_SIZE_DEFAULT);
        params.jacobian_max_loops = root.get<int>("jacobian_max_loops", JACOBIAN_MAX_LOOPS_DEFAULT);

        params.cspace_path_step_size = root.get<float>("cspace_path_step_size", CSPACE_PATH_STEP_SIZE_DEFAULT);
        params.cspace_col_step_size = root.get<float>("cspace_col_step_size", CSPACE_COL_STEP_SIZE_DEFAULT);

    } catch(std::exception & e) {
        std::cout << "Error loading GraspPlannerIKParams: " << e.what() << std::endl;

        return false;
    }

    return true;
}

}
