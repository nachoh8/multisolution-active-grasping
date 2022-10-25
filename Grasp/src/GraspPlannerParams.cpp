#include "../include/Grasp/GraspPlannerParams.hpp"

#include <iostream>

#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;

bool parse_eigen(pt::ptree& json_vec, Eigen::Vector3f& vec) {
    std::vector<float> values;
    for (pt::ptree::value_type &v : json_vec)
    {
        float num = v.second.get_value<float>();
        values.push_back(num);
    }

    if (values.size() == 3) {
        vec = Eigen::Vector3f(values[0], values[1], values[2]);

        return true;
    }

    return false;
}

bool parse_pose(pt::ptree& root, Eigen::Vector3f& pos, Eigen::Vector3f& rot) {
    try {
        bool c_pos = parse_eigen(root.get_child("position"), pos);
        bool c_rot = parse_eigen(root.get_child("orientation"), rot);

        return c_pos && c_rot;
    } catch(std::exception & e) {

        return false;
    }
}

namespace Grasp {

bool load_GraspPlannerParams_json(const std::string& json, GraspPlannerParams& params) {
    pt::ptree root;
    try {
        pt::read_json(json, root);
    } catch(std::exception & e) {
        std::cout << "Error loading GraspPlannerParams from file: " << e.what() << std::endl;

        return false;
    }

    return load_GraspPlannerParams_json(root, params);
}


bool load_GraspPlannerParams_json(const pt::ptree root, GraspPlannerParams& params) {
    std::string robot_json, obj_json;
    try {
        robot_json = root.get<std::string>("robot");
        obj_json = root.get<std::string>("object");
    } catch(std::exception & e) {
        std::cout << "Error loading GraspPlannerParams: " << e.what() << std::endl;

        return false;
    }

    pt::ptree ro_root;
    try {
        pt::read_json(robot_json, ro_root);

        params.robot_file = ro_root.get<std::string>("file");
        params.eef_name = ro_root.get<std::string>("eef_name");
        params.preshape = ro_root.get<std::string>("preshape");

        params.has_eef_pose = parse_pose(ro_root, params.eef_position, params.eef_orientation);
    } catch(std::exception & e) {
        std::cout << "Error loading GraspPlannerParams, robot file: " << e.what() << std::endl;

        return false;
    }

    try {
        pt::read_json(obj_json, ro_root);

        params.object_file = ro_root.get<std::string>("file");

        params.has_obj_pose = parse_pose(ro_root, params.obj_position, params.obj_orientation);
    } catch(std::exception & e) {
        std::cout << "Error loading GraspPlannerParams, obj file: " << e.what() << std::endl;

        return false;
    }

    return true;
}

}

