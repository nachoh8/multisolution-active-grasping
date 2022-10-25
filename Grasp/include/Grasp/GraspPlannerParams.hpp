#pragma once

#include <string>

#include <Eigen/Geometry>
#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;

namespace Grasp {

struct GraspPlannerParams {
    std::string robot_file;
    std::string eef_name;
    std::string preshape;
    std::string object_file;

    bool has_eef_pose = false;
    Eigen::Vector3f eef_position, eef_orientation;

    bool has_obj_pose = false;
    Eigen::Vector3f obj_position, obj_orientation;

    GraspPlannerParams() {}

    GraspPlannerParams(
        const std::string& _robot_file,
        const std::string& _eef_name,
        const std::string& _preshape,
        const std::string& _object_file) {

        robot_file = _robot_file;
        eef_name = _eef_name;
        preshape = _preshape;
        object_file = _object_file;
    }
};

bool load_GraspPlannerParams_json(const std::string& json, GraspPlannerParams& params);

bool load_GraspPlannerParams_json(const pt::ptree root, GraspPlannerParams& params);

}
