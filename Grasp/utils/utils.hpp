#pragma once

#include <string>
#include <vector>
#include <map>

#include <Eigen/Geometry>
#include <boost/property_tree/json_parser.hpp>

#include "../include/Grasp/Grasp.hpp"

namespace pt = boost::property_tree;

inline int var_to_idx(const std::string& var) {
    if (var == "x") {
        return Grasp::CARTESIAN_VARS::TRANS_X;
    } else if (var == "y") {
        return Grasp::CARTESIAN_VARS::TRANS_Y;
    } else if ( var == "z") {
        return Grasp::CARTESIAN_VARS::TRANS_Z;
    } else if ( var == "rx") {
        return Grasp::CARTESIAN_VARS::ROT_ROLL;
    } else if ( var == "ry") {
        return Grasp::CARTESIAN_VARS::ROT_PITCH;
    } else if ( var == "rz") {
        return Grasp::CARTESIAN_VARS::ROT_YAW;
    } else if (var == "theta") {
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
    } else {
        return -1;
    }
}

bool load_graspsS(pt::ptree root, std::vector<Grasp::GraspData>& grasps) {
    pt::ptree root_grasps;
    std::map<std::string, float> var_value;
    try {
        pt::ptree root_gopt = root.get_child("basic_params");
        
        // parse active variables
        pt::ptree root_vars = root_gopt.get_child("active_variables");
        std::cout << "Active variables:";
        for (pt::ptree::value_type &v : root_vars)
        {
            std::string str_var = v.second.get_value<std::string>();
            int var = var_to_idx(str_var);
            if (var == -1) {
                std::cout << "Error: active variable " << str_var << " not valid!\n";
                return false;
            }
            std::cout << " " << str_var;
        }
        std::cout << std::endl;

        // parse default query
        pt::ptree root_q = root_gopt.get_child("default_query");
        std::vector<std::string> vars = {"theta", "phi", "rho", "rx", "ry", "rz"};
        for (auto& var : vars) {
            float v = root_q.get<float>(var);
            var_value.insert(std::pair<std::string, float>(var, v));
        }

        if (var_value.size() != 6) {
            std::cout << "Error: default query size is different from 6!\n";
            return false;
        }

        std::cout << "Default pose: (" << var_value["theta"] << " " << var_value["phi"] << " " << var_value["rho"]
                    << ", " << var_value["rx"] << " " << var_value["ry"] << " " << var_value["rz"] << ")\n";

        // get grasps root
        root_grasps = root.get_child("grasps");
    } catch(std::exception & e) {
        std::cout << "Error loading Grasps from file: " << e.what() << std::endl;

        return false;
    }

    // parse grasps
    try {
        for (auto &grasp_json : root_grasps)
        {
            pt::ptree grasp_obj = grasp_json.second;
            pt::ptree root_query = grasp_obj.get_child("query");
            
            std::vector<float> pose(6 , 0.0f);
            for (const auto& var_v : var_value) {
                pose[var_to_idx(var_v.first)] = root_query.get<float>(var_v.first, var_v.second);
            }

            pt::ptree root_res = grasp_obj.get_child("metrics");

            float comp_rho = root_res.get<float>("computed_rho");
            float comp_roll = root_res.get<float>("computed_roll");
            float comp_pitch = root_res.get<float>("computed_pitch");
            float comp_yaw = root_res.get<float>("computed_yaw");

            Grasp::GraspData grasp;
            grasp.result.measure = root_res.get<float>("outcome");
            grasp.result.volume = root_res.get<float>("volume");
            grasp.result.force_closure = root_res.get<bool>("force_closure");
            grasp.pos = Eigen::Vector3f(pose[Grasp::SPHERICAL_VARS::TRANS_THETA], pose[Grasp::SPHERICAL_VARS::TRANS_PHI], pose[Grasp::SPHERICAL_VARS::TRANS_RHO]);
            grasp.ori = Eigen::Vector3f(pose[Grasp::SPHERICAL_VARS::R_ROLL], pose[Grasp::SPHERICAL_VARS::R_PITCH], pose[Grasp::SPHERICAL_VARS::R_YAW]);

            if (comp_roll != 0 && comp_pitch != 0 && comp_yaw != 0){
                grasp.ori(0) = comp_roll;
                grasp.ori(1) = comp_pitch;
                grasp.ori(2) = comp_yaw;
            }

            if (comp_rho != 0){
                grasp.pos(2) = comp_rho;
            }

            if (grasp_obj.get_child_optional("others") != boost::none) {
                pt::ptree root_others = grasp_obj.get_child("others");
                
                grasp.result.time = root_others.get<float>("time", -1);
                grasp.result.pos_error = root_others.get<float>("position_error", -1);
                grasp.result.ori_error = root_others.get<float>("orientation_error", -1);
            }
            grasps.push_back(grasp);
        }

    } catch (std::exception & e) {
        std::cout << "Error parsing grasps: " << e.what() << std::endl;

        return false;
    }

    return true;
}

bool load_grasps(pt::ptree root, std::vector<Grasp::GraspData>& grasps, std::vector<Grasp::GraspData>& best_grasps) {
    pt::ptree root_grasps, root_best_grasps;
    std::map<std::string, float> var_value;
    try {
        pt::ptree root_gopt = root.get_child("basic_params");
        
        // parse active variables
        pt::ptree root_vars = root_gopt.get_child("active_variables");
        std::cout << "Active variables:";
        for (pt::ptree::value_type &v : root_vars)
        {
            std::string str_var = v.second.get_value<std::string>();
            int var = var_to_idx(str_var);
            if (var == -1) {
                std::cout << "Error: active variable " << str_var << " not valid!\n";
                return false;
            }
            std::cout << " " << str_var;
        }
        std::cout << std::endl;

        // parse default query
        pt::ptree root_q = root_gopt.get_child("default_query");
        std::vector<std::string> vars = {"x", "y", "z", "rx", "ry", "rz"};
        for (auto& var : vars) {
            float v = root_q.get<float>(var);
            var_value.insert(std::pair<std::string, float>(var, v));
        }

        if (var_value.size() != 6) {
            std::cout << "Error: default query size is different from 6!\n";
            return false;
        }

        std::cout << "Default pose: (" << var_value["x"] << " " << var_value["y"] << " " << var_value["z"]
                    << ", " << var_value["rx"] << " " << var_value["ry"] << " " << var_value["rz"] << ")\n";

        // get grasps root
        root_grasps = root.get_child("grasps");
        root_best_grasps = root.get_child("best_result");
    } catch(std::exception & e) {
        std::cout << "Error loading Grasps: " << e.what() << std::endl;

        return false;
    }

    // parse grasps
    try {
        for (auto &grasp_json : root_grasps)
        {
            pt::ptree grasp_obj = grasp_json.second;
            pt::ptree root_query = grasp_obj.get_child("query");
            
            std::vector<float> pose(6 , 0.0f);
            for (const auto& var_v : var_value) {
                pose[var_to_idx(var_v.first)] = root_query.get<float>(var_v.first, var_v.second);
            }

            pt::ptree root_res = grasp_obj.get_child("metrics");

            Grasp::GraspData grasp;
            grasp.result.measure = root_res.get<float>("outcome");
            grasp.result.volume = root_res.get<float>("volume");
            grasp.result.force_closure = root_res.get<bool>("force_closure");
            grasp.pos = Eigen::Vector3f(pose[Grasp::CARTESIAN_VARS::TRANS_X], pose[Grasp::CARTESIAN_VARS::TRANS_Y], pose[Grasp::CARTESIAN_VARS::TRANS_Z]);
            grasp.ori = Eigen::Vector3f(pose[Grasp::CARTESIAN_VARS::ROT_ROLL], pose[Grasp::CARTESIAN_VARS::ROT_PITCH], pose[Grasp::CARTESIAN_VARS::ROT_YAW]);

            if (grasp_obj.get_child_optional("others") != boost::none) {
                pt::ptree root_others = grasp_obj.get_child("others");
                
                grasp.result.time = root_others.get<float>("time", -1);
                grasp.result.pos_error = root_others.get<float>("position_error", -1);
                grasp.result.ori_error = root_others.get<float>("orientation_error", -1);
            }
            grasps.push_back(grasp);
        }

    } catch (std::exception & e) {
        std::cout << "Error parsing grasps: " << e.what() << std::endl;

        return false;
    }

    // parse best grasps
    try {
        for (auto &grasp_json : root_best_grasps)
        {
            pt::ptree grasp_obj = grasp_json.second;
            pt::ptree root_query = grasp_obj.get_child("query");
            
            std::vector<float> pose(6 , 0.0f);
            for (const auto& var_v : var_value) {
                pose[var_to_idx(var_v.first)] = root_query.get<float>(var_v.first, var_v.second);
            }

            Grasp::GraspData grasp;
            grasp.pos = Eigen::Vector3f(pose[Grasp::CARTESIAN_VARS::TRANS_X], pose[Grasp::CARTESIAN_VARS::TRANS_Y], pose[Grasp::CARTESIAN_VARS::TRANS_Z]);
            grasp.ori = Eigen::Vector3f(pose[Grasp::CARTESIAN_VARS::ROT_ROLL], pose[Grasp::CARTESIAN_VARS::ROT_PITCH], pose[Grasp::CARTESIAN_VARS::ROT_YAW]);

            float outcome = 0.0f;
            pt::ptree root_res = grasp_obj.get_child("metrics");
            for (auto &metric_json : root_res) {
                pt::ptree metric_obj = metric_json.second;
                if (metric_obj.get<std::string>("name") == "outcome") {
                    outcome = metric_obj.get<float>("value");
                }
            }

            grasp.result.measure = outcome;

            best_grasps.push_back(grasp);
        }

    } catch (std::exception & e) {
        std::cout << "Error parsing best grasps: " << e.what() << std::endl;

        return false;
    }

    return true;
}

bool load_root_params(const std::string& log_file, pt::ptree& root, pt::ptree& root_planner_params) {
    try {
        pt::read_json(log_file, root);
        pt::ptree root_executor = root.get_child("grasp_executor");
        root_planner_params = root_executor.get_child("params");
        
    } catch(std::exception & e) {
        std::cout << "Error loading roots from file: " << e.what() << std::endl;

        return false;
    }

    return true;
}
