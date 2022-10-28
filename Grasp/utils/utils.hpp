#pragma once

#include <string>
#include <vector>
#include <map>

#include <Eigen/Geometry>
#include <boost/property_tree/json_parser.hpp>

#include "../include/Grasp/Grasp.hpp"

namespace pt = boost::property_tree;

bool loadLogGrasps(pt::ptree root, std::vector<Grasp::GraspData>& grasps, std::vector<Grasp::GraspData>& best_grasps) {
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
            int var = Grasp::var_to_idx(str_var);
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

        if (var_value.size() != Grasp::CARTESIAN_VEC_SIZE) {
            std::cout << "Error: default query size is different from " << Grasp::CARTESIAN_VEC_SIZE << std::endl;
            return false;
        }

        std::cout << "Default pose: ("
                    << var_value["x"] << " " << var_value["y"] << " " << var_value["z"]
                    << ", " << var_value["rx"] << " " << var_value["ry"] << " " << var_value["rz"]
                    << ")" << std::endl;

        // get grasps root
        root_grasps = root.get_child("iterations");
        root_best_grasps = root.get_child("best_results");
    } catch(std::exception & e) {
        std::cout << "Error loading Grasps: " << e.what() << std::endl;

        return false;
    }

    // parse grasps
    try {
        for (auto &grasp_iteration : root_grasps) // iteration
        {
            for (auto &grasp_json : grasp_iteration.second) { // queries
                pt::ptree grasp_obj = grasp_json.second;
                pt::ptree root_query = grasp_obj.get_child("query");
                
                std::vector<float> pose(Grasp::CARTESIAN_VEC_SIZE);
                for (const auto& var_v : var_value) {
                    pose[Grasp::var_to_idx(var_v.first)] = root_query.get<float>(var_v.first, var_v.second);
                }

                Grasp::GraspData grasp;
                grasp.pos = Eigen::Vector3f(pose[Grasp::CARTESIAN_VARS::TRANS_X], pose[Grasp::CARTESIAN_VARS::TRANS_Y], pose[Grasp::CARTESIAN_VARS::TRANS_Z]);
                grasp.ori = Eigen::Vector3f(pose[Grasp::CARTESIAN_VARS::ROT_ROLL], pose[Grasp::CARTESIAN_VARS::ROT_PITCH], pose[Grasp::CARTESIAN_VARS::ROT_YAW]);

                pt::ptree root_res = grasp_obj.get_child("metrics");
                grasp.result.measure = root_res.get<float>("epsilon");

                if (grasp_obj.get_child_optional("metadata") != boost::none) {
                    pt::ptree root_metadata = grasp_obj.get_child("metadata");

                    grasp.result.volume = root_metadata.get<float>("volume");
                    grasp.result.force_closure = root_metadata.get<bool>("force_closure");
                    
                    /*grasp.result.time = root_others.get<float>("time", -1);
                    grasp.result.pos_error = root_others.get<float>("position_error", -1);
                    grasp.result.ori_error = root_others.get<float>("orientation_error", -1);*/
                }

                grasp.result.error = grasp_obj.get<std::string>("error", "");

                grasps.push_back(grasp);
            }
        }

    } catch (std::exception & e) {
        std::cout << "Error parsing grasps: " << e.what() << std::endl;

        return false;
    }

    std::cout << "Total grasps: " << grasps.size() << std::endl;

    // parse best grasps
    try {
        for (auto &grasp_json : root_best_grasps)
        {
            pt::ptree grasp_obj = grasp_json.second;
            pt::ptree root_query = grasp_obj.get_child("query");
            
            std::vector<float> pose(Grasp::CARTESIAN_VEC_SIZE);
            for (const auto& var_v : var_value) {
                pose[Grasp::var_to_idx(var_v.first)] = root_query.get<float>(var_v.first, var_v.second);
            }

            Grasp::GraspData grasp;
            grasp.pos = Eigen::Vector3f(pose[Grasp::CARTESIAN_VARS::TRANS_X], pose[Grasp::CARTESIAN_VARS::TRANS_Y], pose[Grasp::CARTESIAN_VARS::TRANS_Z]);
            grasp.ori = Eigen::Vector3f(pose[Grasp::CARTESIAN_VARS::ROT_ROLL], pose[Grasp::CARTESIAN_VARS::ROT_PITCH], pose[Grasp::CARTESIAN_VARS::ROT_YAW]);
            
            float outcome = 0.0f;
            pt::ptree root_res = grasp_obj.get_child("metrics");
            for (auto &metric_json : root_res) {
                pt::ptree metric_obj = metric_json.second;
                if (metric_obj.get<std::string>("name") == "epsilon") {
                    outcome = metric_obj.get<float>("value");
                }
            }

            grasp.result.measure = outcome;
            grasp.result.error = "";
            grasp.result.volume = 0.0f;
            grasp.result.force_closure = true;

            best_grasps.push_back(grasp);
        }

    } catch (std::exception & e) {
        std::cout << "Error parsing best grasps: " << e.what() << std::endl;

        return false;
    }

    std::cout << "Total best grasps: " << best_grasps.size() << std::endl;

    return true;
}

bool loadLog(const std::string& log_file, Grasp::EnvParameters& planner_params,
                std::vector<Grasp::GraspData>& grasps, std::vector<Grasp::GraspData>& best_grasps) {
    pt::ptree root;
    try {
        pt::read_json(log_file, root);
        pt::ptree root_executor = root.get_child("objective_function");
        std::string planner_name = root_executor.get<std::string>("name");
        pt::ptree root_planner_params = root_executor.get_child("params");

        if (planner_name == "GraspPlanner") {
            if (! Grasp::loadEnvParams(root_planner_params, planner_params)) {
                return false;
            }
        } else {
            std::cout << "Error: the planner " << planner_name << " does not exists" << std::endl;
            return false;    
        }

        return loadLogGrasps(root, grasps, best_grasps);
        
    } catch(std::exception & e) {
        std::cout << "Error loading log: " << e.what() << std::endl;

        return false;
    }
}
