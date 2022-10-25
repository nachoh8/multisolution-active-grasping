#include <iostream>
#include <string>

#include <VirtualRobot/VirtualRobot.h>

#include <boost/property_tree/json_parser.hpp>

#include "../include/Grasp/GraspPlannerParams.hpp"

#include "GraspPlannerWindow.h"
#include "utils.hpp"

namespace pt = boost::property_tree;

bool load_params(const std::string& log_file, GraspPlannerWindowParams& params) {
    pt::ptree root, root_planner_params;

    if (!load_root_params(log_file, root, root_planner_params)) {
        return false;
    }

    if (!load_GraspPlannerParams_json(root_planner_params, params.planner_params)) {
        return false;
    }

    return load_grasps(root, params.grasps, params.best_grasps);
}

int main(int argc, char *argv[]) {

    if (argc != 3) {
        std::cout   << "Error: incorrect number of parameters!!!\n"
                    << "Execution: ./grasp_visualization <mode> <file>\n"
                    << "mode: 0 <grasp_params_file> or 1 <log_file>\n";
        exit(1);
    }

    int mode = std::stoi(argv[1]);
    std::string file = argv[2];

    VirtualRobot::init(argc, argv, "Grasp Planner Visualizer");

    GraspPlannerWindowParams params;
    if (mode == 0) { // from json params
        if (!Grasp::load_GraspPlannerParams_json(file, params.planner_params)) {
            std::cout << "Error: parsing grasp planner params from file " << file << std::endl;
            exit(1);
        }
    } else if (mode == 1) { // from log
        if (!load_params(file, params)) {
            std::cout << "Error: parsing params from log file " << file << std::endl;
            exit(1);
        }
    } else {
        std::cout << "Error: mode " << mode << " is not valid\n";
        exit(1);
    }

    GraspPlannerWindow graspPlanner(params);

    graspPlanner.main();

    return 0;
}
