#include <iostream>
#include <string>

#include <VirtualRobot/VirtualRobot.h>

#include <boost/property_tree/json_parser.hpp>

#include "../include/Grasp/Parameters.hpp"
#include "../include/Grasp/Utils.hpp"

#include "GraspPlannerWindow.h"
#include "utils.hpp"

namespace pt = boost::property_tree;

int main(int argc, char *argv[]) {

    if (argc != 3) {
        std::cout   << "Error: incorrect number of parameters!!!\n"
                    << "Execution: ./grasp_visualization (-params <params_file> | -log <log_file>)\n";
        exit(1);
    }

    VirtualRobot::init(argc, argv, "Grasp Planner Visualizer");

    std::string mode = argv[1];
    std::string file = argv[2];

    GraspPlannerWindowParams params;
    if (mode == "-params") {
        std::cout << "Loading parameters file: " << file << std::endl;

        if (!Grasp::loadEnvParametersFile(file, params.planner_params)) {
            std::cout << "Error: parsing grasp planner params from file" << std::endl;
            exit(1);
        }

    } else if (mode == "-log") {
        std::cout << "Loading log file: " << file << std::endl;
        
        if (!loadLog(file, params.planner_params, params.grasps, params.best_grasps)) {
            std::cout << "Error: parsing log file" << std::endl;
            exit(1);
        }
    } else {
        std::cout << "Error: parameter " << mode << " is not valid\n"
                    << "Execution: ./grasp_visualization (-params <params_file> | -log <log_file>)\n";
        exit(1);
    }

    params.planner_params.verbose = true;
    GraspPlannerWindow graspPlannerW(params);

    graspPlannerW.main();

    return 0;
}
