#include <iostream>
#include <string>

#include <VirtualRobot/VirtualRobot.h>

#include <boost/property_tree/json_parser.hpp>

#include "../include/Grasp/Parameters.hpp"
#include "../include/Grasp/Utils.hpp"

#include "GraspPlannerWindow.h"
#include "EigenGraspPlannerWindow.h"
#include "utils.hpp"

namespace pt = boost::property_tree;

struct GraspPlannerWindowParams {
    Grasp::EnvParameters planner_params;
    std::vector<Grasp::GraspData> grasps, best_grasps;
};

int main(int argc, char *argv[]) {

    if (argc < 3 || argc > 4) {
        std::cout   << "Error: incorrect number of parameters!!!\n"
                    << "Execution: ./grasp_visualization (-params <params_file> [-eigen] | -log <log_file>)\n";
        exit(1);
    } 

    VirtualRobot::init(argc, argv, "Grasp Planner Visualizer");

    std::string mode = argv[1];
    std::string file = argv[2];
    bool eigen_planner = false;

    GraspPlannerWindowParams params;
    if (mode == "-params") {
        std::cout << "Loading parameters file: " << file << std::endl;

        if (!Grasp::loadEnvParametersFile(file, params.planner_params)) {
            std::cout << "Error: parsing grasp planner params from file" << std::endl;
            exit(1);
        }

        eigen_planner = argc == 4 && std::string(argv[3]) == "-eigen";

    } else if (mode == "-log") {
        std::cout << "Loading log file: " << file << std::endl;
        std::string planner_name;
        if (!loadLog(file, planner_name, params.planner_params, params.grasps, params.best_grasps)) {
            std::cout << "Error: parsing log file" << std::endl;
            exit(1);
        }
        if (planner_name == "EigenGraspPlanner") {
            eigen_planner = true;
        }
    } else {
        std::cout << "Error: parameter " << mode << " is not valid\n"
                    << "Execution: ./grasp_visualization (-params <params_file> | -log <log_file>)\n";
        exit(1);
    }

    params.planner_params.verbose = true;
    if (eigen_planner) {
        std::cout << "Using Eigen Grasp Planner" << std::endl;

        EigenGraspPlannerWindow graspPlannerW(params.planner_params, params.grasps, params.best_grasps);

        graspPlannerW.main();
    } else {
        GraspPlannerWindow graspPlannerW(params.planner_params, params.grasps, params.best_grasps);

        graspPlannerW.main();
    }

    return 0;
}
