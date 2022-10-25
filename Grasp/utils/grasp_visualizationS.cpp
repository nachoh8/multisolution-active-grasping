#include <iostream>
#include <string>

#include <VirtualRobot/VirtualRobot.h>

#include <boost/property_tree/json_parser.hpp>

#include "../include/Grasp/GraspPlannerParams.hpp"

#include "GraspPlannerWindowS.h"
#include "utils.hpp"

namespace pt = boost::property_tree;

bool load_params(const std::string& log_file, std::vector<Grasp::GraspData>& grasps) {
    pt::ptree root;
    try {
        pt::read_json(log_file, root);
        
    } catch(std::exception & e) {
        std::cout << "Error loading Grasps from file: " << e.what() << std::endl;

        return false;
    }

    return load_graspsS(root, grasps);
}

int main(int argc, char *argv[]) {

    if (argc < 2 || argc > 3) {
        std::cout   << "Error: incorrect number of parameters!!!\n"
                    << "Execution: ./grasp_visualizationS <grasp_params> [<log_file>]\n";
        exit(1);
    }

    std::string params_file = argv[1];

    VirtualRobot::init(argc, argv, "Grasp PlannerS Visualizer");

    Grasp::GraspPlannerParams plannerParams;
    if (!Grasp::load_GraspPlannerParams_json(params_file, plannerParams)) {
        std::cout << "Error: parsing grasp planner params file\n";
        exit(1);
    }

    GraspPlannerWindowSParams params;
    params.planner_params = plannerParams;

    if (argc == 3) {
        std::string log_file = argv[2];
        if (!load_params(log_file, params.grasps)) {
            std::cout << "Error: parsing grasps from res file\n";
            exit(1);
        }
    }

    GraspPlannerWindowS graspPlanner(params);

    graspPlanner.main();

    return 0;
}
