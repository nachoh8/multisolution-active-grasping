#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>

#include <string>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/property_tree/json_parser.hpp>

#include "../include/Grasp/GraspPlannerIKParams.hpp"

#include "GraspPlannerIKWindow.h"
#include "utils.hpp"

namespace pt = boost::property_tree;

bool load_params(const std::string& log_file, GraspPlannerIKWindowParams& params) {
    pt::ptree root, root_executor_params;
    try {
        pt::read_json(log_file, root);
        pt::ptree root_executor = root.get_child("grasp_executor");
        root_executor_params = root_executor.get_child("params");
        
    } catch(std::exception & e) {
        std::cout << "Error loading Grasps from file: " << e.what() << std::endl;

        return false;
    }

    bool ok = load_GraspPlannerIKParams(root_executor_params, params.planner_params);
    if (ok) {
        return load_grasps(root, params.grasps, params.best_grasps);
    }
    return false;
}

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cout   << "Error: incorrect number of parameters!!!\n"
                    << "Execution: ./grasp_ik_visualization <mode> <file>\n"
                    << "mode: 0 <grasp_params_file> or 1 <log_file>\n";
        exit(1);
    }

    int mode = std::stoi(argv[1]);
    std::string file = argv[2];

    VirtualRobot::init(argc, argv, "GraspPlanner IK visualization");
    std::cout << " --- START --- " << std::endl;

    GraspPlannerIKWindowParams params;
    if (mode == 0) {
        if (!Grasp::load_GraspPlannerIKParams(file, params.planner_params)) {
            exit(1);
        }
    } else if (mode == 1) {
        if (!load_params(file, params)) {
            std::cout << "Error: parsing params from log file\n";
            exit(1);
        }
    } else {
        std::cout << "Error: mode " << mode << " is not valid\n";
        exit(1);
    }

    GraspPlannerIKWindow plannerUI(params);

    plannerUI.main();

    return 0;

}
