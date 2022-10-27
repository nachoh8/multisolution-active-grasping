#include "../include/Grasp/GraspPlanner.hpp"

#include <stdexcept>

#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/VirtualRobotException.h>

#include "../include/Grasp/Utils.hpp"

namespace Grasp {

GraspPlanner::GraspPlanner(const EnvParameters& params) : BaseGraspExecutor() {
    loadScene(params);
}

void GraspPlanner::loadScene(const EnvParameters& params) {
    VirtualRobot::ScenePtr scene = VirtualRobot::SceneIO::loadScene(params.scene_file);
    
    if (!scene)
    {
        VR_ERROR << " no scene ..." << std::endl;
        exit(1);
    }

    std::vector< VirtualRobot::RobotPtr > robots = scene->getRobots();

    if (robots.size() != 1)
    {
        VR_ERROR << "The scene can only have 1 robot" << std::endl;
        exit(1);
    }

    robot = robots[0];
    robot->setThreadsafe(false);

    VirtualRobot::EndEffectorPtr _eef = robot->getEndEffector(params.eef);

    if (!_eef)
    {
        VR_ERROR << "Need a correct EEF in robot" << std::endl;
        exit(1);
    }

    if (params.eef_preshape.empty()) {
        eef_preshape = "";
    } else {
        eef_preshape = params.eef_preshape;
        _eef->setPreshape(params.eef_preshape);
    }

    eefCloned = _eef->createEefRobot("eef", "icub_eef");
    eef = eefCloned->getEndEffector(params.eef);

    TCP = eef->getTcp();

    /// Load object
    std::vector< VirtualRobot::ManipulationObjectPtr > objects = scene->getManipulationObjects();

    if (objects.size() != 1)
    {
        VR_ERROR << "Need exactly 1 manipulation object" << std::endl;
        exit(1);
    }

    object = objects[0];
    
    /// Set quality measure
    qualityMeasure.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object));
    qualityMeasure->calculateObjectProperties();

    std::cout << "Scene loaded correctyly\n";
}

GraspResult GraspPlanner::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy) {

    // 1. Move EE
    Eigen::Matrix4f pose = poseVecToMatrix(xyz, rpy);
    // eefCloned->setGlobalPose(pose);
    eefCloned->setGlobalPoseForRobotNode(TCP, pose);

    // 2. Check Collisions
    GraspData grasp;
    grasp.pos = xyz;
    grasp.ori = rpy;
    if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), eef->createSceneObjectSet())) {
        std::cout << "Error: Collision detected!" << std::endl;
        grasp.result = GraspResult("eef_collision");
        
        grasps.push_back(grasp);

        return grasp.result;
    }

    // 3. Close EE
    closeEE();

    // 4. Evaluate grasp
    grasp.result = graspQuality();

    grasps.push_back(grasp);
    std::cout << "Grasp " << grasps.size() << ":\n";
    std::cout << poseVecToStr(xyz, rpy);
    std::cout << "Grasp Quality (epsilon measure):" << grasp.result.measure << std::endl;
    std::cout << "v measure:" << grasp.result.volume << std::endl;
    std::cout << "Force closure: " << (grasp.result.force_closure ? "yes" : "no") << std::endl;

    return grasp.result;
}

bool GraspPlanner::parseQuery(const std::vector<double>& query, Eigen::Vector3f& xyz, Eigen::Vector3f& rpy) {
    return queryToCartesian(query, xyz, rpy);
}

void GraspPlanner::reset() {
    openEE();
}

}
