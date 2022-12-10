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
    verbose = params.verbose;

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
    Eigen::Matrix4f tcp_init_pose = poseVecToMatrix(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(-1.57f, 0.0f, 0.0f));
    eefCloned->setGlobalPoseForRobotNode(TCP, tcp_init_pose);

    /// Load object
    VirtualRobot::ManipulationObjectPtr _object = scene->getManipulationObject(params.object);

    if (!_object)
    {
        VR_ERROR << "The manipulation object " << params.object << " is not in the scene" << std::endl;
        exit(1);
    }

    object = _object;
    
    /// Set quality measure
    qualityMeasure.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object));
    qualityMeasure->calculateObjectProperties();

    if (verbose) {
        std::cout << "====Scene loaded correctly====\n";

        std::cout << "EEF: " << params.eef << std::endl;
        std::cout << "EEF Preshape: " << params.eef_preshape << std::endl;

        VirtualRobot::BoundingBox obj_bbox = object->getCollisionModel()->getGlobalBoundingBox();
        Eigen::Vector3f bbox_size = (obj_bbox.getMax() - obj_bbox.getMin()).cwiseAbs();
        std::cout << "Object: " << object->getName() << " | Size (x,y,z): (" << bbox_size.x() << ", " << bbox_size.y() << ", " << bbox_size.z() << ")" << std::endl;
    }
}

GraspResult GraspPlanner::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp) {
    reset();

    if (verbose) {
        std::cout << "Grasp:" << std::endl;
        std::cout << "\tTCP target pose: ("
                << xyz.x() << " " << xyz.y() << " " << xyz.z()
                << ", " << rpy.x() << " " << rpy.y() << " " << rpy.z()
                << ")" << std::endl;
    }

    // 1. Move EE
    Eigen::Matrix4f pose = poseVecToMatrix(xyz, rpy);
    // eefCloned->setGlobalPose(pose);
    eefCloned->setGlobalPoseForRobotNode(TCP, pose);

    // 2. Check Collisions
    GraspData grasp;
    grasp.pos = xyz;
    grasp.ori = rpy;
    if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), eef->createSceneObjectSet())) {
        if (verbose) {
            std::cout << "\tError: Collision detected!" << std::endl;
        }
        grasp.result = GraspResult("eef_collision");
        
        if (save_grasp) grasps.push_back(grasp);

        return grasp.result;
    }

    // 3. Close EE
    closeEE();

    // 4. Evaluate grasp
    grasp.result = graspQuality();

    if (verbose) {
        std::cout << "\tEpsilon measure: " << grasp.result.measure << std::endl
                << "\tVolume measure: " << grasp.result.volume << std::endl
                << "\tForce closure: " << (grasp.result.force_closure ? "yes" : "no") << std::endl;
    }

    if (save_grasp) grasps.push_back(grasp);

    return grasp.result;
}

bool GraspPlanner::parseQuery(const std::vector<double>& query, Eigen::Vector3f& xyz, Eigen::Vector3f& rpy) {
    return queryToCartesian(query, xyz, rpy);
}

void GraspPlanner::reset() {
    openEE();
}

}
