#include "../include/Grasp/GraspPlanner.hpp"

#include <iomanip>
#include <stdexcept>

#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>

#include "../include/Grasp/GraspVars.hpp"

namespace Grasp {

/// Init

GraspPlanner::GraspPlanner(const GraspPlannerParams& params)
: params(params)
{
    loadScene();
}

GraspPlanner::GraspPlanner(const std::string& json_file)
{
    if (!load_GraspPlannerParams_json(json_file, this->params)) {
        throw "Errpr creating GraspPlanner from json";
    }

    loadScene();
}

void GraspPlanner::loadScene() {
    /// Load robot
    robot.reset();
    
    robot = VirtualRobot::RobotIO::loadRobot(params.robot_file);

    if (!robot) {
        exit(1);
    }

    VirtualRobot::EndEffectorPtr _eef = robot->getEndEffector(params.eef_name);

    if (!_eef) {
        exit(1);
    }

    if (!params.preshape.empty())
    {
        _eef->setPreshape(params.preshape);
    }

    eefCloned = _eef->createEefRobot("eef", "icub_eef");
    eef = eefCloned->getEndEffector(params.eef_name);

    if (params.has_eef_pose) {
        moveEE(params.eef_position, params.eef_orientation);
    }

    /// Load object
    object = VirtualRobot::ObjectIO::loadManipulationObject(params.object_file);

    if (!object) {
        exit(1);
    }

    if (params.has_obj_pose) {
        float x[6];
        x[0] = params.obj_position.x();
        x[1] = params.obj_position.y();
        x[2] = params.obj_position.z();
        x[3] = params.obj_orientation.x();
        x[4] = params.obj_orientation.y();
        x[5] = params.obj_orientation.z();

        Eigen::Matrix4f m;
        VirtualRobot::MathTools::posrpy2eigen4f(x, m);

        object->setGlobalPose(m);
    } else {
        // objectToTCP
        Eigen::Matrix4f pos =  eef->getTcp()->getGlobalPose();
        object->setGlobalPose(pos);
    }
    

    /// Set quality measure
    qualityMeasure.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object));
    qualityMeasure->calculateObjectProperties();
    
    // TODO: search about Grasp class
    // std::string name = "Grasp Planner - " + eef->getName();
    // grasps.reset(new VirtualRobot::GraspSet(name, eefCloned->getType(), params.eef_name));
}

/// Public

GraspResult GraspPlanner::executeQueryGrasp(const std::vector<double>& query) {
    if (query.size() != NUM_GRASP_VARS) {
        std::cerr << "Error: query size is different of " << NUM_GRASP_VARS << "!!!\n";
        exit(1);
    }

    // 1. query to position
    
    Eigen::Vector3f xyz(query[CARTESIAN_VARS::TRANS_X], query[CARTESIAN_VARS::TRANS_Y], query[CARTESIAN_VARS::TRANS_Z]);
    Eigen::Vector3f rpy(query[CARTESIAN_VARS::ROT_ROLL], query[CARTESIAN_VARS::ROT_PITCH], query[CARTESIAN_VARS::ROT_YAW]);

    // 2. Execute grasp

    return executeGrasp(xyz, rpy);
}

GraspResult GraspPlanner::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp) {

    // 1. Open and Move EE
    openEE();
    moveEE(xyz, rpy);

    // 2. Check Collisions
    GraspData grasp;
    grasp.pos = xyz;
    grasp.ori = rpy;
    if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), eef->createSceneObjectSet())) {
        std::cout << "Error: Collision detected!" << std::endl;
        grasp.result = GraspResult("eef_collision");
        if (save_grasp) grasps.push_back(grasp);
        return grasp.result;
    }

    // 3. Close EE
    closeEE();

    // 4. Evaluate grasp
    grasp.result = graspQuality();

    if (save_grasp) {
        grasps.push_back(grasp);
        std::cout << "Grasp " << grasps.size() << ":\n";
    }

    std::cout << "Grasp Quality (epsilon measure):" << grasp.result.measure << std::endl;
    std::cout << "v measure:" << grasp.result.volume << std::endl;
    std::cout << "Force closure: " << (grasp.result.force_closure ? "yes" : "no") << std::endl;

    return grasp.result;
}

/// Grasping

void GraspPlanner::moveEE(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy) {
    float x[6];
    x[0] = xyz.x();
    x[1] = xyz.y();
    x[2] = xyz.z();
    x[3] = rpy.x();
    x[4] = rpy.y();
    x[5] = rpy.z();

    Eigen::Matrix4f m;
    VirtualRobot::MathTools::posrpy2eigen4f(x, m);

    // m = eefCloned->getGlobalPose() * m;
    eefCloned->setGlobalPose(m);
}

GraspResult GraspPlanner::graspQuality() {
    if (contacts.size() > 0) {
        qualityMeasure->setContactPoints(contacts);

        float volume = qualityMeasure->getVolumeGraspMeasure();
        float epsilon = qualityMeasure->getGraspQuality();
        bool fc = qualityMeasure->isGraspForceClosure();

        return GraspResult(epsilon, volume, fc);;
    }

    std::cout << "GraspQuality: not contacts!!!\n";

    return GraspResult();
}

void GraspPlanner::closeEE()
{
    contacts.clear();
    
    contacts = eef->closeActors(object);
}

void GraspPlanner::openEE()
{
    contacts.clear();

    if (!params.preshape.empty())
    {
        eef->setPreshape(params.preshape);
    }
    else
    {
        eef->openActors();
    }
}

}
