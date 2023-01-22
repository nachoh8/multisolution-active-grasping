#include "../include/Grasp/EigenGraspPlanner.hpp"

#include <stdexcept>

#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/VirtualRobotException.h>

#include "../include/Grasp/Utils.hpp"

namespace Grasp {

EigenGraspPlanner::EigenGraspPlanner(const EnvParameters& params) : GraspPlanner(params) {
    eigen_eef = robot->getEndEffector("Left Hand 9DOF");

    eigengrasps[0] << 25.31, 58.15, 47.65, 7.87, 21.34, 43.19, 28.86, 26.63, 18.11;
    eigengrasps[0] = eigengrasps[0] * M_PI / 180.0f;
    eigengrasps[1] << 24.98, 58.33, 46.67, 7.76, 21.49, 42.38, 29.25, 25.56, 17.75;
    eigengrasps[1] = eigengrasps[1] * M_PI / 180.0f;
    amplitude << 0.0, 0.0;
}


GraspResult EigenGraspPlanner::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp) {
    reset();

    if (verbose) {
        std::cout << "Grasp:" << std::endl;
        std::cout << "\tTCP target pose: ("
                << xyz.x() << " " << xyz.y() << " " << xyz.z()
                << ", " << rpy.x() << " " << rpy.y() << " " << rpy.z()
                << ")" << std::endl;
        std::cout << "\tAmplitude: (" << amplitude.x() << ", " << amplitude.y() << ")" << std::endl;
    }

    // 1. Move EE
    Eigen::Matrix4f pose = poseVecToMatrix(xyz, rpy);
    eefCloned->setGlobalPoseForRobotNode(TCP, pose);

    // 2. Set preshape
    GraspData grasp;
    grasp.eigen_amplitudes = amplitude;
    Eigen::VectorXf eigengrasp1 = eigengrasps[0] * amplitude[0];
    Eigen::VectorXf eigengrasp2 = eigengrasps[1] * amplitude[1];
    
    setPreshape(eigengrasp1 + eigengrasp2);

    // 3. Check Collisions
    grasp.pos = xyz;
    grasp.ori = rpy;
    if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), eef->createSceneObjectSet())) {
        if (verbose) {
            std::cout << "\tError: Collision detected!" << std::endl;
        }
        grasp.result = GraspResult("eef_collision");
        grasp.result.eigengrasp1 = eigengrasp1;
        grasp.result.eigengrasp2 = eigengrasp2;
        
        if (save_grasp) grasps.push_back(grasp);

        return grasp.result;
    }

    // 4. Close EE
    closeEE();

    // 5. Evaluate grasp
    grasp.result = graspQuality();
    grasp.result.eigengrasp1 = eigengrasp1;
    grasp.result.eigengrasp2 = eigengrasp2;

    if (verbose) {
        std::cout << "\tEpsilon measure: " << grasp.result.measure << std::endl
                << "\tVolume measure: " << grasp.result.volume << std::endl
                << "\tForce closure: " << (grasp.result.force_closure ? "yes" : "no") << std::endl;
    }

    if (save_grasp) grasps.push_back(grasp);

    return grasp.result;
}

bool EigenGraspPlanner::parseQuery(const std::vector<double>& query, Eigen::Vector3f& xyz, Eigen::Vector3f& rpy) {
    if (query.size() != CARTESIAN_VEC_SIZE + 2) {
        std::cout << "Error: query size is different of " << CARTESIAN_VEC_SIZE+2 << "!!!\n";
        return false;
    }

    amplitude << query[CARTESIAN_VEC_SIZE], query[CARTESIAN_VEC_SIZE+1];

    std::vector<double> cartseian_query(query.begin(), query.begin()+CARTESIAN_VEC_SIZE);
    return queryToCartesian(cartseian_query, xyz, rpy);
}

void EigenGraspPlanner::reset() {
    GraspPlanner::openEE();
    eigen_eef->setPreshape("Grasp Preshape");
}

void EigenGraspPlanner::setPreshape(const Eigen::VectorXf& joint_values) {
    eigen_eef->setPreshape("Grasp Preshape");

    if (verbose) {
        std::cout << "\tFinal posture (DOFs): ( ";
        for (size_t i = 0; i < joint_values.size(); i++) {
            std::cout << joint_values[i] << " ";
        }
        std::cout << ")\n";
    }

    auto actors = eigen_eef->getActors();
    for (size_t i = 0; i < actors.size(); i++) {
        auto _actor = actors[i];

        _actor->moveActor(joint_values[i]);
    }
    
    eefCloned->setJointValues(eigen_eef->getConfiguration());
}

}
