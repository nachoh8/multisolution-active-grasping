#pragma once

#include <vector>
#include <string>
#include <iomanip>
#include <sstream>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include "VirtualRobot/Grasping/GraspSet.h"

#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>
#include <GraspPlanning/GraspPlanner/GenericGraspPlanner.h>
#include <GraspPlanning/ApproachMovementSurfaceNormal.h>

#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/MathTools.h>

#include <Eigen/Geometry>

#include "GraspExecutor.hpp"
#include "GraspPlannerParams.hpp"

namespace Grasp {

class GraspPlannerS : public GraspExecutor {
public:
    GraspPlannerS(const GraspPlannerParams& params);

    GraspPlannerS(const std::string& json_file);

    /**
     * @brief Execute grasp from bayesopt query and computes its quality
     * 
     * @param query EEF Position
     * @return Grasp quality 
     */
    GraspResult executeQueryGrasp(const std::vector<double>& query);

    /**
     * @brief Execute grasp for an eef pose
     * 
     * @param xyz position @param rpy orientation 
     * @return Grasp quality
     */
    GraspResult executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp=true);

protected:
    /// Init Methods
    
    /**
     * @brief init simox environment (robo, object, etc)
     * 
     */
    void loadScene();

    /// Grasp Methods

    /**
     * @brief Move EEF to the desired position
     * 
     * @param xyz Position
     * @param rpy Rotation
     */
    void moveEE(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy);

    /**
     * @brief Measure the quality of the current grasp
     * 
     * @return GraspResult
     */
    GraspResult graspQuality();

    void closeEE();

    void openEE();

    bool RayIntersectsTriangle(Eigen::Vector3f rayOrigin, Eigen::Vector3f rayVector, 
                                                                Eigen::Vector3f vertex0, Eigen::Vector3f vertex1, Eigen::Vector3f vertex2, 
                                                                float& rho);

    /// Others
    inline std::string modelPoseToStr(const Eigen::Vector3f& pos, const Eigen::Matrix3f& ori_m) {
        //Eigen::Vector3f ori = ori_m.eulerAngles(0,1,2);
        //***
        Eigen::Matrix4f m_ori = Eigen::Matrix4f::Identity(4,4);
        m_ori.block(0,0,3,3) = ori_m;

        Eigen::Vector3f ori;

        VirtualRobot::MathTools::eigen4f2rpy(m_ori, ori);
        //***

        std::stringstream  ss;
        ss << std::setprecision(3);
        //ss << "Position(x,y,z):\n(" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n"
        ss << "Position(theta,phi,rho):\n(" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n"
            << "Orientation(r,p,y):\n(" << ori.x() << ", " << ori.y() << ", " << ori.z() << ")\n";

        return ss.str();
    }

    /// Attributes
    GraspPlannerParams params;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::EndEffectorPtr eef;
    VirtualRobot::TriMeshModelPtr objectModel;
    VirtualRobot::RobotPtr eefCloned;
    VirtualRobot::RobotNodePtr TCP;
    VirtualRobot::ManipulationObjectPtr object;

    VirtualRobot::EndEffector::ContactInfoVector contacts;

    float comp_rho;
    float comp_roll;
    float comp_pitch;
    float comp_yaw;

    Eigen::Matrix4f wOrigin;

    std::vector<GraspData> grasps;

    GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure;

    /// Distribution to draw random retreat distances from.
    std::uniform_real_distribution<float> distribRetreatDistance;

    std::default_random_engine randomEngine { std::random_device{}() };
};

}
