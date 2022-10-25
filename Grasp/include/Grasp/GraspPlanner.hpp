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

#include <Eigen/Geometry>

#include "GraspExecutor.hpp"
#include "GraspPlannerParams.hpp"

namespace Grasp {

class GraspPlanner : public GraspExecutor {
public:
    GraspPlanner(const GraspPlannerParams& params);

    GraspPlanner(const std::string& json_file);

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

    /// Others
    inline std::string modelPoseToStr(const Eigen::Vector3f& pos, const Eigen::Matrix3f& ori_m) {
        Eigen::Vector3f ori = ori_m.eulerAngles(0,1,2);
        std::stringstream  ss;
        ss << std::setprecision(3);
        ss << "Position(x,y,z):\n(" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n"
            << "Orientation(r,p,y):\n(" << ori.x() << ", " << ori.y() << ", " << ori.z() << ")\n";

        return ss.str();
    }

    /// Attributes
    GraspPlannerParams params;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::EndEffectorPtr eef;
    VirtualRobot::RobotPtr eefCloned;
    VirtualRobot::ManipulationObjectPtr object;

    VirtualRobot::EndEffector::ContactInfoVector contacts;

    std::vector<GraspData> grasps;

    GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure;
};

}
