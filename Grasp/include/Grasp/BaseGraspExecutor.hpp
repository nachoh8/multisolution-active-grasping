#pragma once

#include <vector>
#include <string>
#include <iomanip>
#include <sstream>

#include <Eigen/Geometry>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>
#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>

#include "GraspResult.hpp"

namespace Grasp {

struct GraspData {
    Eigen::Vector3f pos, ori;
    Eigen::Vector2f eigen_amplitudes;
    GraspResult result;
};

class BaseGraspExecutor {
public:
    BaseGraspExecutor() {}

    /**
     * @brief Execute grasp from a query and computes its quality.
     * Before executing the grasp, it reset the robot to its initial position.
     * WARNING: If query cannot be parsed it exit the program.
     * 
     * @param query
     * @return Grasp quality 
     */
    GraspResult executeQueryGrasp(const std::vector<double>& query);

    /**
     * @brief Execute grasp for the specified TCP pose
     * 
     * @param xyz position @param rpy orientation 
     * @return Grasp quality 
     */
    virtual GraspResult executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp=false) = 0;

protected:

    /**
     * @brief Measure the quality of the current grasp
     * 
     * @return GraspResult
     */
    GraspResult graspQuality();

    void closeEE();
    
    void openEE();

    /**
     * @brief reset robot to original pose
     */
    virtual void reset() = 0;

    /**
     * @brief parse query to TCP pose
     * 
     * @param query vector of doubles
     * @param xyz TCP position
     * @param rpy TCP orientation
     * @return True if query has been parsed correctly to xyz and rpy
     */
    virtual bool parseQuery(const std::vector<double>& query, Eigen::Vector3f& xyz, Eigen::Vector3f& rpy) = 0;
 
    inline Eigen::Matrix4f poseVecToMatrix(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy) {
        float x[6];
        x[0] = xyz.x();
        x[1] = xyz.y();
        x[2] = xyz.z();
        x[3] = rpy.x();
        x[4] = rpy.y();
        x[5] = rpy.z();

        Eigen::Matrix4f m;
        VirtualRobot::MathTools::posrpy2eigen4f(x, m);

        return m;
    }

    inline void poseMatrixToVec(const Eigen::Matrix4f& pose, Eigen::Vector3f& xyz, Eigen::Vector3f& rpy) {
        xyz = pose.block<3,1>(0,3);
        rpy = VirtualRobot::MathTools::eigen4f2rpy(pose);
    }

    inline std::string poseVecToStr(const Eigen::Vector3f& pos, const Eigen::Vector3f& ori) {
        // Eigen::Vector3f ori = ori_m.eulerAngles(0,1,2); // TODO: review conversion
        std::stringstream  ss;
        ss << std::setprecision(3);
        ss << "Position(x,y,z):\n(" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n"
            << "Orientation(r,p,y):\n(" << ori.x() << ", " << ori.y() << ", " << ori.z() << ")\n";

        return ss.str();
    }

    bool verbose;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::EndEffectorPtr eef;
    std::string eef_preshape;
    VirtualRobot::RobotNodePtr TCP;
    VirtualRobot::EndEffector::ContactInfoVector contacts;
    VirtualRobot::ManipulationObjectPtr object;

    GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure;
    
    std::vector<GraspData> grasps;
};

}
