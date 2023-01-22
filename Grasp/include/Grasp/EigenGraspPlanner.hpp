#pragma once

#include <Eigen/Geometry>

#include "GraspPlanner.hpp"
#include "Parameters.hpp"

namespace Grasp {

class EigenGraspPlanner : public GraspPlanner {
public:
    EigenGraspPlanner(const EnvParameters& params);

    GraspResult executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp=false);

protected:

    void reset();

    bool parseQuery(const std::vector<double>& query, Eigen::Vector3f& xyz, Eigen::Vector3f& rpy);

    void setPreshape(const Eigen::VectorXf& joint_values);
    
    // temporal amplitudes
    Eigen::Vector2f amplitude;

    /// Attributes
    std::vector< Eigen::VectorXf > eigengrasps = {Eigen::VectorXf(9), Eigen::VectorXf(9)};

    VirtualRobot::RobotConfigPtr eigen_eef_orig_config;
    VirtualRobot::EndEffectorPtr eigen_eef;
};

}
