#pragma once

#include <Eigen/Geometry>

#include "BaseGraspExecutor.hpp"
#include "Parameters.hpp"

namespace Grasp {

class GraspPlanner : public BaseGraspExecutor {
public:
    GraspPlanner(const EnvParameters& params);

    GraspResult executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy);

protected:
    
    /**
     * @brief init simox environment (robot, object, etc)
     * 
     */
    void loadScene(const EnvParameters& params);

    void reset();

    bool parseQuery(const std::vector<double>& query, Eigen::Vector3f& xyz, Eigen::Vector3f& rpy);
    

    /// Attributes
    VirtualRobot::RobotPtr eefCloned;
};

}
