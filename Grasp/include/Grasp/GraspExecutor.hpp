#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "GraspResult.hpp"

namespace Grasp {

struct GraspData {
    Eigen::Vector3f pos, ori;
    GraspResult result;
};

class GraspExecutor {
public:
    /**
     * @brief Execute grasp from bayesopt query and computes its quality
     * 
     * @param query EEF position (x,y,z,r,p,y)
     * @return Grasp quality 
     */
    virtual GraspResult executeQueryGrasp(const std::vector<double>& query) = 0;
};

}
