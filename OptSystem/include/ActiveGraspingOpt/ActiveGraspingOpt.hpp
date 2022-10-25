#pragma once

#include <random>
#include <vector>

#include <bayesopt/bayesopt.hpp>
#include <bayesopt/parameters.hpp>
#include <specialtypes.hpp>

#include <Grasp/GraspResult.hpp>

#include "ActiveGraspingOptParams.hpp"

namespace ActiveGraspingOpt {

class ActiveGraspingOpt : public bayesopt::ContinuousModel {
public:
    ActiveGraspingOpt(const ActiveGraspingOptParams& params, const bopt_params& bo_params);

    /**
     * @brief Defines the actual function to be optimized
     * 
     * @param query point to be evaluated
     * @return value of the function at the point evaluated
     */
    double evaluateSample(const vectord& query);
    
private:
    /// Init Methods


    /// Evaluate Methods

    /**
     * @brief Creates the query to be optimized from the values of the original query and of the default query param
     * 
     * @param query Original query with size <= work_dim
     * @return Final query with size = work_dim 
     */
    vectord createOptQuery(const vectord& query);

    /**
     * @brief Executes n trials
     * 
     * @param query point to be evaluated, with size = work_dim
     * @return Results for each trial
     */
    std::vector<Grasp::GraspResult> applyQueryToHand(const vectord& query);

    /**
     * @brief Computes the grasp quality
     * 
     * @param qualities Result of all trials
     * @return Average grasp quality measure
     */
    double evaluateGraspQuality(const std::vector<Grasp::GraspResult>& qualities);

    /// Attributes

    ActiveGraspingOptParams params;

    unsigned int work_dim;
    double      ymax, ymin;
};

}
