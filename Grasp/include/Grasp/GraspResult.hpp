#pragma once

#include <Eigen/Geometry>

namespace Grasp {

struct GraspResult
{
    double measure; // epsilon measure
    double volume; // Volume grasp quality ratio of GWS volume / OWS volume
    bool force_closure; // True if wrench space origin is inside GWS-Hull

    std::string error;

    Eigen::VectorXf eigengrasp1, eigengrasp2;

    GraspResult()
    {
        error = "";

        measure       = 0;
        volume        = 0;
        force_closure = false;
    }

    GraspResult(const std::string& _error)
    {
        error = _error;
        
        measure       = 0;
        volume        = 0;
        force_closure = false;
    }

    GraspResult(const double _measure, const double _volume, const bool _force_closure)
    {
        measure       = _measure;
        volume        = _volume;
        force_closure = _force_closure;

        error = "";
    }

};

/*struct EigenGraspResult : GraspResult
{
    Eigen::VectorXf eigengrasp1, eigengrasp2;

    EigenGraspResult() : GraspResult()
    {
    }

    EigenGraspResult(const std::string& _error, const Eigen::VectorXf& _eigengrasp1) : GraspResult(_error)
    {
        eigengrasp1 = _eigengrasp1;
    }

    EigenGraspResult(const std::string& _error, const Eigen::VectorXf& _eigengrasp1, const Eigen::VectorXf& _eigengrasp2) : EigenGraspResult(_error, _eigengrasp1)
    {
        eigengrasp2 = _eigengrasp2;
    }

    EigenGraspResult(const double _measure, const double _volume, const bool _force_closure, const Eigen::VectorXf& _eigengrasp1) 
    : GraspResult(_measure, _volume, _force_closure)
    {
        eigengrasp1 = _eigengrasp1;
    }

    EigenGraspResult(const double _measure, const double _volume, const bool _force_closure, const Eigen::VectorXf& _eigengrasp1, const Eigen::VectorXf& _eigengrasp2) 
    : EigenGraspResult(_measure, _volume, _force_closure, _eigengrasp1)
    {
        eigengrasp2 = _eigengrasp2;
    }
};*/
}
