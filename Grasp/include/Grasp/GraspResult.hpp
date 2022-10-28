#pragma once

namespace Grasp {

struct GraspResult
{
    double measure; // epsilon measure
    double volume; // Volume grasp quality ratio of GWS volume / OWS volume
    bool force_closure; // True if wrench space origin is inside GWS-Hull

    std::string error;

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

/*struct GraspResultIK : GraspResult {
    double time; // execution time in ms
    double pos_error; // position error in mm
    double ori_error; // orientation error in degrees

    GraspResultIK() : GraspResult() {
    }

    GraspResultIK(const double _measure, const double _volume, const bool _force_closure, const double _time, const double _pos_error, const double _ori_error)
    {
        measure       = _measure;
        volume        = _volume;
        force_closure = _force_closure;

        time = _time;
        pos_error = _pos_error;
        ori_error = _ori_error;
    }
};*/

}
