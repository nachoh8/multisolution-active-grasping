#pragma once

namespace Grasp {
struct GraspResult
{
    double measure;
    double volume;
    bool force_closure;

    double time; // ms
    double pos_error; // mm
    double ori_error; // degrees
    float rho; // Computed intersection rho
    float roll; //Computed or from spherical coords
    float pitch;
    float yaw;

    std::string error;

    GraspResult()
    {
        measure       = 0;
        volume        = 0;
        force_closure = false;

        time = -1;
        pos_error = -1;
        ori_error = -1;

        rho = 0;
        roll = 0;
        pitch = 0;
        yaw = 0;
    }

    GraspResult(const float _rho, const float _roll, const float _pitch, const float _yaw)
    {
        measure       = 0;
        volume        = 0;
        force_closure = false;

        time = -1;
        pos_error = -1;
        ori_error = -1;

        rho = 0;
        roll = _roll;
        pitch = _pitch;
        yaw = _yaw;
    }

    GraspResult(const double _measure, const double _volume, bool _force_closure, const float _rho, const float _roll, const float _pitch, const float _yaw)
    {
        measure       = _measure;
        volume        = _volume;
        force_closure = _force_closure;

        time = -1;
        pos_error = -1;
        ori_error = -1;

        rho = _rho;
        roll = _roll;
        pitch = _pitch;
        yaw = _yaw;
    }

    GraspResult(const std::string& _error)
    {
        error = _error;
        
        measure       = 0;
        volume        = 0;
        force_closure = false;

        time = -1;
        pos_error = -1;
        ori_error = -1;
    }

    GraspResult(const double _measure, const double _volume, const bool _force_closure)
    {
        measure       = _measure;
        volume        = _volume;
        force_closure = _force_closure;

        time = -1;
        pos_error = -1;
        ori_error = -1;
    }

    GraspResult(const double _measure, const double _volume, const bool _force_closure, const double _time, const double _pos_error, const double _ori_error)
    {
        measure       = _measure;
        volume        = _volume;
        force_closure = _force_closure;

        time = _time;
        pos_error = _pos_error;
        ori_error = _ori_error;
    }
};
}
