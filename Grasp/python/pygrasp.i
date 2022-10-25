%module pygrasp

%{
    #include <Grasp/GraspVars.hpp>
    #include <Grasp/GraspPlannerParams.hpp>
    #include <Grasp/GraspPlannerIKParams.hpp>
    #include <Grasp/GraspResult.hpp>

    #include <Grasp/GraspExecutor.hpp>
    #include <Grasp/TestGramacyExecutor.hpp>
    #include <Grasp/GraspPlanner.hpp>
    #include <Grasp/GraspPlannerS.hpp>
    #include <Grasp/GraspPlannerIK.hpp>
%}

%include "std_string.i"
%include "std_vector.i"
namespace std {
    %template(vectord) vector<double>;
}

%include <eigen.i>

%eigen_typemaps(Eigen::Vector3f)
%eigen_typemaps(Eigen::Matrix3f)
%eigen_typemaps(Eigen::Matrix4f)
%eigen_typemaps(Eigen::MatrixXf)

%include "Grasp/GraspVars.hpp"
// %include "Grasp/GraspResult.hpp"

namespace Grasp {

    struct GraspResult
    {
        double measure;
        double volume;
        bool force_closure;

        double time;
        double pos_error;
        double ori_error;

        float rho;
        float roll;
        float pitch;
        float yaw;

        std::string error;
    };

    class GraspExecutor {
    public:
        virtual GraspResult executeQueryGrasp(const std::vector<double>& query) = 0;
    };

    class TestGramacyExecutor : public GraspExecutor {
    public:
        
        GraspResult executeQueryGrasp(const std::vector<double>& query);
    };

    struct GraspPlannerParams {
        std::string robot_file;
        std::string eef_name;
        std::string preshape;
        std::string object_file;

        bool has_eef_pose = false;
        Eigen::Vector3f eef_position, eef_orientation;

        bool has_obj_pose = false;
        Eigen::Vector3f obj_position, obj_orientation;

        GraspPlannerParams();

        GraspPlannerParams(
            const std::string& _robot_file,
            const std::string& _eef_name,
            const std::string& _preshape,
            const std::string& _object_file
        );
    };

    struct GraspPlannerIKParams {
        std::string scene;
        std::string reachability;
        std::string eef;
        std::string eef_preshape;
        std::string rns;
        std::vector<std::string> robot_cols;

        float max_error_pos, max_error_ori;
        float jacobian_step_size;
        int jacobian_max_loops;
        float cspace_path_step_size, cspace_col_step_size;

        GraspPlannerIKParams() {}
    };

    bool load_GraspPlannerParams_json(const std::string& json, GraspPlannerParams& params);
    bool load_GraspPlannerIKParams(const std::string& json, GraspPlannerIKParams& params);

    class GraspPlanner : public GraspExecutor {
    public:
        GraspPlanner(const GraspPlannerParams& params);
        GraspPlanner(const std::string& json_file);

        GraspResult executeQueryGrasp(const std::vector<double>& query);
    };

    class GraspPlannerS : public GraspExecutor {
    public:
        GraspPlannerS(const GraspPlannerParams& params);
        GraspPlannerS(const std::string& json_file);

        GraspResult executeQueryGrasp(const std::vector<double>& query);
    };

    class GraspPlannerIK : public GraspExecutor {
    public:
        GraspPlannerIK(const GraspPlannerIKParams& params);

        GraspResult executeQueryGrasp(const std::vector<double>& query);
    };
}

%inline %{
    Eigen::Vector3f test_eigen_numpy_type() {
        return Eigen::Vector3f(1.1, 2.2, 3.3);
    }
%}
