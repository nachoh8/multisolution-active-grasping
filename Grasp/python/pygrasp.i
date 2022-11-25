%module pygrasp

%{
    #include <Grasp/CoordSys.hpp>
    #include <Grasp/Parameters.hpp>
    #include <Grasp/GraspResult.hpp>
    #include <Grasp/Utils.hpp>
    #include <Grasp/BaseGraspExecutor.hpp>
    #include <Grasp/GraspPlanner.hpp>
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

%include "Grasp/CoordSys.hpp"

namespace Grasp {

    struct GraspResult
    {
        double measure;
        double volume;
        bool force_closure;

        std::string error;
    };

    class BaseGraspExecutor {
    public:
        virtual GraspResult executeQueryGrasp(const std::vector<double>& query) = 0;
    };

    struct EnvParameters {
        bool verbose;
        std::string scene_file; // scene xml file
        std::string object; // target object
        std::string eef; // end-effector name
        std::string eef_preshape; // default end-effector pose

        EnvParameters();

        EnvParameters(
            bool _verbose,
            const std::string& _robot_file,
            const std::string& _eef_name,
            const std::string& _preshape,
            const std::string& _object_file
        );
    };

    bool loadEnvParametersFile(const std::string& json_file, EnvParameters& params);

    class GraspPlanner : public BaseGraspExecutor {
    public:
        GraspPlanner(const EnvParameters& params);

        GraspResult executeQueryGrasp(const std::vector<double>& query);
    };
}

%inline %{
    Eigen::Vector3f test_eigen_numpy_type() {
        return Eigen::Vector3f(1.1, 2.2, 3.3);
    }
%}
