#include <iostream>
#include <memory>

#include <bayesopt/parameters.hpp>

#include <Grasp/Grasp.hpp>

#include <ActiveGraspingOpt/ActiveGraspingOptParams.hpp>
#include <ActiveGraspingOpt/ActiveGraspingOpt.hpp>

void test_GraspPlanner() {

    std::cout << "Test GraspPlanner-BayesOpt\n";

    /// Set params

    ActiveGraspingOpt::ActiveGraspingOptParams params;
    params.active_variables.push_back(Grasp::CARTESIAN_VARS::TRANS_Y);
    params.active_variables.push_back(Grasp::CARTESIAN_VARS::TRANS_Z);

    const int opt_dim = params.active_variables.size();

    params.object = "bottle";
    params.n_grasp_trials = 1;
    params.lower_bound = vectord(opt_dim);
    params.lower_bound[0] = -20;
    params.lower_bound[1] = -150;
    params.upper_bound = vectord(opt_dim, 1);
    params.upper_bound[0] = 35;
    params.upper_bound[1] = 34;
    params.default_query = vectord(Grasp::CARTESIAN_VEC_SIZE, 0);

    Grasp::EnvParameters gpParams;
    std::string params_file = "../config/tests/GP/bottle_params.json";
    if (!Grasp::loadEnvParametersFile(params_file, gpParams)) {
        std::cout << "Error: loading params from " << params_file << std::endl;
        exit(1);
    }
    std::shared_ptr<Grasp::GraspPlanner> executor = std::make_shared<Grasp::GraspPlanner>(gpParams); // TODO when the pointer is Base -> segmentation fault when calling virtual functions
    std::vector<double> query = {0,0,0,0,0,0};
    executor->executeQueryGrasp(query);
    params.executor = executor;

    /// Optimize

    bopt_params opt_param;
    opt_param       = initialize_parameters_to_default();

    ActiveGraspingOpt::ActiveGraspingOpt opt(params, opt_param);

    vectord best_grasp(opt_dim);

    opt.optimize(best_grasp);

    /// Show result

    std::cout << "Result:\n";
    for (auto& pt : best_grasp) {
        std::cout << pt << std::endl;
    }
    

    std::cout << "END TEST\n";
}


int main(int argc, char *argv[]) {

    test_GraspPlanner();

    return 0;
}
