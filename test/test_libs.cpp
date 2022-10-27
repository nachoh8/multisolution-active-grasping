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
    //params.active_variables.push_back(GRASP_VAR::TRANS_X);
    params.active_variables.push_back(Grasp::CARTESIAN_VARS::TRANS_Y);
    params.active_variables.push_back(Grasp::CARTESIAN_VARS::TRANS_Z);
    /*params.active_variables.push_back(GRASP_VAR::ROT_ROLL);
    params.active_variables.push_back(GRASP_VAR::ROT_PITCH);
    params.active_variables.push_back(GRASP_VAR::ROT_YAW);*/

    const int opt_dim = params.active_variables.size();

    params.object = "WaterBottle";
    params.n_grasp_trials = 1;
    params.lower_bound = vectord(opt_dim, 0); // 0: X, 1: Y, 2: Z, 3: RX, 4: RY:, RZ: 5
    //params.lower_bound[0] = 0;
    params.lower_bound[0] = -110;
    params.lower_bound[1] = -2;
    // params.lower_bound[3] = -3.14;
    // params.lower_bound[4] = -3.14;
    // params.lower_bound[5] = -3.14;
    params.upper_bound = vectord(opt_dim, 1);
    // params.upper_bound[0] = 60;
    params.upper_bound[0] = 20;
    params.upper_bound[1] = 18;
    // params.upper_bound[3] = 3.14;
    // params.upper_bound[4] = 3.14;
    // params.upper_bound[5] = 3.14;
    params.default_query = vectord(Grasp::CARTESIAN_VEC_SIZE, 0);

    /*Grasp::GraspPlannerParams plannerParams;
    if (!Grasp::load_GraspPlannerParams_json("../config/grasp_params.json", plannerParams)) {
        std::cout << "Error: parsing grasp planner params\n";
        exit(1);
    }*/
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
