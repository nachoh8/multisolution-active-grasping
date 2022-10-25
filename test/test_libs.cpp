#include <iostream>
#include <memory>

#include <bayesopt/parameters.hpp>

#include <Grasp/Grasp.hpp>

#include <ActiveGraspingOpt/ActiveGraspingOptParams.hpp>
#include <ActiveGraspingOpt/ActiveGraspingOpt.hpp>


void test_gramacy() {
    std::cout << "Test Gramacy\n";

    ActiveGraspingOpt::ActiveGraspingOptParams params;
    params.active_variables.push_back(0);
    params.active_variables.push_back(1);
    params.n_grasp_trials = 1;

    const int opt_dim = params.active_variables.size();
    params.lower_bound = vectord(opt_dim, 0);
    params.upper_bound = vectord(opt_dim, 1);

    params.default_query = vectord(2, 0);

    std::shared_ptr<Grasp::GraspExecutor> executor = std::make_shared<Grasp::TestGramacyExecutor>();
    params.executor = executor;

    // bopt_params bo_params = ActiveGraspingOpt::initBoptParams();
    bopt_params opt_param;
    opt_param       = initialize_parameters_to_default();

    ActiveGraspingOpt::ActiveGraspingOpt opt(params, opt_param);

    vectord best_point(2);

    opt.optimize(best_point);

    for (auto& pt : best_point) {
        std::cout << pt << std::endl;
    }

    std::cout << "END TEST\n";
}


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
    params.default_query = vectord(Grasp::NUM_GRASP_VARS, 0);

    /*Grasp::GraspPlannerParams plannerParams;
    if (!Grasp::load_GraspPlannerParams_json("../config/grasp_params.json", plannerParams)) {
        std::cout << "Error: parsing grasp planner params\n";
        exit(1);
    }*/

    std::shared_ptr<Grasp::GraspExecutor> executor = std::make_shared<Grasp::GraspPlanner>("../config/grasp/tests/grasp_params.json");
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


void test_GraspPlannerIK() {

    std::cout << "Test GraspPlannerIK-BayesOpt (X,Y)\n";

    Grasp::GraspPlannerIKParams planner_params;
    if (!Grasp::load_GraspPlannerIKParams("../config/graspIK/tests/grasp_params.json", planner_params)) {
        exit(1);
    }

    // Grasp::GraspPlannerIK planner(planner_params);
    // std::vector<double> query = {-239.204, -32.972, 586.954, -1.57, 0.0,  3.14};
    // Grasp::GraspResult res = planner.executeQueryGrasp(query);

    ActiveGraspingOpt::ActiveGraspingOptParams opt_params;
    opt_params.active_variables.push_back(Grasp::CARTESIAN_VARS::TRANS_X);
    opt_params.active_variables.push_back(Grasp::CARTESIAN_VARS::TRANS_Y);
    
    const int opt_dim = opt_params.active_variables.size();
    opt_params.n_grasp_trials = 1;
    
    opt_params.lower_bound = vectord(opt_dim, 0);
    opt_params.lower_bound[0] = -236.0f;
    opt_params.lower_bound[1] = -80.0f;

    opt_params.upper_bound = vectord(opt_dim, 1);
    opt_params.upper_bound[0] = -191.0f;
    opt_params.upper_bound[1] = -26.0f;

    opt_params.default_query = vectord(Grasp::NUM_GRASP_VARS, 0);
    opt_params.default_query[2] = 541.0f;
    opt_params.default_query[3] = -1.57f;
    opt_params.default_query[4] = 0.0f;
    opt_params.default_query[5] = 3.14f;

    std::shared_ptr<Grasp::GraspExecutor> executor = std::make_shared<Grasp::GraspPlannerIK>(planner_params);
    opt_params.executor = executor;

    /// Optimize

    bopt_params bopt_param;
    bopt_param = initialize_parameters_to_default();
    bopt_param.n_init_samples = 4;
    bopt_param.n_iterations = 5;

    ActiveGraspingOpt::ActiveGraspingOpt opt(opt_params, bopt_param);

    vectord best_grasp(opt_dim);

    opt.optimize(best_grasp);

    /// Show result

    std::cout << "Result:\n";
    for (auto& pt : best_grasp) {
        std::cout << pt << std::endl;
    }
    
}


int main(int argc, char *argv[]) {

    test_gramacy();
    std::cout << "-----------------------------------------\n";
    test_GraspPlanner();
    std::cout << "-----------------------------------------\n";
    test_GraspPlannerIK();

    return 0;
}
