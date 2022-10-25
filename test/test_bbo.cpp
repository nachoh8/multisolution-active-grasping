#include "bayesopt/bayesopt.hpp"

class TestBBO : public bayesopt::ContinuousModel {

public:
    TestBBO(size_t dim, const bopt_params& bo_params)
    : bayesopt::ContinuousModel(dim, bo_params)
    {
    }

    /**
     * @brief Defines the actual function to be optimized
     * 
     * @param query point to be evaluated
     * @return value of the function at the point evaluated
     */
    double evaluateSample(const vectord& query) {
        return -1.0f;
    }

};

int main(int argc, char *argv[]) {
    bopt_params params = initialize_parameters_to_default();
    TestBBO (1, params);
    return 0;
}

