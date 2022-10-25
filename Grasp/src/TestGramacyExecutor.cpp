#include "../include/Grasp/TestGramacyExecutor.hpp"

#include <cmath>

inline double sqr(double x) {return x*x;};

namespace Grasp {

GraspResult TestGramacyExecutor::executeQueryGrasp(const std::vector<double>& query) {
    
    if (query.size() != 2) return GraspResult();
    double x0 = query[0], x1 = query[1];

    x0 = (x0 * 8) - 2;
    x1 = (x1 * 8) - 2;

    double r = (x0 * std::exp(-sqr(x0)-sqr(x1)));
    
    return GraspResult(r, 0.0f, false);
}

}
