#pragma once

#include "GraspExecutor.hpp"

namespace Grasp {
class TestGramacyExecutor : public GraspExecutor {
public:
    
    GraspResult executeQueryGrasp(const std::vector<double>& query);
};
}