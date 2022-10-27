#include "../include/Grasp/BaseGraspExecutor.hpp"

namespace Grasp {

GraspResult BaseGraspExecutor::executeQueryGrasp(const std::vector<double>& query) {
    Eigen::Vector3f xyz, rpy;
    if (parseQuery(query, xyz, rpy)) {
        reset();
        return executeGrasp(xyz, rpy);
    }

    std::cout << "Error: the query could not be parsed!!!\n";
    exit(1);
    return GraspResult();
}

GraspResult BaseGraspExecutor::graspQuality() {
    if (contacts.size() > 0) {
        qualityMeasure->setContactPoints(contacts);

        float volume = qualityMeasure->getVolumeGraspMeasure();
        float epsilon = qualityMeasure->getGraspQuality();
        bool fc = qualityMeasure->isGraspForceClosure();

        return GraspResult(epsilon, volume, fc);;
    }

    return GraspResult();
}

void BaseGraspExecutor::closeEE() {
    contacts.clear();
    contacts = eef->closeActors(object);
}

void BaseGraspExecutor::openEE() {
    contacts.clear();

    if (eef_preshape.empty()) {
        eef->openActors();
    }
    else
    {
        eef->setPreshape(eef_preshape);
    }
}


}
