#include "../include/Grasp/Utils.hpp"

#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;

bool loadEnvParams(const pt::ptree root, Grasp::EnvParameters& params) {
    std::string robot_json, obj_json;
    try {
        params.scene_file = root.get<std::string>("scene_file");
        params.eef = root.get<std::string>("eef");
        params.eef_preshape = root.get<std::string>("eef_preshape");
    } catch(std::exception & e) {
        std::cout << "Error loading EnvParameters: " << e.what() << std::endl;

        return false;
    }

    return true;
}

namespace Grasp {
    bool queryToCartesian(const std::vector<double>& query, Eigen::Vector3f& xyz, Eigen::Vector3f& rpy) {
        if (query.size() != CARTESIAN_VEC_SIZE) {
            std::cout << "Error: query size is different of " << CARTESIAN_VEC_SIZE << "!!!\n";
            return false;
        }
        xyz = Eigen::Vector3f(query[CARTESIAN_VARS::TRANS_X], query[CARTESIAN_VARS::TRANS_Y], query[CARTESIAN_VARS::TRANS_Z]);
        rpy = Eigen::Vector3f(query[CARTESIAN_VARS::ROT_ROLL], query[CARTESIAN_VARS::ROT_PITCH], query[CARTESIAN_VARS::ROT_YAW]);

        return true;
    }
    
    bool loadEnvParametersFile(const std::string& json_file, EnvParameters& params) {
        pt::ptree root;
        try {
            pt::read_json(json_file, root);
        } catch(std::exception & e) {
            std::cout << "Error loading EnvParameters from file: " << e.what() << std::endl;

            return false;
        }

        return loadEnvParams(root, params);
    }
}
