#include "../include/Grasp/GraspPlannerIK.hpp"

#include <vector>

#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/IK/GenericIKSolver.h>

#include <MotionPlanning/Planner/GraspIkRrt.h>
#include <MotionPlanning/CSpace/CSpaceSampled.h>
#include <MotionPlanning/PostProcessing/ShortcutProcessor.h>

inline void poseError(const Eigen::Matrix4f& currentPose, const Eigen::Matrix4f& targetPose, float& posError, float& oriError) {
    posError = (currentPose.block(0, 3, 3, 1) - targetPose.block(0, 3, 3, 1)).norm();
    
    VirtualRobot::MathTools::Quaternion q1 = VirtualRobot::MathTools::eigen4f2quat(currentPose);
    VirtualRobot::MathTools::Quaternion q2 = VirtualRobot::MathTools::eigen4f2quat(targetPose);
    VirtualRobot::MathTools::Quaternion d = getDelta(q1, q2);
    oriError = fabs(180.0f - (d.w + 1.0f) * 90.0f);
}

namespace Grasp {

/// INIT

GraspPlannerIK::GraspPlannerIK(const GraspPlannerIKParams& params)
    : params(params)
{
    useOnlyPosition = false;
    useReachability = false;
    useCollision = true;

    ikMaxErrorPos = params.max_error_pos;
    ikMaxErrorOri = params.max_error_ori;

    ikJacobianStepSize = params.jacobian_step_size;
    ikJacobianMaxLoops = params.jacobian_max_loops;

    cspacePathStepSize = params.cspace_path_step_size;
    cspaceColStepSize = params.cspace_col_step_size;

    loadScene();

    loadReach();

    /// Set quality measure

    qualityMeasure.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object));
    qualityMeasure->calculateObjectProperties();

    printInfo();
}

void GraspPlannerIK::loadScene()
{
    VirtualRobot::ScenePtr scene = VirtualRobot::SceneIO::loadScene(params.scene);

    if (!scene)
    {
        VR_ERROR << " no scene ..." << std::endl;
        exit(1);
    }

    std::vector< VirtualRobot::RobotPtr > robots = scene->getRobots();

    if (robots.size() != 1)
    {
        VR_ERROR << "Need exactly 1 robot" << std::endl;
        exit(1);
    }

    robot = robots[0];
    robot->setThreadsafe(false);

    rns = robot->getRobotNodeSet(params.rns);

    if (!rns)
    {
        VR_ERROR << "Need a correct robot node set in robot" << std::endl;
        exit(1);
    }
    
    /// Load EEF

    eef = robot->getEndEffector(params.eef);

    if (!eef)
    {
        VR_ERROR << "Need a correct EEF in robot" << std::endl;
        exit(1);
    }

    if (!params.eef_preshape.empty()) {
        eef->setPreshape(params.eef_preshape);
    }
    rns->getJointValues(startConfig);

    /// Load Object

    std::vector< VirtualRobot::ManipulationObjectPtr > objects = scene->getManipulationObjects();

    if (objects.size() != 1)
    {
        VR_ERROR << "Need exactly 1 manipulation object" << std::endl;
        exit(1);
    }

    object = objects[0];

    /// Load Environment
    obstacles = scene->getObstacles();
    std::vector<VirtualRobot::SceneObjectSetPtr> soss = scene->getSceneObjectSets();
    if (soss.size() > 1) {
        VR_ERROR << "Only 1 objet set is allowed" << std::endl;
        exit(1);
    }

    /// Add collisions

    cdm.reset(new VirtualRobot::CDManager());
    cdm->addCollisionModel(object);
    if (soss.size() == 1) {
        cdm->addCollisionModel(soss[0]);
    }
    
    for (auto& r_col : params.robot_cols) {
        VirtualRobot::SceneObjectSetPtr col = robot->getRobotNodeSet(r_col);
        cdm->addCollisionModel(col);
    }
    
}

void GraspPlannerIK::loadReach()
{
    if (params.reachability == "") return;

    reachSpace.reset(new VirtualRobot::Reachability(robot));

    try
    {
        reachSpace->load(params.reachability);
        useReachability = true;
    }
    catch (VirtualRobot::VirtualRobotException& e)
    {
        std::cout << " ERROR while loading reach space" << std::endl;
        std::cout << e.what();
        reachSpace.reset();
        return;
    }
}

/// Public

GraspResult GraspPlannerIK::executeQueryGrasp(const std::vector<double>& query) {
    if (query.size() != 6) {
        std::cout << "Error: query needs 6 values (x,y,z,r,p,y)" << std::endl;
        // return GraspResult();
        exit(1);
    }

    float x[6];
    x[0] = query[0];
    x[1] = query[1];
    x[2] = query[2];
    x[3] = query[3];
    x[4] = query[4];
    x[5] = query[5];

    Eigen::Matrix4f targetPose;
    VirtualRobot::MathTools::posrpy2eigen4f(x, targetPose);

    return executeGrasp(targetPose);
}

GraspResult GraspPlannerIK::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy) {
    /// pos, rpy -> pose matrix
    float x[6];
    x[0] = xyz.x();
    x[1] = xyz.y();
    x[2] = xyz.z();
    x[3] = rpy.x();
    x[4] = rpy.y();
    x[5] = rpy.z();

    Eigen::Matrix4f targetPose;
    VirtualRobot::MathTools::posrpy2eigen4f(x, targetPose);

    return executeGrasp(targetPose);
}

GraspResult GraspPlannerIK::executeGrasp(const Eigen::Matrix4f& targetPose) {
    /// 1. To start config
    reset();

    /// 2. Plan and execute
    float posError = -1.0f, oriError = -1.0f, planTime = -1.0f;
    std::string error;
    if (plan(targetPose, posError, oriError, planTime, error)) {
        /// 3. Set final config
        Eigen::VectorXf finalPos;
        birrtSolOptimized->interpolate(1, finalPos);
        robot->setJointValues(rns, finalPos);

        /// 4. Measure grasp quality
        if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), eef->createSceneObjectSet())) {
            std::cout << "Error: EEF Collision detected!" << std::endl;
            reset();
            return GraspResult("eef_collision");
        }

        closeEEF();
        GraspResult result = graspQuality();
        result.pos_error = posError;
        result.ori_error = oriError;
        result.time = planTime;

        std::cout << "Grasp Quality (epsilon measure):" << result.measure << std::endl;
        std::cout << "Volume measure:" << result.volume << std::endl;
        std::cout << "Force closure: " << (result.force_closure ? "yes" : "no") << std::endl;

        return result;
    }

    reset();

    return GraspResult(error);
}

void GraspPlannerIK::printInfo() {
    std::cout << "---------------------CONFIGURATION------------------------\n";
    std::cout << "Scene: " << params.scene << std::endl;
    std::cout << "Robot: " << robot->getName() << std::endl;
    std::cout << "EEF: " << eef->getName() << std::endl;
    std::cout << "EEF Prehsape: " << params.eef_preshape << std::endl;
    std::cout << "Kinematic chain: " << rns->getName() << std::endl;
    std::cout << "Manipulation Object: " << object->getName() << std::endl;

    std::cout << "Use colllisions: " << useCollision << std::endl;
    std::cout << "Use Reachability: " << useReachability << ", FILE " << params.reachability << std::endl;
    std::cout << "Use Only position: " << useOnlyPosition << std::endl;

    std::cout << "IK Solver Max error -> Position: " << ikMaxErrorPos << " | Ori: " << ikMaxErrorOri << std::endl;
    std::cout << "IK Solver Jacobian -> Step size: " << ikJacobianStepSize << " | Max loops: " << ikJacobianMaxLoops << std::endl;
    std::cout << "Cspace -> Path step size: " << cspacePathStepSize << " | Col step size: " << cspaceColStepSize << std::endl;
    std::cout << "----------------------------------------------\n";
}

/// Grasping

bool GraspPlannerIK::plan(const Eigen::Matrix4f& targetPose, float& posError, float& oriError, float& planTime, std::string& error) {
    /// 1. IKSolver setup
    VirtualRobot::GenericIKSolverPtr ikSolver(new VirtualRobot::GenericIKSolver(rns));

    // set reachability
    if (useReachability && reachSpace) {
        ikSolver->setReachabilityCheck(reachSpace);
    }

    // set collision detection
    VirtualRobot::CDManagerPtr _cdm;
    
    if (useCollision) {
        _cdm = cdm;
    } else {
        _cdm.reset(new VirtualRobot::CDManager());
    }

    ikSolver->collisionDetection(_cdm);
    
    // set params
    ikSolver->setMaximumError(ikMaxErrorPos, ikMaxErrorOri);
    ikSolver->setupJacobian(ikJacobianStepSize, ikJacobianMaxLoops);

    VirtualRobot::IKSolver::CartesianSelection selection = useOnlyPosition ? VirtualRobot::IKSolver::Position : VirtualRobot::IKSolver::All;

    /// 2. Solve IK
    bool planOK = ikSolver->solve(targetPose, selection, ikMaxLoops);
    std::cout << "IK Solver success: " << planOK << std::endl;
    if (!planOK) {
        error = "ik_fail";
        return false;
    }

    Eigen::Matrix4f actPose = eef->getTcp()->getGlobalPose();
    poseError(actPose, targetPose, posError, oriError);

    std::cout << "IK Solver Error pos: " << posError << std::endl;
    std::cout << "IK Solver Error ori: " << oriError << std::endl;

    /// 3. Get Goal config and reset
    Eigen::VectorXf goalConfig;
    rns->getJointValues(goalConfig);
    reset();

    /// 4. BiRRT setup
    cspace.reset(new Saba::CSpaceSampled(robot, _cdm, rns, 1000000));
    cspace->setSamplingSize(cspacePathStepSize);
    cspace->setSamplingSizeDCD(cspaceColStepSize);

    Saba::BiRrtPtr rrt(new Saba::BiRrt(cspace, Saba::Rrt::eExtend, Saba::Rrt::eConnect));
    rrt->setStart(startConfig);
    rrt->setGoal(goalConfig);

    /// 5. Execute and postprocessing
    planOK = rrt->plan(true);
    std::cout << "BiRRT success: " << planOK << std::endl;
    planTime = rrt->getPlanningTimeMS();
    std::cout << "BiRRT time: " << planTime << " ms" << std::endl;
    if (!planOK) {
        error = "birrt_fail";
        return false;
    }

    birrtSolution = rrt->getSolution();
    Saba::ShortcutProcessorPtr postProcessing(new Saba::ShortcutProcessor(birrtSolution, cspace, false));
    birrtSolOptimized = postProcessing->optimize(optOptimzeStep);

    return true;
}

void GraspPlannerIK::closeEEF()
{
    contacts.clear();

    contacts = eef->closeActors(object);
}

void GraspPlannerIK::openEEF()
{
    contacts.clear();

    if (!params.eef_preshape.empty()) {
        eef->setPreshape(params.eef_preshape);
    }
    else
    {
        eef->openActors();
    }
}

GraspResult GraspPlannerIK::graspQuality() {
    if (contacts.size() > 0) {
        qualityMeasure->setContactPoints(contacts);

        float volume = qualityMeasure->getVolumeGraspMeasure();
        float epsilon = qualityMeasure->getGraspQuality();
        bool fc = qualityMeasure->isGraspForceClosure();

        return GraspResult(epsilon, volume, fc);;
    }

    std::cout << "GraspQuality: not contacts!!!\n";

    return GraspResult();
}

/// OTHERS

void GraspPlannerIK::reset() {
    openEEF();
    robot->setJointValues(rns, startConfig);
    openEEF();
}

}
