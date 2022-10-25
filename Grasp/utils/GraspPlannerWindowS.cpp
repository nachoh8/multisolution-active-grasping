#include "GraspPlannerWindowS.h"

#include "GraspPlanning/Visualization/CoinVisualization/CoinConvexHullVisualization.h"
#include "GraspPlanning/ContactConeGenerator.h"
#include "GraspPlanning/MeshConverter.h"
#include <GraspPlanning/GraspQuality/GraspEvaluationPoseUncertainty.h>

#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "VirtualRobot/Visualization/TriMeshModel.h"
#include <VirtualRobot/Visualization/VisualizationNode.h>

#include <QFileDialog>
#include <QProgressDialog>

#include <Eigen/Geometry>

#include <ctime>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <random>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoUnits.h>

#include "../include/Grasp/GraspVars.hpp"

using namespace std;
using namespace VirtualRobot;
using namespace GraspStudio;

using namespace Grasp;

GraspPlannerWindowS::GraspPlannerWindowS(const GraspPlannerWindowSParams& params)
: QMainWindow(nullptr), GraspPlannerS(params.planner_params)
{
    srand((unsigned) time(0));

    VR_INFO << " start " << std::endl;

    robotFile = params.planner_params.robot_file;
    eefName = params.planner_params.eef_name;
    preshape = params.planner_params.preshape;

    eefVisu = nullptr;

    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotSep = new SoSeparator;
    objectSep = new SoSeparator;
    frictionConeSep = new SoSeparator;
    graspsSep = new SoSeparator;
    graspsSep->ref();

    sceneSep->addChild(robotSep);
    sceneSep->addChild(objectSep);
    sceneSep->addChild(frictionConeSep);
    sceneSep->addChild(graspsSep);

    /// UI
    setupUI();

    eefVisu = CoinVisualizationFactory::CreateEndEffectorVisualization(eef);
    eefVisu->ref();

    if (params.grasps.size() > 0) {
        grasps = params.grasps;
        current_grasp = 0;
        executeGrasp(grasps[0].pos, grasps[0].ori, false);
    }

    //best_grasps = params.best_grasps;

    buildVisu();

    viewer->viewAll();
}

GraspPlannerWindowS::~GraspPlannerWindowS()
{
    sceneSep->unref();
    graspsSep->unref();

    if (eefVisu)
    {
        eefVisu->unref();
    }
}

int GraspPlannerWindowS::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

GraspResult GraspPlannerWindowS::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp) {
    GraspResult res = GraspPlannerS::executeGrasp(xyz, rpy, save_grasp);

    std::stringstream ss;
    ss << std::setprecision(3);
    ss << "Grasp: " << (current_grasp+1) << "/" << grasps.size() << "\n"
        << "Quality:" << res.measure << std::endl
        << "Volume:" << res.volume << std::endl
        << "Force closure: " << (res.force_closure ? "yes" : "no") << std::endl;

    UI.graspInfo->setText(QString(ss.str().c_str()));

    buildVisu();

    return res;
}

void GraspPlannerWindowS::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(false);
#ifdef WIN32
    viewer->setAntialiasing(true, 8);
#endif
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    connect(UI.pushButtonAddGrasp, SIGNAL(clicked()), this, SLOT(add_grasp()));
    connect(UI.pushButtonQuality, SIGNAL(clicked()), this, SLOT(measure_quality()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));

    connect(UI.pushButtonPrevGrasp, SIGNAL(clicked()), this, SLOT(previous_grasp()));
    connect(UI.pushButtonNextGrasp, SIGNAL(clicked()), this, SLOT(next_grasp()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxCones, SIGNAL(clicked()), this, SLOT(frictionConeVisu()));
    //connect(UI.checkBoxShowBestGrasps, SIGNAL(clicked()), this, SLOT(bestGraspsVisu()));
    connect(UI.checkBoxMove, SIGNAL(clicked()), this, nullptr);

    connect(UI.objSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
    connect(UI.objSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
    connect(UI.objSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
    connect(UI.objSliderRX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectRX()));
    connect(UI.objSliderRY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectRY()));
    connect(UI.objSliderRZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectRZ()));
}

void GraspPlannerWindowS::buildVisu()
{

    robotSep->removeAllChildren();
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (eefCloned)
    {
        visualizationRobot = eefCloned->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();

        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            // visualizationRobot->highlight(UI.checkBoxHighlight->isChecked());
        }
    }

    if (TCP)
    {
        TCP->showCoordinateSystem(true);
    }

    /*
    if (robot)
    {
        visualizationRobot = robot->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();
        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            //visualizationRobot->highlight(true);
        }
    }
    */
    objectSep->removeAllChildren();

    if (object)
    {

#if 1
        SceneObject::VisualizationType colModel2 = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, colModel2);

        if (visualisationNode)
        {
            objectSep->addChild(visualisationNode);
        }

#else
        if (UI.checkBoxColModel->isChecked())
        {
            VirtualRobot::MathTools::ConvexHull3DPtr ch = ConvexHullGenerator::CreateConvexHull(object->getCollisionModel()->getTriMeshModel());
            CoinConvexHullVisualizationPtr chv(new CoinConvexHullVisualization(ch));
            SoSeparator* s = chv->getCoinVisualization();
            if (s)
            {
                objectSep->addChild(s);
            }
        }
        else
        {
            SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, SceneObject::Full);
            if (visualisationNode)
            {
                objectSep->addChild(visualisationNode);
            }
        }
#endif
        /*SoNode *s = CoinVisualizationFactory::getCoinVisualization(object->getCollisionModel()->getTriMeshModel(),true);
        if (s)
            objectSep->addChild(s);   */
    }

    frictionConeSep->removeAllChildren();
    bool fc = (UI.checkBoxCones->isChecked());

    if (fc && contacts.size() > 0 && qualityMeasure)
    {
        ContactConeGeneratorPtr cg = qualityMeasure->getConeGenerator();
        float radius = cg->getConeRadius();
        float height = cg->getConeHeight();
        float scaling = 30.0f;
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(contacts, height * scaling, radius * scaling, true);

        if (visualisationNode)
        {
            frictionConeSep->addChild(visualisationNode);
        }

        // add approach dir visu
        for (auto& contact : contacts)
        {
            SoSeparator* s = new SoSeparator;
            Eigen::Matrix4f ma;
            ma.setIdentity();
            ma.block(0, 3, 3, 1) = contact.contactPointFingerGlobal;
            SoMatrixTransform* m = CoinVisualizationFactory::getMatrixTransformScaleMM2M(ma);
            s->addChild(m);
            s->addChild(CoinVisualizationFactory::CreateArrow(contact.approachDirectionGlobal, 10.0f, 1.0f));
            frictionConeSep->addChild(s);
        }
    }


    // Object info
    Eigen::Vector3f o_pos = object->getGlobalPosition();
    Eigen::Matrix3f o_ori = object->getGlobalOrientation();
    
    std::stringstream ss_o;
    ss_o << std::setprecision(3);
    ss_o  << "Name: " << object->getName() << "\n"
        << modelPoseToStr(o_pos, o_ori);
    
    UI.objectInfo->setText(QString(ss_o.str().c_str()));

    // Robot info
    Eigen::Vector3f xyz = TCP->getGlobalPosition();

    xyz.z() += 100;

    VirtualRobot::MathTools::SphericalCoord r_pos_sc = VirtualRobot::MathTools::toSphericalCoords(xyz);
    Eigen::Vector3f r_pos;
    r_pos.x() = r_pos_sc.theta;
    r_pos.y() = r_pos_sc.phi;
    r_pos.z() = r_pos_sc.r;

    Eigen::Matrix3f r_ori = TCP->getGlobalOrientation();
    Eigen::Matrix4f m_ori = Eigen::Matrix4f::Identity(4,4);
    m_ori.block(0,0,3,3) = r_ori;

    Eigen::Vector3f comprobacion_rpy;

    VirtualRobot::MathTools::eigen4f2rpy(m_ori, comprobacion_rpy);

    std::cout << "Comprobacion RPY" << comprobacion_rpy << std::endl;
    
    std::stringstream ss;
    ss << std::setprecision(3);
    ss << modelPoseToStr(r_pos, r_ori);
    
    UI.robotInfo->setText(QString(ss.str().c_str()));

    //buildBestGraspsSetVisu();

    viewer->scheduleRedraw();
}

void GraspPlannerWindowS::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

void GraspPlannerWindowS::quit()
{
    std::cout << "GraspPlannerWindowS: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

/*void GraspPlannerWindowS::plan()
{
    planGrasp();
    openEEF();
    closeEEF();
}
*/

void GraspPlannerWindowS::add_grasp() {
    Eigen::Vector3f pos = TCP->getGlobalPosition();
    Eigen::Vector3f ori = TCP->getGlobalOrientation().eulerAngles(0, 1, 2);
    
    current_grasp = grasps.size();
    
    executeGrasp(pos, ori, true);
}

void GraspPlannerWindowS::measure_quality()
{
    if (current_grasp >= 0 && current_grasp < grasps.size()) {
        Grasp::GraspData grasp = grasps[current_grasp];
        VirtualRobot::MathTools::SphericalCoord scoords;
        std::cout << "SPHERICAL COORDS" << std::endl;
        std::cout << grasp.pos << std::endl;
        scoords.theta = grasp.pos.x();
        scoords.phi = grasp.pos.y();
        scoords.r = grasp.pos.z();
        Eigen::Vector3f pos = VirtualRobot::MathTools::toPosition(scoords);
        
        pos.z() += -100;

        std::cout << "CARTESIAN COORDS" << std::endl;
        std::cout << pos << std::endl;
        std::cout << "ORIENTATION" << std::endl;
        std::cout << grasp.ori << std::endl;
        //pos = {0,0,400};
        grasps[current_grasp].result = executeGrasp(pos, grasp.ori, false);
    } else {
        std::cout << "measure_quality: FIRST YOU HAVE TO SELECT A GRASP\n";
    }
}


void GraspPlannerWindowS::previous_grasp() {
    if (current_grasp > 0) {
        current_grasp--;

        measure_quality();
    }
}

void GraspPlannerWindowS::next_grasp() {
    if (current_grasp < grasps.size() - 1) {
        current_grasp++;

        measure_quality();
    }
}

void GraspPlannerWindowS::closeEEF()
{
    closeEE();

    GraspResult res;
    std::stringstream ss;
    if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), eef->createSceneObjectSet())) {
        std::cout << "Error: Collision detected!" << std::endl;
        res = GraspResult(comp_rho, comp_roll, comp_pitch, comp_yaw);
        ss << "Grasp Nr " << grasps.size() << "\nCollision detected\n";
    } else {
        res = graspQuality();
        ss << std::setprecision(3);
        ss << "Grasp Nr " << grasps.size() << "\nQuality: " << res.measure << "\nForce closure: ";

        if (res.force_closure)
        {
            ss << "yes";
        }
        else
        {
            ss << "no";
        }
    }

    UI.graspInfo->setText(QString(ss.str().c_str()));

    buildVisu();
}

void GraspPlannerWindowS::openEEF()
{
    openEE();

    buildVisu();
}


void GraspPlannerWindowS::frictionConeVisu()
{
    buildVisu();
}

void GraspPlannerWindowS::colModel()
{
    buildVisu();
}
 
bool GraspPlannerWindowS::evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::RobotPtr eefRobot, VirtualRobot::EndEffectorPtr eef, int nrEvalLoops, GraspEvaluationPoseUncertainty::PoseEvalResults& results)
{
    if (!g || !eefRobot || !eef)
    {
        return false;
    }

    GraspEvaluationPoseUncertaintyPtr eval(new GraspEvaluationPoseUncertainty(GraspEvaluationPoseUncertainty::PoseUncertaintyConfig()));

    results = eval->evaluateGrasp(g, eef, object, qualityMeasure, nrEvalLoops);

    return true;
}


void GraspPlannerWindowS::sliderReleased_ObjectX()
{
    float v = (float)UI.objSliderX->value();
    v /= 100;

    UI.objSliderX->setValue(0);

    updateObj(v, SPHERICAL_VARS::TRANS_THETA);
}

void GraspPlannerWindowS::sliderReleased_ObjectY()
{
    float v = (float)UI.objSliderY->value();
    v /= 100;

    UI.objSliderY->setValue(0);

    //updateObj(v, SPHERICAL_VARS::TRANS_Y);
    updateObj(v, SPHERICAL_VARS::TRANS_PHI);
}

void GraspPlannerWindowS::sliderReleased_ObjectZ()
{
    float v = (float)UI.objSliderZ->value();

    UI.objSliderZ->setValue(0);

    //updateObj(v, SPHERICAL_VARS::TRANS_Z)
    updateObj(v, SPHERICAL_VARS::TRANS_RHO);;
}

void GraspPlannerWindowS::sliderReleased_ObjectRX()
{
    float v = (float)UI.objSliderRX->value();
    v /= 300;
    
    UI.objSliderRX->setValue(0);

    updateObj(v, SPHERICAL_VARS::R_ROLL);
}

void GraspPlannerWindowS::sliderReleased_ObjectRY()
{
    float v = (float)UI.objSliderRY->value();
    v /= 300;

    UI.objSliderRY->setValue(0);

    updateObj(v, SPHERICAL_VARS::R_PITCH);
}

void GraspPlannerWindowS::sliderReleased_ObjectRZ()
{
    float v = (float)UI.objSliderRZ->value();
    v /= 300;

    UI.objSliderRZ->setValue(0);

    updateObj(v, SPHERICAL_VARS::R_YAW);
}


void GraspPlannerWindowS::updateObj(const float value, const int idx) {
    float x[6] = {0};
    x[idx] = value;

    VirtualRobot::MathTools::SphericalCoord scoords;
    Eigen::Vector3f position;
    Eigen::Vector3f new_position;
    Eigen::Matrix4f m;
    VirtualRobot::MathTools::posrpy2eigen4f(x, m);

    bool moveObj = (UI.checkBoxMove->isChecked());
    if (moveObj) {
        m = object->getGlobalPose() * m;
        object->setGlobalPose(m);
    } else {
        if (x[0]!=0 || x[1]!=0 || x[2]!=0){
            position = TCP->getGlobalPosition();
            scoords = VirtualRobot::MathTools::toSphericalCoords(position);
            scoords.r = scoords.r + x[2];
            scoords.theta = scoords.theta + x[0];
            scoords.phi = scoords.phi + x[1];
            new_position = VirtualRobot::MathTools::toPosition(scoords);

            new_position.z() += -100;

            m.block(0, 3, 3, 1) = new_position;
            eefCloned->setGlobalPoseForRobotNode(TCP, m);
        }
        else if (x[3]!=0 || x[4]!=0 || x[5]!=0)
        {
            m = TCP->getGlobalPose() * m;
            eefCloned->setGlobalPoseForRobotNode(TCP, m);
        }   
    }
    

    buildVisu();
}