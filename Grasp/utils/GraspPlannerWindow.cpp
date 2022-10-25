#include "GraspPlannerWindow.h"

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

GraspPlannerWindow::GraspPlannerWindow(const GraspPlannerWindowParams& params)
: QMainWindow(nullptr), GraspPlanner(params.planner_params)
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

    best_grasps = params.best_grasps;

    buildVisu();

    viewer->viewAll();
}

GraspPlannerWindow::~GraspPlannerWindow()
{
    sceneSep->unref();
    graspsSep->unref();

    if (eefVisu)
    {
        eefVisu->unref();
    }
}

int GraspPlannerWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

GraspResult GraspPlannerWindow::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp) {
    GraspResult res = GraspPlanner::executeGrasp(xyz, rpy, save_grasp);

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

void GraspPlannerWindow::setupUI()
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
    connect(UI.checkBoxShowBestGrasps, SIGNAL(clicked()), this, SLOT(bestGraspsVisu()));
    connect(UI.checkBoxMove, SIGNAL(clicked()), this, nullptr);

    connect(UI.objSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
    connect(UI.objSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
    connect(UI.objSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
    connect(UI.objSliderRX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectRX()));
    connect(UI.objSliderRY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectRY()));
    connect(UI.objSliderRZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectRZ()));
}

void GraspPlannerWindow::buildVisu()
{

    robotSep->removeAllChildren();
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (eefCloned && !UI.checkBoxShowBestGrasps->isChecked())
    {
        visualizationRobot = eefCloned->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();

        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            // visualizationRobot->highlight(UI.checkBoxHighlight->isChecked());
        }
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
    Eigen::Vector3f r_pos = eefCloned->getGlobalPosition();
    Eigen::Matrix3f r_ori = eefCloned->getGlobalOrientation();
    
    std::stringstream ss;
    ss << std::setprecision(3);
    ss << modelPoseToStr(r_pos, r_ori);
    
    UI.robotInfo->setText(QString(ss.str().c_str()));

    buildBestGraspsSetVisu();

    viewer->scheduleRedraw();
}

void GraspPlannerWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

void GraspPlannerWindow::quit()
{
    std::cout << "GraspPlannerWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

/*void GraspPlannerWindow::plan()
{
    planGrasp();

    openEEF();
    closeEEF();
}
*/

void GraspPlannerWindow::add_grasp() {
    Eigen::Vector3f pos = eefCloned->getGlobalPosition();
    Eigen::Vector3f ori = eefCloned->getGlobalOrientation().eulerAngles(0, 1, 2);
    
    current_grasp = grasps.size();
    
    executeGrasp(pos, ori, true);
}

void GraspPlannerWindow::measure_quality()
{
    if (current_grasp >= 0 && current_grasp < grasps.size()) {
        Grasp::GraspData grasp = grasps[current_grasp];
        grasps[current_grasp].result = executeGrasp(grasp.pos, grasp.ori, false);
    } else {
        std::cout << "measure_quality: FIRST YOU HAVE TO SELECT A GRASP\n";
    }
}


void GraspPlannerWindow::previous_grasp() {
    if (current_grasp > 0) {
        current_grasp--;

        measure_quality();
    }
}

void GraspPlannerWindow::next_grasp() {
    if (current_grasp < grasps.size() - 1) {
        current_grasp++;

        measure_quality();
    }
}

void GraspPlannerWindow::closeEEF()
{
    closeEE();

    GraspResult res;
    std::stringstream ss;
    if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), eef->createSceneObjectSet())) {
        std::cout << "Error: Collision detected!" << std::endl;
        res = GraspResult("eef_collision");
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

void GraspPlannerWindow::openEEF()
{
    openEE();

    buildVisu();
}


void GraspPlannerWindow::frictionConeVisu()
{
    buildVisu();
}

void GraspPlannerWindow::bestGraspsVisu()
{
    if (best_grasps.size() == 0) {
        std::cout << "Error: There are not best grasps!!!\n";
        return;
    }

    if (best_grasps_visu.size() == 0) { // compute grasps
        Eigen::Vector3f pos = eefCloned->getGlobalPosition();
        Eigen::Vector3f ori = eefCloned->getGlobalOrientation().eulerAngles(0, 1, 2);

        for (int i = 0; i < best_grasps.size(); i++) {
            // move to grasp pose
            GraspResult res = executeGrasp(best_grasps[i].pos, best_grasps[i].ori, false);

            // save visualization
            VirtualRobot::CoinVisualizationPtr grasp_visu = eefCloned->clone()->getVisualization<CoinVisualization>(SceneObject::Full);
            best_grasps_visu.push_back(grasp_visu);
        }

        // reset to current grasp
        executeGrasp(pos, ori, false);
    }

    buildVisu();
}


void GraspPlannerWindow::colModel()
{
    buildVisu();
}
 
bool GraspPlannerWindow::evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::RobotPtr eefRobot, VirtualRobot::EndEffectorPtr eef, int nrEvalLoops, GraspEvaluationPoseUncertainty::PoseEvalResults& results)
{
    if (!g || !eefRobot || !eef)
    {
        return false;
    }

    GraspEvaluationPoseUncertaintyPtr eval(new GraspEvaluationPoseUncertainty(GraspEvaluationPoseUncertainty::PoseUncertaintyConfig()));

    results = eval->evaluateGrasp(g, eef, object, qualityMeasure, nrEvalLoops);

    return true;
}


void GraspPlannerWindow::sliderReleased_ObjectX()
{
    float v = (float)UI.objSliderX->value();

    UI.objSliderX->setValue(0);

    updateObj(v, CARTESIAN_VARS::TRANS_X);
}

void GraspPlannerWindow::sliderReleased_ObjectY()
{
    float v = (float)UI.objSliderY->value();

    UI.objSliderY->setValue(0);

    updateObj(v, CARTESIAN_VARS::TRANS_Y);
}

void GraspPlannerWindow::sliderReleased_ObjectZ()
{
    float v = (float)UI.objSliderZ->value();

    UI.objSliderZ->setValue(0);

    updateObj(v, CARTESIAN_VARS::TRANS_Z);
}

void GraspPlannerWindow::sliderReleased_ObjectRX()
{
    float v = (float)UI.objSliderRX->value();
    v /= 300;
    
    UI.objSliderRX->setValue(0);

    updateObj(v, CARTESIAN_VARS::ROT_ROLL);
}

void GraspPlannerWindow::sliderReleased_ObjectRY()
{
    float v = (float)UI.objSliderRY->value();
    v /= 300;

    UI.objSliderRY->setValue(0);

    updateObj(v, CARTESIAN_VARS::ROT_PITCH);
}

void GraspPlannerWindow::sliderReleased_ObjectRZ()
{
    float v = (float)UI.objSliderRZ->value();
    v /= 300;

    UI.objSliderRZ->setValue(0);

    updateObj(v, CARTESIAN_VARS::ROT_YAW);
}


void GraspPlannerWindow::updateObj(const float value, const int idx) {
    float x[6] = {0};
    x[idx] = value;

    
    Eigen::Matrix4f m;
    VirtualRobot::MathTools::posrpy2eigen4f(x, m);

    bool moveObj = (UI.checkBoxMove->isChecked());
    if (moveObj) {
        m = object->getGlobalPose() * m;
        object->setGlobalPose(m);
    } else {
        m = eefCloned->getGlobalPose() * m;
        eefCloned->setGlobalPose(m);
    }
    

    buildVisu();
}


void GraspPlannerWindow::buildBestGraspsSetVisu()
{
    graspsSep->removeAllChildren();

    if (UI.checkBoxShowBestGrasps->isChecked() && best_grasps.size() > 0)
    {   
        std::vector< std::vector<float> > colors;
        int i = 0;
        for (auto& grasp_visu : best_grasps_visu) {
            SoNode* visualisationNode = grasp_visu->getCoinVisualization();

            if (visualisationNode)
            {
                graspsSep->addChild(visualisationNode);

                float r = ((double) rand() / (RAND_MAX)) + 1;
                float g = ((double) rand() / (RAND_MAX)) + 1;
                float b = ((double) rand() / (RAND_MAX)) + 1;
                if (i > 0) {
                    bool ok = false; int it = 0;
                    do {
                        r = ((double) rand() / (RAND_MAX)) + 1;
                        g = ((double) rand() / (RAND_MAX)) + 1;
                        b = ((double) rand() / (RAND_MAX)) + 1;
                        for (int j = 0; j < i; j++) {
                            std::vector<float> c = colors[j];
                            float dist = (r-c[0]) * (r-c[0]);
                            dist += (g-c[1]) * (g-c[1]);
                            dist += (b-c[2]) * (b-c[2]);

                            dist = sqrt(dist);
                            if (dist > 0.4f) {
                                ok = true;
                                break;
                            }
                        }

                    } while(!ok && (++it) < 5);
                }

                std::vector<float> cv = {r,g,b};
                colors.push_back(cv);

                grasp_visu->colorize(VisualizationFactory::Color(r, g, b));
                i++;
            }
        }
    }
}
