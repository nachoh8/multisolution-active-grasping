#include "EigenGraspPlannerWindow.h"

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

#include "../include/Grasp/CoordSys.hpp"

using namespace std;
using namespace VirtualRobot;
using namespace GraspStudio;

using namespace Grasp;

EigenGraspPlannerWindow::EigenGraspPlannerWindow(const Grasp::EnvParameters& _planner_params, const std::vector<Grasp::GraspData>& _grasps, const std::vector<Grasp::GraspData>& _best_grasps)
: QMainWindow(nullptr), EigenGraspPlanner(_planner_params)
{
    srand((unsigned) time(0));

    verbose = true;

    VR_INFO << " start " << std::endl;

    scene_file = _planner_params.scene_file;
    eefName = _planner_params.eef;
    preshape = _planner_params.eef_preshape;

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

    TCP->showCoordinateSystem(true);

    object->showBoundingBox(true, true);
    object->showCoordinateSystem(true);
    
    // SET FIRST GRASP
    // Eigen::Vector3f pos;
    // pos << 0.0, 80.0, 0.0;
    // Eigen::Vector3f ori;
    // ori << -1.57, 0.0, 0.0;
    // executeGrasp(pos, ori, false);
    if (_grasps.size() > 0) {
        grasps = _grasps;
        current_grasp = 0;
        amplitude = grasps[current_grasp].eigen_amplitudes;
        executeGrasp(grasps[0].pos, grasps[0].ori, false);
    }
    
    best_grasps = _best_grasps;

    buildVisu();

    viewer->viewAll();
}

EigenGraspPlannerWindow::~EigenGraspPlannerWindow()
{
    sceneSep->unref();
    graspsSep->unref();

    if (eefVisu)
    {
        eefVisu->unref();
    }
}

int EigenGraspPlannerWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

GraspResult EigenGraspPlannerWindow::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp) {
    GraspResult res = EigenGraspPlanner::executeGrasp(xyz, rpy, save_grasp);

    std::stringstream ss;
    ss << std::setprecision(3);
    ss << "Grasp: " << (current_grasp+1) << "/" << grasps.size() << "\n"
        << "Quality:" << res.measure << std::endl
        << "Volume:" << res.volume << std::endl
        << "Force closure: " << (res.force_closure ? "yes" : "no") << std::endl
        << "Error: " << res.error << std::endl;

    UI.graspInfo->setText(QString(ss.str().c_str()));

    buildVisu();

    return res;
}

void EigenGraspPlannerWindow::setupUI()
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

    connect(UI.objSliderDOF, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectDOF()));
    connect(UI.objSliderDOFV, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectDOFV()));
    connect(UI.objSliderAmplitude, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectAmplitude()));
    connect(UI.objSliderAmplitudeV, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectAmplitudeV()));
    connect(UI.pushButtonResetAmplitude, SIGNAL(clicked()), this, SLOT(reset_amplitude()));
}

void EigenGraspPlannerWindow::buildVisu()
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
    Eigen::Vector3f o_pos;
    Eigen::Vector3f o_ori;
    poseMatrixToVec(object->getGlobalPose(), o_pos, o_ori);
    
    std::stringstream ss_o;
    ss_o << std::setprecision(3);
    ss_o  << "Name: " << object->getName() << "\n"
        << poseVecToStr(o_pos, o_ori);
    
    UI.objectInfo->setText(QString(ss_o.str().c_str()));

    // TCP info
    Eigen::Vector3f r_pos;
    Eigen::Vector3f r_ori;
    poseMatrixToVec(TCP->getGlobalPose(), r_pos, r_ori);
    
    std::stringstream ss;
    ss << std::setprecision(3);
    ss << poseVecToStr(r_pos, r_ori);
    ss << "Amplitude: (" << amplitude[0] << ", " << amplitude[1] << ")\n";
    
    UI.robotInfo->setText(QString(ss.str().c_str()));

    buildBestGraspsSetVisu();

    viewer->scheduleRedraw();
}

void EigenGraspPlannerWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

void EigenGraspPlannerWindow::quit()
{
    std::cout << "EigenGraspPlannerWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void EigenGraspPlannerWindow::add_grasp() {
    Eigen::Vector3f pos;
    Eigen::Vector3f ori;
    poseMatrixToVec(TCP->getGlobalPose(), pos, ori);
    
    current_grasp = grasps.size();
    
    executeGrasp(pos, ori, true);
}

void EigenGraspPlannerWindow::measure_quality()
{
    if (current_grasp >= 0 && current_grasp < grasps.size()) {
        Grasp::GraspData grasp = grasps[current_grasp];
        amplitude = grasp.eigen_amplitudes;
        grasps[current_grasp].result = executeGrasp(grasp.pos, grasp.ori, false);
    } else {
        std::cout << "measure_quality: FIRST YOU HAVE TO SELECT A GRASP\n";
    }
}


void EigenGraspPlannerWindow::previous_grasp() {
    if (current_grasp > 0) {
        current_grasp--;

        measure_quality();
    }
}

void EigenGraspPlannerWindow::next_grasp() {
    if (current_grasp < grasps.size() - 1) {
        current_grasp++;

        measure_quality();
    }
}

void EigenGraspPlannerWindow::closeEEF()
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

void EigenGraspPlannerWindow::openEEF()
{
    openEE();

    buildVisu();
}


void EigenGraspPlannerWindow::frictionConeVisu()
{
    buildVisu();
}

void EigenGraspPlannerWindow::bestGraspsVisu()
{
    if (best_grasps.size() == 0) {
        std::cout << "Error: There are not best grasps!!!\n";
        return;
    }

    if (best_grasps_visu.size() == 0) { // compute grasps
        Eigen::Vector3f pos;
        Eigen::Vector3f ori;
        Eigen::Vector2f _amplitudes = amplitude;
        poseMatrixToVec(TCP->getGlobalPose(), pos, ori); // save current position

        for (int i = 0; i < best_grasps.size(); i++) {
            // move to grasp pose
            amplitude = best_grasps[i].eigen_amplitudes;
            GraspResult res = executeGrasp(best_grasps[i].pos, best_grasps[i].ori, false);

            // save visualization
            VirtualRobot::CoinVisualizationPtr grasp_visu = eefCloned->clone()->getVisualization<CoinVisualization>(SceneObject::Full);
            best_grasps_visu.push_back(grasp_visu);
        }

        // reset to current grasp
        amplitude = _amplitudes;
        executeGrasp(pos, ori, false);
    }

    buildVisu();
}


void EigenGraspPlannerWindow::colModel()
{
    buildVisu();
}
 
bool EigenGraspPlannerWindow::evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::RobotPtr eefRobot, VirtualRobot::EndEffectorPtr eef, int nrEvalLoops, GraspEvaluationPoseUncertainty::PoseEvalResults& results)
{
    if (!g || !eefRobot || !eef)
    {
        return false;
    }

    GraspEvaluationPoseUncertaintyPtr eval(new GraspEvaluationPoseUncertainty(GraspEvaluationPoseUncertainty::PoseUncertaintyConfig()));

    results = eval->evaluateGrasp(g, eef, object, qualityMeasure, nrEvalLoops);

    return true;
}


void EigenGraspPlannerWindow::sliderReleased_ObjectDOF()
{
    current_dof = UI.objSliderDOF->value() - 1;
    std::cout << "Set actuator to: " << current_dof+1 << std::endl;
}

void EigenGraspPlannerWindow::sliderReleased_ObjectDOFV()
{
    float v = (float)UI.objSliderDOFV->value() / 100.0f;

    UI.objSliderDOFV->setValue(0);
    
    std::cout << "Actuator: " << current_dof+1 << std::endl;
    std::cout << "Angle: " << v << std::endl;
    std::cout << "Limits reached: " << eef->getActors()[current_dof]->moveActor(eigengrasps[0][current_dof]) << std::endl;
}

void EigenGraspPlannerWindow::sliderReleased_ObjectAmplitude()
{
    current_amplitude = UI.objSliderAmplitude->value() - 1;
    UI.objSliderAmplitudeV->setValue(int(amplitude[current_amplitude] * 100));
    std::cout << "Set Amplitude to: " << current_amplitude+1 << std::endl;
}

void EigenGraspPlannerWindow::sliderReleased_ObjectAmplitudeV()
{
    float v = (float)UI.objSliderAmplitudeV->value() / 100;

    std::cout << "Amplitude: " << current_amplitude+1 << std::endl;
    amplitude[current_amplitude] = v;
    std::cout << "Alpha: " << amplitude << std::endl;

    Eigen::VectorXf res = eigengrasps[0] * amplitude[0] + eigengrasps[1] * amplitude[1];
    setPreshape(res);
}


void EigenGraspPlannerWindow::reset_amplitude() {
    amplitude << 0.0, 0.0;
    UI.objSliderAmplitudeV->setValue(0);
}

void EigenGraspPlannerWindow::buildBestGraspsSetVisu()
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