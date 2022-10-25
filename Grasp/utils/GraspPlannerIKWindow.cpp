
#include "GraspPlannerIKWindow.h"
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
#include "MotionPlanning/Planner/GraspIkRrt.h"
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/PostProcessing/ShortcutProcessor.h"
#include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>
#include <QFileDialog>
#include <Eigen/Geometry>

#include <ctime>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <iomanip>
#include <random>

#include <QImage>
#include <QGLWidget>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

/// INIT

GraspPlannerIKWindow::GraspPlannerIKWindow(const GraspPlannerIKWindowParams& params)
    : QMainWindow(nullptr), GraspPlannerIK(params.planner_params)
{
    srand((unsigned) time(0));

    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotSep = new SoSeparator;
    objectSep = new SoSeparator;
    graspsSep = new SoSeparator;
    reachableGraspsSep = new SoSeparator;
    reachabilitySep = new SoSeparator;
    obstaclesSep = new SoSeparator;
    rrtSep = new SoSeparator;

    playbackMode = false;

    sceneSep->addChild(robotSep);
    sceneSep->addChild(objectSep);
    sceneSep->addChild(graspsSep);
    graspsSep->ref();
    sceneSep->addChild(reachableGraspsSep);
    sceneSep->addChild(reachabilitySep);
    sceneSep->addChild(obstaclesSep);
    sceneSep->addChild(rrtSep);

    setupUI();

    UI.checkBoxColCheckIK->setChecked(useCollision);
    UI.checkBoxReachabilitySpaceIK->setChecked(useReachability);
    UI.checkBoxOnlyPosition->setChecked(useOnlyPosition);

    UI.doubleSpinBoxCSpaceColStepSize->setValue(cspaceColStepSize);
    UI.doubleSpinBoxCSpacePathStepSize->setValue(cspacePathStepSize);
    UI.doubleSpinBoxIKMaxErrorPos->setValue(ikMaxErrorPos);
    UI.doubleSpinBoxIKMaxErrorOri->setValue(ikMaxErrorOri);
    UI.doubleSpinBoxIKJacobianStepSize->setValue(ikJacobianStepSize);
    UI.doubleSpinBoxIKJacobianMaxLoops->setValue(ikJacobianMaxLoops);


    targetPoseBox = Obstacle::createBox(30.0f, 30.0f, 30.0f);
    targetPoseBox->showCoordinateSystem(true);
    targetPoseBoxSep = new SoSeparator();
    targetPoseBoxSep->addChild(CoinVisualization(targetPoseBox->getVisualization()).getCoinVisualization());
    sceneSep->addChild(targetPoseBoxSep);

    if (params.grasps.size() > 0) {
        grasps = params.grasps;
        current_grasp = 0;
        updateTargetGrasp();
    } else {
        box2TCP();
    }

    best_grasps = params.best_grasps;
    best_grasps_full_robot = false;

    buildVisu();

    viewer->viewAll();

    SoSensorManager* sensor_mgr = SoDB::getSensorManager();
    SoTimerSensor* timer = new SoTimerSensor(timerCB, this);
    timer->setInterval(SbTime(TIMER_MS / 1000.0f));
    sensor_mgr->insertTimerSensor(timer);
}

/// EEF

void GraspPlannerIKWindow::closeEEF()
{
    GraspPlannerIK::closeEEF();

    redraw();

}

void GraspPlannerIKWindow::openEEF()
{
    GraspPlannerIK::openEEF();

    redraw();

}

void GraspPlannerIKWindow::closeEEFbtn()
{
    closeEEF();

    Grasp::GraspResult result = graspQuality();

    if (result.error != "") {
        std::cout << "Grasp error:" << result.error << std::endl;
    } else {
        std::cout << "Grasp Quality (epsilon measure):" << result.measure << std::endl;
        std::cout << "v measure:" << result.volume << std::endl;
        std::cout << "Force closure: " << (result.force_closure ? "yes" : "no") << std::endl;
    }
    
}

void GraspPlannerIKWindow::openEEFbtn()
{
    openEEF();
}

/// UI

GraspPlannerIKWindow::~GraspPlannerIKWindow()
{
    sceneSep->unref();
}

void GraspPlannerIKWindow::previous_grasp() {
    if (current_grasp > 0) {
        current_grasp--;

        updateTargetGrasp();
    }
}

void GraspPlannerIKWindow::next_grasp() {
    if (current_grasp < grasps.size() - 1) {
        current_grasp++;

        updateTargetGrasp();
    }
}

void GraspPlannerIKWindow::saveNewGrasp() {
    Eigen::Matrix4f targetPose = targetPoseBox->getGlobalPose();
    Eigen::Vector3f xyz = targetPose.block<3,1>(0,3);
    Eigen::Vector3f rpy = VirtualRobot::MathTools::eigen4f2rpy(targetPose);
    
    std::cout << "--------CONFIG--------\n";
    std::cout << "Target Pose: ("
                << xyz.transpose() << ", "
                << rpy.transpose() << ")" << std::endl;
    
    useCollision = UI.checkBoxColCheckIK->isChecked();
    useReachability = UI.checkBoxReachabilitySpaceIK->isChecked();
    useOnlyPosition = UI.checkBoxOnlyPosition->isChecked();

    cspaceColStepSize = (float)UI.doubleSpinBoxCSpaceColStepSize->value();
    cspacePathStepSize = (float)UI.doubleSpinBoxCSpacePathStepSize->value();
    ikMaxErrorPos = (float)UI.doubleSpinBoxIKMaxErrorPos->value();
    ikMaxErrorOri = (float)UI.doubleSpinBoxIKMaxErrorOri->value();
    ikJacobianStepSize = (float)UI.doubleSpinBoxIKJacobianStepSize->value();
    ikJacobianMaxLoops = (int)UI.doubleSpinBoxIKJacobianMaxLoops->value();

    printInfo();

    Grasp::GraspData grasp;
    grasp.pos = xyz;
    grasp.ori = rpy;
    grasp.result = executeGrasp(targetPose);

    current_grasp = grasps.size();
    grasps.push_back(grasp);
    updateTargetGrasp();
}

void GraspPlannerIKWindow::timerCB(void* data, SoSensor* /*sensor*/)
{
    GraspPlannerIKWindow* ikWindow = static_cast<GraspPlannerIKWindow*>(data);
    float x[6];
    x[0] = (float)ikWindow->UI.horizontalSliderX->value();
    x[1] = (float)ikWindow->UI.horizontalSliderY->value();
    x[2] = (float)ikWindow->UI.horizontalSliderZ->value();
    x[3] = (float)ikWindow->UI.horizontalSliderRo->value();
    x[4] = (float)ikWindow->UI.horizontalSliderPi->value();
    x[5] = (float)ikWindow->UI.horizontalSliderYa->value();
    x[0] /= 10.0f;
    x[1] /= 10.0f;
    x[2] /= 10.0f;
    x[3] /= 300.0f;
    x[4] /= 300.0f;
    x[5] /= 300.0f;

    if (x[0] != 0 || x[1] != 0 || x[2] != 0 || x[3] != 0 || x[4] != 0 || x[5] != 0)
    {
        ikWindow->updateObject(x);
        ikWindow->redraw();
    }
}


void GraspPlannerIKWindow::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(false);
    viewer->setAntialiasing(true, 4);
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(false);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEFbtn()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEFbtn()));

    connect(UI.checkBoxSolution, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxOnlyPosition, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxTCP, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxBestGrasps, SIGNAL(clicked()), this, SLOT(bestGraspsVisu()));
    connect(UI.checkBoxFullRobotBestGrasps, SIGNAL(clicked()), this, SLOT(bestGraspsVisu()));
    connect(UI.checkBoxReachabilitySpace, SIGNAL(clicked()), this, SLOT(reachVisu()));
    connect(UI.pushButtonIKRRT, SIGNAL(clicked()), this, SLOT(planIKRRT()));

    connect(UI.pushButtonPrevGrasp, SIGNAL(clicked()), this, SLOT(previous_grasp()));
    connect(UI.pushButtonNextGrasp, SIGNAL(clicked()), this, SLOT(next_grasp()));
    connect(UI.saveNewGrasp, SIGNAL(clicked()), this, SLOT(saveNewGrasp()));

    connect(UI.horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
    connect(UI.horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
    connect(UI.horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
    connect(UI.horizontalSliderRo, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectA()));
    connect(UI.horizontalSliderPi, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectB()));
    connect(UI.horizontalSliderYa, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectG()));
    connect(UI.horizontalSliderSolution, SIGNAL(valueChanged(int)), this, SLOT(sliderSolution(int)));

    UI.checkBoxColCheckIK->setChecked(false);
    UI.checkBoxReachabilitySpaceIK->setChecked(false);

}

QString GraspPlannerIKWindow::formatString(const char* s, float f)
{
    QString str1(s);

    if (f >= 0)
    {
        str1 += " ";
    }

    if (fabs(f) < 1000)
    {
        str1 += " ";
    }

    if (fabs(f) < 100)
    {
        str1 += " ";
    }

    if (fabs(f) < 10)
    {
        str1 += " ";
    }

    QString str1n;
    str1n.setNum(f, 'f', 3);
    str1 = str1 + str1n;
    return str1;
}


void GraspPlannerIKWindow::resetSceneryAll()
{
    GraspPlannerIK::reset();
}


void GraspPlannerIKWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

void GraspPlannerIKWindow::buildVisu()
{
    showCoordSystem();

    robotSep->removeAllChildren();
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (!UI.checkBoxBestGrasps->isChecked() && robot)
    {
        visualizationRobot = robot->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();

        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            //visualizationRobot->highlight(true);
        }
    }

    targetPoseBoxSep->removeAllChildren();
    if (!UI.checkBoxBestGrasps->isChecked() && targetPoseBox) {
        targetPoseBoxSep->addChild(CoinVisualization(targetPoseBox->getVisualization()).getCoinVisualization());
    }

    objectSep->removeAllChildren();

    if (object)
    {
        SceneObject::VisualizationType colModel2 = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, colModel2);

        if (visualisationNode)
        {
            objectSep->addChild(visualisationNode);
        }
    }

    obstaclesSep->removeAllChildren();
    
    if (obstacles.size() > 0)
    {
        for (const auto & obstacle : obstacles)
        {
            SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(obstacle, colModel);

            if (visualisationNode)
            {
                obstaclesSep->addChild(visualisationNode);
            }
        }
    }

    Eigen::Matrix4f targetPose = targetPoseBox->getGlobalPose();
    Eigen::Vector3f xyz = targetPose.block<3,1>(0,3);
    Eigen::Vector3f rpy = VirtualRobot::MathTools::eigen4f2rpy(targetPose);

    std::stringstream ss_o;
    ss_o << std::setprecision(3);
    ss_o << "Target Pose:\n"
            << xyz.x() << ", " << xyz.y() << ", " << xyz.z() << "\n"
            << rpy.x() << ", " << rpy.y() << ", " << rpy.z();
    
    UI.targetPose->setText(QString(ss_o.str().c_str()));

    // buildGraspSetVisu();
    buildBestGraspsSetVisu();

    buildRRTVisu();

    redraw();
}

int GraspPlannerIKWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void GraspPlannerIKWindow::quit()
{
    std::cout << "IKRRTWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}


void GraspPlannerIKWindow::updateObject(float x[6])
{
    Eigen::Matrix4f m;
    MathTools::posrpy2eigen4f(x, m);

    m = targetPoseBox->getGlobalPose() * m;
    targetPoseBox->setGlobalPose(m);

    redraw();

}

void GraspPlannerIKWindow::updateTargetGrasp() {
    reset();
    
    Grasp::GraspData grasp = grasps[current_grasp];
    float x[6];
    x[0] = grasp.pos.x();
    x[1] = grasp.pos.y();
    x[2] = grasp.pos.z();
    x[3] = grasp.ori.x();
    x[4] = grasp.ori.y();
    x[5] = grasp.ori.z();

    Eigen::Matrix4f targetPose;
    VirtualRobot::MathTools::posrpy2eigen4f(x, targetPose);

    targetPoseBox->setGlobalPose(targetPose);

    std::stringstream ss;
    ss << std::setprecision(3);
    ss << "Grasp: " << (current_grasp+1) << "/" << grasps.size() << "\n";
    if (grasp.result.error != "") {
        ss << "Error: " << grasp.result.error << std::endl;
    } else {
        ss << "Quality:" << grasp.result.measure << std::endl
            << "Volume:" << grasp.result.volume << std::endl
            << "Force closure: " << (grasp.result.force_closure ? "yes" : "no") << std::endl;
    }

    UI.graspInfo->setText(QString(ss.str().c_str()));

    buildVisu();
}

void GraspPlannerIKWindow::sliderReleased_ObjectX()
{
    UI.horizontalSliderX->setValue(0);
    buildVisu();
}

void GraspPlannerIKWindow::sliderReleased_ObjectY()
{
    UI.horizontalSliderY->setValue(0);
    buildVisu();
}

void GraspPlannerIKWindow::sliderReleased_ObjectZ()
{
    UI.horizontalSliderZ->setValue(0);
    buildVisu();
}

void GraspPlannerIKWindow::sliderReleased_ObjectA()
{
    UI.horizontalSliderRo->setValue(0);
    buildVisu();
}

void GraspPlannerIKWindow::sliderReleased_ObjectB()
{
    UI.horizontalSliderPi->setValue(0);
    buildVisu();
}

void GraspPlannerIKWindow::sliderReleased_ObjectG()
{
    UI.horizontalSliderYa->setValue(0);
    buildVisu();
}

void GraspPlannerIKWindow::showCoordSystem()
{
    if (eef)
    {
        RobotNodePtr tcp = eef->getTcp();

        if (!tcp)
        {
            return;
        }

        tcp->showCoordinateSystem(UI.checkBoxTCP->isChecked());
    }

    if (object)
    {
        object->showCoordinateSystem(UI.checkBoxTCP->isChecked());
    }
}

void GraspPlannerIKWindow::buildRRTVisu()
{
    rrtSep->removeAllChildren();

    if (!UI.checkBoxSolution->isChecked())
    {
        return;
    }

    if (!birrtSolution)
    {
        return;
    }

    std::shared_ptr<Saba::CoinRrtWorkspaceVisualization> w(new Saba::CoinRrtWorkspaceVisualization(robot, cspace, eef->getTcpName()));

    if (birrtSolOptimized)
    {
        w->addCSpacePath(birrtSolOptimized, Saba::CoinRrtWorkspaceVisualization::eGreen);
    }

    //w->addConfiguration(startConfig,Saba::CoinRrtWorkspaceVisualization::eGreen,3.0f);
    SoSeparator* sol = w->getCoinVisualization();
    rrtSep->addChild(sol);
}

/*void GraspPlannerIKWindow::buildGraspSetVisu()
{
    graspsSep->removeAllChildren();

    if (UI.checkBoxGraspSet->isChecked() && eef && graspSet && object)
    {
        SoSeparator* visu = CoinVisualizationFactory::CreateGraspSetVisualization(graspSet, eef, object->getGlobalPose());

        if (visu)
        {
            graspsSep->addChild(visu);
        }
    }

    // show reachable graps
    reachableGraspsSep->removeAllChildren();

    if (UI.checkBoxReachableGrasps->isChecked() && eef && graspSet && object && reachSpace)
    {
        GraspSetPtr rg = reachSpace->getReachableGrasps(graspSet, object);

        if (rg->getSize() > 0)
        {
            SoSeparator* visu = CoinVisualizationFactory::CreateGraspSetVisualization(rg, eef, object->getGlobalPose());

            if (visu)
            {
                reachableGraspsSep->addChild(visu);
            }
        }
    }
}*/

void GraspPlannerIKWindow::bestGraspsVisu()
{
    if (best_grasps.size() == 0) {
        std::cout << "Error: There are not best grasps!!!\n";
        return;
    }

    if (best_grasps_visu.size() == 0 || best_grasps_full_robot != UI.checkBoxFullRobotBestGrasps->isChecked()) { // compute grasps
        best_grasps_full_robot = UI.checkBoxFullRobotBestGrasps->isChecked();
        best_grasps_visu.clear();
        for (int i = 0; i < best_grasps.size(); i++) {
            std::cout << "****Computing best grasp " << (i+1) << "/" << best_grasps.size() << "****\n";

            float x[6];
            x[0] = best_grasps[i].pos.x();
            x[1] = best_grasps[i].pos.y();
            x[2] = best_grasps[i].pos.z();
            x[3] = best_grasps[i].ori.x();
            x[4] = best_grasps[i].ori.y();
            x[5] = best_grasps[i].ori.z();

            Eigen::Matrix4f targetPose;
            VirtualRobot::MathTools::posrpy2eigen4f(x, targetPose);

            /// move to grasp pose
            Grasp::GraspResult res = executeGrasp(targetPose);

            /// save visualization
            VirtualRobot::CoinVisualizationPtr grasp_visu;
            if (best_grasps_full_robot) {
                grasp_visu = robot->clone()->getVisualization<CoinVisualization>(SceneObject::Full);
            } else {
                VirtualRobot::RobotPtr r = eef->createEefRobot(eef->getName(), eef->getName());
                VirtualRobot::RobotNodePtr tcpN = r->getEndEffector(eef->getName())->getTcp();
                r->setGlobalPoseForRobotNode(tcpN, targetPose);
                
                grasp_visu = r->getVisualization<CoinVisualization>(SceneObject::Full);
            }

            best_grasps_visu.push_back(grasp_visu);
        }

        // reset to start position
        reset();
    }

    buildVisu();
}

void GraspPlannerIKWindow::buildBestGraspsSetVisu()
{
    graspsSep->removeAllChildren();

    if (UI.checkBoxBestGrasps->isChecked() && best_grasps.size() > 0)
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

void GraspPlannerIKWindow::reachVisu()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    reachabilitySep->removeAllChildren();

    if (UI.checkBoxReachabilitySpace->checkState() == Qt::Checked)
    {
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(reachSpace, VirtualRobot::ColorMap::eRed, true);

        if (visualisationNode)
        {
            reachabilitySep->addChild(visualisationNode);
        }
    }
}

void GraspPlannerIKWindow::planIKRRT()
{
    Eigen::Matrix4f targetPose = targetPoseBox->getGlobalPose();
    Eigen::Vector3f xyz = targetPose.block<3,1>(0,3);
    Eigen::Vector3f rpy = VirtualRobot::MathTools::eigen4f2rpy(targetPose);
    
    std::cout << "--------CONFIG--------\n";
    std::cout << "Target Pose: ("
                << xyz.transpose() << ", "
                << rpy.transpose() << ")" << std::endl;
    
    useCollision = UI.checkBoxColCheckIK->isChecked();
    useReachability = UI.checkBoxReachabilitySpaceIK->isChecked();
    useOnlyPosition = UI.checkBoxOnlyPosition->isChecked();

    cspaceColStepSize = (float)UI.doubleSpinBoxCSpaceColStepSize->value();
    cspacePathStepSize = (float)UI.doubleSpinBoxCSpacePathStepSize->value();
    ikMaxErrorPos = (float)UI.doubleSpinBoxIKMaxErrorPos->value();
    ikMaxErrorOri = (float)UI.doubleSpinBoxIKMaxErrorOri->value();
    ikJacobianStepSize = (float)UI.doubleSpinBoxIKJacobianStepSize->value();
    ikJacobianMaxLoops = (int)UI.doubleSpinBoxIKJacobianMaxLoops->value();

    printInfo();

    // std::vector<double> query = {xyz.x(), xyz.y(), xyz.z(), rpy.x(), rpy.y(), rpy.z()};
    // executeQueryGrasp(query);
    // executeGrasp(xyz, rpy);
    executeGrasp(targetPose);

    buildVisu();
}

void GraspPlannerIKWindow::colModel()
{
    buildVisu();
}

void GraspPlannerIKWindow::sliderSolution(int pos)
{
    if (!birrtSolution)
    {
        return;
    }
    openEEF();

    Saba::CSpacePathPtr s = birrtSolution;

    if (birrtSolOptimized)
    {
        s = birrtSolOptimized;
    }

    float p = (float)pos / 1000.0f;
    Eigen::VectorXf iPos;
    s->interpolate(p, iPos);
    robot->setJointValues(rns, iPos);

    redraw();
}

void GraspPlannerIKWindow::redraw()
{
    viewer->scheduleRedraw();
    UI.frameViewer->update();
    viewer->scheduleRedraw();
    this->update();
    viewer->scheduleRedraw();
}

void GraspPlannerIKWindow::box2TCP()
{
    if (!eef) return;

    RobotNodePtr tcp = eef->getTcp();
    if (!tcp || !targetPoseBox)
    {
        return;
    }

    // Eigen::Matrix4f m = tcp->getGlobalPose();
    float x[6];
    x[0] = -246;
    x[1] = -93;
    x[2] = 664;
    x[3] = -1.57f;
    x[4] = 0.0f;
    x[5] = 3.14f;

    Eigen::Matrix4f m;
    VirtualRobot::MathTools::posrpy2eigen4f(x, m);
    targetPoseBox->setGlobalPose(m);
    viewer->render();
}
