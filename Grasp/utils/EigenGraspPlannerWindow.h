
#pragma once

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Grasping/GraspSet.h>

#include "GraspPlanning/GraspStudio.h"
#include "GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h"
#include "GraspPlanning/GraspPlanner/GenericGraspPlanner.h"
#include "GraspPlanning/ApproachMovementSurfaceNormal.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include <GraspPlanning/GraspQuality/GraspEvaluationPoseUncertainty.h>


#include <vector>

#include "../include/Grasp/EigenGraspPlanner.hpp"
#include "../include/Grasp/Parameters.hpp"

#include "ui_EigenGraspPlanner.h"

using namespace Grasp;

class EigenGraspPlannerWindow : public QMainWindow, public EigenGraspPlanner
{
    Q_OBJECT
public:
    EigenGraspPlannerWindow(const Grasp::EnvParameters& _planner_params, const std::vector<Grasp::GraspData>& _grasps, const std::vector<Grasp::GraspData>& _best_grasps);
    ~EigenGraspPlannerWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

    /**
     * @brief Execute grasp for this eef pose
     * 
     * @param xyz-position @param rpy-orientation 
     * @return Grasp quality 
     */
    GraspResult executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp=false);

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;


    void closeEEF();
    void openEEF();

    void colModel();
    void frictionConeVisu();
    void bestGraspsVisu();
    void buildBestGraspsSetVisu();

    void buildVisu();

    //void plan();
    void add_grasp();
    void measure_quality();

    void previous_grasp();
    void next_grasp();

    void sliderReleased_ObjectDOF();
    void sliderReleased_ObjectDOFV();
    void sliderReleased_ObjectAmplitude();
    void sliderReleased_ObjectAmplitudeV();

protected:
    bool evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::RobotPtr eefRobot, VirtualRobot::EndEffectorPtr eef, int nrEvalLoops, GraspStudio::GraspEvaluationPoseUncertainty::PoseEvalResults& results);

    void setupUI();

    static void timerCB(void* data, SoSensor* sensor);

    int current_grasp = -1;

    int current_dof = 0, current_amplitude = 0;

    Ui::EigenGraspPlanner UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* objectSep;
    SoSeparator* frictionConeSep;
    SoSeparator* graspsSep;

    std::vector<GraspData> best_grasps;
    std::vector<std::shared_ptr<VirtualRobot::CoinVisualization> > best_grasps_visu;

    std::string scene_file;
    std::string eefName;
    std::string preshape;

    SoSeparator* eefVisu;

    std::shared_ptr<VirtualRobot::CoinVisualization> visualizationRobot;
    std::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;

};
