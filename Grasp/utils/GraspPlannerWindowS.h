
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

#include "../include/Grasp/GraspPlannerS.hpp"
#include "../include/Grasp/GraspPlannerParams.hpp"

#include "ui_GraspPlannerS.h"

using namespace Grasp;

struct GraspPlannerWindowSParams {
    GraspPlannerParams planner_params;
    std::vector<Grasp::GraspData> grasps;
};

class GraspPlannerWindowS : public QMainWindow, public GraspPlannerS
{
    Q_OBJECT
public:
    GraspPlannerWindowS(const GraspPlannerWindowSParams& params);
    ~GraspPlannerWindowS() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

    /**
     * @brief Execute grasp for this eef pose
     * 
     * @param xyz-position @param rpy-orientation 
     * @return Grasp quality 
     */
    GraspResult executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp=true);

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;


    void closeEEF();
    void openEEF();

    void colModel();
    void frictionConeVisu();

    void buildVisu();

    //void plan();
    void add_grasp();
    void measure_quality();

    void previous_grasp();
    void next_grasp();

    void sliderReleased_ObjectX();
    void sliderReleased_ObjectY();
    void sliderReleased_ObjectZ();

    void sliderReleased_ObjectRX();
    void sliderReleased_ObjectRY();
    void sliderReleased_ObjectRZ();

protected:
    bool evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::RobotPtr eefRobot, VirtualRobot::EndEffectorPtr eef, int nrEvalLoops, GraspStudio::GraspEvaluationPoseUncertainty::PoseEvalResults& results);

    void setupUI();

    static void timerCB(void* data, SoSensor* sensor);

    void updateObj(const float value, const int idx);

    int current_grasp = -1;

    Ui::GraspPlannerS UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* objectSep;
    SoSeparator* frictionConeSep;
    SoSeparator* graspsSep;

    std::string robotFile;
    std::string eefName;
    std::string preshape;

    SoSeparator* eefVisu;

    std::shared_ptr<VirtualRobot::CoinVisualization> visualizationRobot;
    std::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;

};
