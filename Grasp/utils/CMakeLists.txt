CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

## FIND PACKAGES ##

FIND_PACKAGE(Simox REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

if(NOT (Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION))
  message(FATAL_ERROR "Simox package not found")
endif()

## LIB FILES ##
link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

SET( GRASP_UI_INCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/utils.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerWindow.h
)
  
SET( GRASP_UI_SRCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/grasp_visualization.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerWindow.cpp
)

SET( GRASP_S_UI_INCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/utils.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerWindowS.h
)
  
SET( GRASP_S_UI_SRCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/grasp_visualizationS.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerWindowS.cpp
)

SET( GRASP_IK_INCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerIKWindow.h
)
  
SET( GRASP_IK_SRCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/grasp_ik_visualization.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerIKWindow.cpp
)

## EXTERNAL INCLUDE ##

include_directories(
  ${Simox_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GRASP_LIB_INCLUDE_DIR}
  )

## LINK ##

set(GUI_MOC_HDRS_PLANNER
    ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerWindow.h
)

set(GUI_UIS_PLANNER
    ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlanner.ui
)

set(GUI_MOC_HDRS_PLANNER_S
    ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerWindowS.h
)

set(GUI_UIS_PLANNER_S
    ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerS.ui
)

set(GUI_MOC_HDRS_PLANNER_IK
    ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerIKWindow.h
)

set(GUI_UIS_PLANNER_IK
    ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerIK.ui
)

SimoxQtApplication(grasp_visualization "${GRASP_UI_SRCS}" "${GRASP_UI_INCS}" "${GUI_MOC_HDRS_PLANNER}" "${GUI_UIS_PLANNER}")
TARGET_LINK_LIBRARIES(grasp_visualization PRIVATE Grasp)

SimoxQtApplication(grasp_visualizationS "${GRASP_S_UI_SRCS}" "${GRASP_S_UI_INCS}" "${GUI_MOC_HDRS_PLANNER_S}" "${GUI_UIS_PLANNER_S}")
TARGET_LINK_LIBRARIES(grasp_visualizationS PRIVATE Grasp)

SimoxQtApplication(grasp_ik_visualization "${GRASP_IK_SRCS}" "${GRASP_IK_INCS}" "${GUI_MOC_HDRS_PLANNER_IK}" "${GUI_UIS_PLANNER_IK}")
TARGET_LINK_LIBRARIES(grasp_ik_visualization PRIVATE Grasp)
