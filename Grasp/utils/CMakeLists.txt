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
  ${CMAKE_CURRENT_SOURCE_DIR}/EigenGraspPlannerWindow.h
)
  
SET( GRASP_UI_SRCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/grasp_visualization.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlannerWindow.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/EigenGraspPlannerWindow.cpp
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
    ${CMAKE_CURRENT_SOURCE_DIR}/EigenGraspPlannerWindow.h
)

set(GUI_UIS_PLANNER
    ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlanner.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/EigenGraspPlanner.ui
)

SimoxQtApplication(grasp_visualization "${GRASP_UI_SRCS}" "${GRASP_UI_INCS}" "${GUI_MOC_HDRS_PLANNER}" "${GUI_UIS_PLANNER}")
TARGET_LINK_LIBRARIES(grasp_visualization PRIVATE Grasp)
