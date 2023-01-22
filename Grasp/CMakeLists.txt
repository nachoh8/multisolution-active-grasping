CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

set(LIBRARY_NAME Grasp)

## FIND PACKAGES ##

FIND_PACKAGE(Simox REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

## LIB FILES ##

SET( GRASP_INCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Grasp/GraspResult.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Grasp/Parameters.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Grasp/CoordSys.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Grasp/Utils.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Grasp/BaseGraspExecutor.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Grasp/GraspPlanner.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Grasp/EigenGraspPlanner.hpp
)
  
SET( GRASP_SRCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/BaseGraspExecutor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GraspPlanner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/EigenGraspPlanner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Utils.cpp
)

set(GRASP_LIB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" PARENT_SCOPE)

## EXTERNAL INCLUDE ##

include_directories(
  ${Simox_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  )

## LINK ##

add_library(${LIBRARY_NAME} STATIC ${GRASP_INCS} ${GRASP_SRCS})

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${EIGEN3_LIBS} ${Boost_LIBRARIES} VirtualRobot GraspStudio)

INSTALL(
    TARGETS ${LIBRARY_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
INSTALL(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/Grasp" DESTINATION include
)

## Python ##
add_subdirectory(python)

## Grasp Visualization ##
add_subdirectory(utils)
