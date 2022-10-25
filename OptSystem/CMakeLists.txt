CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

set(LIBRARY_NAME ActiveGraspOpt)

set(ACTIVE_GRASP_OPT_LIB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" PARENT_SCOPE)

## FIND LIBRARIES ##

FIND_LIBRARY(NLOPT nlopt)
if(NOT NLOPT)
  message(FATAL_ERROR "NLOPT library not found")
endif()

FIND_LIBRARY(BAYESOPT bayesopt)
if(NOT BAYESOPT)
  message(FATAL_ERROR "BAYESOPT library not found")
endif()

## EXTERNAL INCLUDE ##

include_directories(
  "${BayesOpt_DIR}/include"
  "${BayesOpt_DIR}/utils"
  ${GRASP_LIB_INCLUDE_DIR}
  )

## LIB FILES ##

set( OPT_INCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ActiveGraspingOpt/ActiveGraspingOpt.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ActiveGraspingOpt/ActiveGraspingOptParams.hpp
)
  
set( OPT_SRCS 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ActiveGraspingOpt.cpp
)

## LINK ##
add_library(${LIBRARY_NAME} STATIC ${OPT_INCS} ${OPT_SRCS})

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} Grasp bayesopt nlopt)

INSTALL(
    TARGETS ${LIBRARY_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
INSTALL(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/ActiveGraspingOpt" DESTINATION include
)

## PYTHON ##
add_subdirectory(python)
