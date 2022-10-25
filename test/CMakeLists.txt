cmake_minimum_required(VERSION 3.0.2)

link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

include_directories(
    ${ACTIVE_GRASP_OPT_LIB_INCLUDE_DIR}
    ${GRASP_LIB_INCLUDE_DIR}
    "${BayesOpt_DIR}/include"
    "${BayesOpt_DIR}/utils"
    )
add_executable(test_libs test_libs.cpp)
add_dependencies(test_libs Grasp ActiveGraspOpt)
TARGET_LINK_LIBRARIES(test_libs Grasp bayesopt ActiveGraspOpt)
