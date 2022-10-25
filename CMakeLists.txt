cmake_minimum_required(VERSION 3.0.2)

project(ActiveGrasping)

## CMAKE SETTINGS ##

set(CMAKE_BUILD_TYPE RELEASE) # RELEASE or DEBUG

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if (NOT "$ENV{BayesOpt_DIR}" STREQUAL "")
  set(BayesOpt_DIR "$ENV{BayesOpt_DIR}" CACHE INTERNAL "Copied from environment variable")
endif()

## C++ FLAGS ##
set(CMAKE_C_COMPILER "gcc-8")
set(CMAKE_CXX_COMPILER "g++-8")
set(CMAKE_CXX_STANDARD_DEFAULT)
add_definitions(-std=c++2a)


## MODULES ##
add_subdirectory(Grasp ${CMAKE_BINARY_DIR}/Grasp)
add_subdirectory(OptSystem ${CMAKE_BINARY_DIR}/OptSystem)

## TESTS ##
add_subdirectory(test ${CMAKE_BINARY_DIR}/tests)
