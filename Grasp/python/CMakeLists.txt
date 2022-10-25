cmake_minimum_required(VERSION 3.0.2)

set(LIB_NAME "pygrasp")

find_package(SWIG REQUIRED)
find_package(PythonLibs 3.6 REQUIRED)
find_package(Boost REQUIRED)

## SET CMAKE FLAGS ##
link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

set(CMAKE_SWIG_FLAGS -c++) # -Wall

## INCLUDE ##
include(${SWIG_USE_FILE})
include_directories(
    ${PYTHON_INCLUDE_PATH}
    ${Boost_INCLUDE_DIRS}
    "../include"
)
# ${GRASP_LIB_INCLUDE_DIR}
    
## LINK ##
set_source_files_properties(pygrasp.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(pygrasp.i PROPERTIES SWIG_FLAGS "-includeall")

# set(Grasp_LIB "/home/nacho/ActiveGrasping/active-grasping-bo/Grasp/build/libGrasp.so")

swig_add_library(${LIB_NAME} LANGUAGE python SOURCES pygrasp.i)
swig_link_libraries(${LIB_NAME} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} Grasp)

execute_process ( COMMAND 
         python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
         OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)

INSTALL(
    FILES "${PROJECT_SOURCE_DIR}/build/Grasp/python/${LIB_NAME}.py" "${PROJECT_SOURCE_DIR}/build/lib/_${LIB_NAME}.so" # "${CMAKE_CURRENT_SOURCE_DIR}/gutils.py"
    DESTINATION "${PYTHON_SITE_PACKAGES}/pygrasp"
)
