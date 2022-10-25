cmake_minimum_required(VERSION 3.0.2)

find_package(PythonLibs 3.6 REQUIRED)

execute_process ( COMMAND 
         python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
         OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)

file(GLOB core_files
    "core/*.py"
)

file(GLOB executors_files
    "executors/*.py"
)

file(GLOB synf_files
    "syntethic_functions/*.py"
)

INSTALL(
    FILES ${core_files} # optimizer_executor.py datalog.py objective_function.py metric.py
    DESTINATION "${PYTHON_SITE_PACKAGES}/multisolution_active_grasping/core"
)

INSTALL(
    FILES ${executors_files}
    DESTINATION "${PYTHON_SITE_PACKAGES}/multisolution_active_grasping/executors"
)

INSTALL(
    FILES ${synf_files}
    DESTINATION "${PYTHON_SITE_PACKAGES}/multisolution_active_grasping/syntethic_functions"
)
