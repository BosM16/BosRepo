# Project name is not mandatory, but you should use it
project(emc)

set(executable_name test_emc)
#set(CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_CXX_STANDARD 11)

# States that CMake required version must be >=2.8
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# project version                                   
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)

# Appends the cmake/modules path inside the MAKE_MODULE_PATH variable which stores the
# directories of additional CMake modules (eg MacroOutOfSourceBuild.cmake):
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# The macro below forces the build directory to be different from source directory
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build("${PROJECT_NAME} requires an out of source build. Please, see README.")

# configure header file: define version number variables                            
configure_file(
    ${PROJECT_SOURCE_DIR}/cmake/config.h.in
    ${PROJECT_BINARY_DIR}/config.h
    )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


########################################################################################################################
find_package(catkin REQUIRED)

#find_package (Eigen3 3.3 REQUIRED NO_MODULE)
## Find external libraries sets following variables:
## <name>_FOUND, <name>_LIBRARIES (.so-files), <name>_INCLUDE_DIRS (.h-files), <name>_DEFINITIONS (compile flags)
message(" ")
#find_package(OpenCV 3.4.0 EXACT REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_subdirectory(src)


