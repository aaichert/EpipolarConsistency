cmake_minimum_required(VERSION 3.16)

############################################################
# Epipolar Consistency Conditions in X-ray imaging.
# Created by Andre Aichert andre.aichert@fau.de
# Pattern Recognition Lab (CS5)
# Friedrich-Alexander Universität Erlangen-Nürnberg
############################################################

project(EpipolarConsistency VERSION 1.2.2 LANGUAGES C CXX CUDA)

# The current version
set(ECC_LIBRARY_VERSION 1 2 2)

# Define add_subproject and add_package to create XXConfig.cmake and XXConfigVersion.cmake
include(cmake_scripts/add_package.cmake)

# Disable in-source builds
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "Please create a separate directory for build files.")
endif()

########################
# Some default settings
########################

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/../export" CACHE PATH "Default install path" FORCE)
endif()

option(BUILD_SHARED_LIBS "Build Shared Libraries" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(MSVC)
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
    add_compile_options(/MP)
endif()

if(MSVC AND CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "limited configs" FORCE)
endif()

####################
# External Packages
####################

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(OPENMP_LIBS OpenMP::OpenMP_CXX)
endif()

# Enable automatic processing of Qt files
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(NLopt REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Svg PrintSupport OpenGLWidgets OpenGL)

find_package(GetSet CONFIG QUIET)
if(NOT TARGET GetSet)
    add_library(GetSet INTERFACE IMPORTED)
endif()
if(NOT TARGET GetSetGui)
    add_library(GetSetGui INTERFACE IMPORTED)
endif()

# Ensure include directories are populated for GetSet targets
if(TARGET GetSet AND EXISTS "${GetSet_DIR}/include")
    target_include_directories(GetSet INTERFACE "${GetSet_DIR}/include")
endif()
if(TARGET GetSetGui AND EXISTS "${GetSet_DIR}/include")
    target_include_directories(GetSetGui INTERFACE "${GetSet_DIR}/include")
endif()

include(cmake_scripts/findAndTestCUDA.cmake)
include(cmake_scripts/qt_win32_copy_dlls.cmake)

########################
# Libraries "./code"
########################

# Add header-only libs
include_directories(./code/HeaderOnly/)
# Important stuff is located in the "code" directory
include_directories(./code)

# The following libraries are always being built and a find_package(...) script is generated
add_package(LibProjectiveGeometry            "code"  ${ECC_LIBRARY_VERSION})
add_package(LibEpipolarConsistency           "code"  ${ECC_LIBRARY_VERSION})
add_package(LibRayCastBackproject            "code"  ${ECC_LIBRARY_VERSION})
add_package(LibUtilsCuda                     "code"  ${ECC_LIBRARY_VERSION})
add_package(LibUtilsQt                       "code"  ${ECC_LIBRARY_VERSION})
add_package(LibGeometryCalibration           "code"  ${ECC_LIBRARY_VERSION})


#####################################################
# Executables "./tools" (enable/disable in CMake)
#####################################################

add_subproject(nrrdView                      "tools" ON)
add_subproject(TrajectoryView                "tools" ON)
add_subproject(VolumeRendering               "tools" ON)

add_subproject(VisualizeECC                  "tools" ON)
add_subproject(Registration                  "tools" OFF)
add_subproject(FluoroTracking                "tools" OFF)

add_subproject(projtable2ompl                "tools" OFF)
add_subproject(ComputeRadonIntermediate      "tools" OFF)

# temporarily:
# add_package(LibSimple           "code"  0 0 1)
add_subproject(Registration2D3D              "tools" OFF)
add_subproject(FDCTCalibrationCorrection     "tools" OFF)
add_subproject(FDCTMotionCorrection          "tools" OFF)

# Sandbox
add_subproject(ConsistencyDemo               "tools" ON)


# move to deprecated
# add_subproject(CuppingCorrection             "tools" ON)
# add_subproject(ProjectionDefects             "tools" OFF)

#####################################################
# Deprecated Executables
#####################################################

# add_subproject(Amsterdam                     "deprecated" OFF)
# add_subproject(AngioMotionCorrection         "deprecated" OFF)
# add_subproject(ComputeRadonDerivative        "deprecated" OFF)
# add_subproject(EpipolarConsistencyDemo       "deprecated" OFF)
# add_subproject(FDCTCalibrationCorrection     "deprecated" OFF)
# add_subproject(FDCTMotionCorrection          "deprecated" OFF)
# add_subproject(FDCTVisualize                 "deprecated" OFF)
# add_subproject(FRST                          "deprecated" OFF)
# add_subproject(JLCCVisualization             "deprecated" OFF)
# add_subproject(LibAmsterdam                  "deprecated" OFF)
# add_subproject(StanfordConverter             "deprecated" OFF)
