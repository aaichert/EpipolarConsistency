# We need cmake version 3.2.2 or higher
cmake_minimum_required(VERSION 3.2.2)

############################################################
# Epipolar Consistency Conditions in X-ray imaging.
# Created by Andre Aichert andre.aichert@fau.de
# Pattern Recognition Lab (CS5)
# Friedrich-Alexander Universität Erlangen-Nürnberg
############################################################

# Create a new solution
project(EpipolarConsistency)

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

# Tell CMake to install binaries and exports locally
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX "../export" CACHE PATH "Default install path" FORCE )
endif()

# Distinguish debug/realse binaries. And nothing else.
set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "limited configs" FORCE)

# Shall dynamic or static linkage be the default?
option(BUILD_SHARED_LIBS "Build Shared Libraries" ON)

# Let MSVC export all symbols by default and create output in bin dir
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# Enable parallel compilation on MSVC
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    message(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
	# Allows solutions to be placed in certain folders
	set_property( GLOBAL PROPERTY USE_FOLDERS ON)
endif()

# I'd rather use current standards
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
    # and a console window under windows
    if (WIN32)
        add_definitions(-mconsole)
    endif()
endif()

####################
# External Packages
####################

# Add OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Add Eigen
set(EIGEN3_INCLUDE_PATH "/usr/include/eigen3" CACHE PATH "Full path to Eigen3")
include_directories(${EIGEN3_INCLUDE_PATH})

# Add NLopt optimizers
find_package(NLopt REQUIRED)
#set(NLOPT_LIBRARY "/usr/lib/x86_64-linux-gnu/libnlopt.so" CACHE FILEPATH  "Full path to nlopt.lib")
#set(NLOPT_INCLUDE_PATH "/usr/include" CACHE PATH "Directory containing nlopt.h/hpp")
include_directories(${NLOPT_INCLUDE_DIRS})


# Add Qt5
find_package(Qt5Widgets)
find_package(OpenGL)
find_package(Qt5OpenGL)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include(cmake_scripts/qt5_win32_copy_dlls.cmake)

# Add GetSet
find_package(GetSet REQUIRED)
include_directories(${GetSet_INCLUDE_DIRS})

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

add_subproject(VisualizeECC                  "tools" OFF)
add_subproject(Registration                  "tools" OFF)
add_subproject(FluoroTracking                "tools" ON)

add_subproject(projtable2ompl                "tools" OFF)
add_subproject(ComputeRadonIntermediate      "tools" OFF)

# temporarily:
add_package(LibSimple           "code"  0 0 1)
add_subproject(Registration2D3D              "tools" OFF)
add_subproject(FDCTCalibrationCorrection     "tools" ON)
add_subproject(FDCTMotionCorrection          "tools" ON)

# Sandbox
# add_subproject(ConsistencyDemo               "tools" OFF)


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
