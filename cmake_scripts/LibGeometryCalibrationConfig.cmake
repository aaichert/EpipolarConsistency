get_filename_component(myDir ${CMAKE_CURRENT_LIST_FILE} PATH)
get_filename_component(rootDir ${myDir} ABSOLUTE)

# set the version of myself
set(LIBGEOMETRYCALIBRATION_VERSION_MAJOR 1)
set(LIBGEOMETRYCALIBRATION_VERSION_MINOR 2)
set(LIBGEOMETRYCALIBRATION_VERSION_PATCH 2)
set(LIBGEOMETRYCALIBRATION_VERSION ${LIBGEOMETRYCALIBRATION_VERSION_MAJOR}.${LIBGEOMETRYCALIBRATION_VERSION_MINOR}.${LIBGEOMETRYCALIBRATION_VERSION_PATCH} )

# what is my include directory
set(LIBGEOMETRYCALIBRATION_INCLUDE_DIRS "${rootDir}/include")
set(LIBGEOMETRYCALIBRATION_DIR "${rootDir}")

# import the exported targets
include(${myDir}/cmake/LibGeometryCalibration-targets.cmake)

# set the expected library variable
set(LIBGEOMETRYCALIBRATION_LIBRARIES LibGeometryCalibration )
