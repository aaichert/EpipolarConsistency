get_filename_component(myDir ${CMAKE_CURRENT_LIST_FILE} PATH)
get_filename_component(rootDir ${myDir} ABSOLUTE)

# set the version of myself
set(LIBPROJECTIVEGEOMETRY_VERSION_MAJOR 1)
set(LIBPROJECTIVEGEOMETRY_VERSION_MINOR 2)
set(LIBPROJECTIVEGEOMETRY_VERSION_PATCH 2)
set(LIBPROJECTIVEGEOMETRY_VERSION ${LIBPROJECTIVEGEOMETRY_VERSION_MAJOR}.${LIBPROJECTIVEGEOMETRY_VERSION_MINOR}.${LIBPROJECTIVEGEOMETRY_VERSION_PATCH} )

# what is my include directory
set(LIBPROJECTIVEGEOMETRY_INCLUDE_DIRS "${rootDir}/include")
set(LIBPROJECTIVEGEOMETRY_DIR "${rootDir}")

# import the exported targets
include(${myDir}/cmake/LibProjectiveGeometry-targets.cmake)

# set the expected library variable
set(LIBPROJECTIVEGEOMETRY_LIBRARIES LibProjectiveGeometry )
