get_filename_component(myDir ${CMAKE_CURRENT_LIST_FILE} PATH)
get_filename_component(rootDir ${myDir} ABSOLUTE)

# set the version of myself
set(LIBEPIPOLARCONSISTENCY_VERSION_MAJOR 1)
set(LIBEPIPOLARCONSISTENCY_VERSION_MINOR 2)
set(LIBEPIPOLARCONSISTENCY_VERSION_PATCH 2)
set(LIBEPIPOLARCONSISTENCY_VERSION ${LIBEPIPOLARCONSISTENCY_VERSION_MAJOR}.${LIBEPIPOLARCONSISTENCY_VERSION_MINOR}.${LIBEPIPOLARCONSISTENCY_VERSION_PATCH} )

# what is my include directory
set(LIBEPIPOLARCONSISTENCY_INCLUDE_DIRS "${rootDir}/include")
set(LIBEPIPOLARCONSISTENCY_DIR "${rootDir}")

# import the exported targets
include(${myDir}/cmake/LibEpipolarConsistency-targets.cmake)

# set the expected library variable
set(LIBEPIPOLARCONSISTENCY_LIBRARIES LibEpipolarConsistency )
