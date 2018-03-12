get_filename_component(myDir ${CMAKE_CURRENT_LIST_FILE} PATH)
get_filename_component(rootDir ${myDir} ABSOLUTE)

# set the version of myself
set(LIBRAYCASTBACKPROJECT_VERSION_MAJOR 1)
set(LIBRAYCASTBACKPROJECT_VERSION_MINOR 2)
set(LIBRAYCASTBACKPROJECT_VERSION_PATCH 2)
set(LIBRAYCASTBACKPROJECT_VERSION ${LIBRAYCASTBACKPROJECT_VERSION_MAJOR}.${LIBRAYCASTBACKPROJECT_VERSION_MINOR}.${LIBRAYCASTBACKPROJECT_VERSION_PATCH} )

# what is my include directory
set(LIBRAYCASTBACKPROJECT_INCLUDE_DIRS "${rootDir}/include")
set(LIBRAYCASTBACKPROJECT_DIR "${rootDir}")

# import the exported targets
include(${myDir}/cmake/LibRayCastBackproject-targets.cmake)

# set the expected library variable
set(LIBRAYCASTBACKPROJECT_LIBRARIES LibRayCastBackproject )
