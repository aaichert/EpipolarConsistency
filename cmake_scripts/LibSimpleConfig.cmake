get_filename_component(myDir ${CMAKE_CURRENT_LIST_FILE} PATH)
get_filename_component(rootDir ${myDir} ABSOLUTE)

# set the version of myself
set(LIBSIMPLE_VERSION_MAJOR 0)
set(LIBSIMPLE_VERSION_MINOR 0)
set(LIBSIMPLE_VERSION_PATCH 1)
set(LIBSIMPLE_VERSION ${LIBSIMPLE_VERSION_MAJOR}.${LIBSIMPLE_VERSION_MINOR}.${LIBSIMPLE_VERSION_PATCH} )

# what is my include directory
set(LIBSIMPLE_INCLUDE_DIRS "${rootDir}/include")
set(LIBSIMPLE_DIR "${rootDir}")

# import the exported targets
include(${myDir}/cmake/LibSimple-targets.cmake)

# set the expected library variable
set(LIBSIMPLE_LIBRARIES LibSimple )
