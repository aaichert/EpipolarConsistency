get_filename_component(myDir ${CMAKE_CURRENT_LIST_FILE} PATH)
get_filename_component(rootDir ${myDir} ABSOLUTE)

# set the version of myself
set(LIBUTILSGL_VERSION_MAJOR 1)
set(LIBUTILSGL_VERSION_MINOR 2)
set(LIBUTILSGL_VERSION_PATCH 2)
set(LIBUTILSGL_VERSION ${LIBUTILSGL_VERSION_MAJOR}.${LIBUTILSGL_VERSION_MINOR}.${LIBUTILSGL_VERSION_PATCH} )

# what is my include directory
set(LIBUTILSGL_INCLUDE_DIRS "${rootDir}/include")
set(LIBUTILSGL_DIR "${rootDir}")

# import the exported targets
include(${myDir}/cmake/LibUtilsGL-targets.cmake)

# set the expected library variable
set(LIBUTILSGL_LIBRARIES LibUtilsGL )
