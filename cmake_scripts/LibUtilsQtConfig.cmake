get_filename_component(myDir ${CMAKE_CURRENT_LIST_FILE} PATH)
get_filename_component(rootDir ${myDir} ABSOLUTE)

# set the version of myself
set(LIBUTILSQT_VERSION_MAJOR 1)
set(LIBUTILSQT_VERSION_MINOR 2)
set(LIBUTILSQT_VERSION_PATCH 2)
set(LIBUTILSQT_VERSION ${LIBUTILSQT_VERSION_MAJOR}.${LIBUTILSQT_VERSION_MINOR}.${LIBUTILSQT_VERSION_PATCH} )

# what is my include directory
set(LIBUTILSQT_INCLUDE_DIRS "${rootDir}/include")
set(LIBUTILSQT_DIR "${rootDir}")

# import the exported targets
include(${myDir}/cmake/LibUtilsQt-targets.cmake)

# set the expected library variable
set(LIBUTILSQT_LIBRARIES LibUtilsQt )
