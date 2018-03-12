get_filename_component(myDir ${CMAKE_CURRENT_LIST_FILE} PATH)
get_filename_component(rootDir ${myDir} ABSOLUTE)

# set the version of myself
set(LIBUTILSCUDA_VERSION_MAJOR 1)
set(LIBUTILSCUDA_VERSION_MINOR 2)
set(LIBUTILSCUDA_VERSION_PATCH 2)
set(LIBUTILSCUDA_VERSION ${LIBUTILSCUDA_VERSION_MAJOR}.${LIBUTILSCUDA_VERSION_MINOR}.${LIBUTILSCUDA_VERSION_PATCH} )

# what is my include directory
set(LIBUTILSCUDA_INCLUDE_DIRS "${rootDir}/include")
set(LIBUTILSCUDA_DIR "${rootDir}")

# import the exported targets
include(${myDir}/cmake/LibUtilsCuda-targets.cmake)

# set the expected library variable
set(LIBUTILSCUDA_LIBRARIES LibUtilsCuda )
