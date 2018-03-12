# This is a very basic file for the new style find_package() search mode,
# i.e. Config-mode.
# In this mode find_package() searches for a <package>Config.cmake 
# file and an associated <package>Version.cmake file, which it loads to check 
# the version number.
# This file can be used with configure_file() to generate such a file for a project
# with very basic logic.
# It sets PACKAGE_VERSION_EXACT if the current version string and the requested
# version string are exactly the same and it sets PACKAGE_VERSION_COMPATIBLE
# if the current version is >= requested version.
# If this is not good enough for your project, you need to write your own
# improved <package>Version.cmake file.


set(PACKAGE_VERSION 1.2.2)

if("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}" )
   set(PACKAGE_VERSION_COMPATIBLE FALSE)
else("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}" )
   set(PACKAGE_VERSION_COMPATIBLE TRUE)
   if( "${PACKAGE_FIND_VERSION}" STREQUAL "${PACKAGE_VERSION}")
      set(PACKAGE_VERSION_EXACT TRUE)
   endif( "${PACKAGE_FIND_VERSION}" STREQUAL "${PACKAGE_VERSION}")
endif("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}" )
