#
# Function to create configure scripts for find_package automatically
# For a package containing the sub-project MyOptionalProject of version 1.0.2
# 	add_package MyOptionalProject "." 1 0 2)
# The same for an optional project that shall not have a find_package script (on by default)
# 	add_subproject(MyOptionalProject "." ON)


#
# Function to create configure scripts for find_package automatically
#
function(configure_find_script LIB_NAME VERSION_MAJOR VERSION_MINOR VERSION_PATCH )
	#set(${varName} ${varValue} PARENT_SCOPE)
	string(TOUPPER ${LIB_NAME} LIB_NAME_CAPS)
	# Create ${LIB_NAME}Config.cmake and ${LIB_NAME}ConfigVersion.cmake files for find_package()
	configure_file(cmake_scripts/Config.cmake.in ${CMAKE_CURRENT_SOURCE_DIR}/cmake_scripts/${LIB_NAME}Config.cmake @ONLY )
	configure_file(cmake_scripts/ConfigVersion.cmake.in ${CMAKE_CURRENT_SOURCE_DIR}/cmake_scripts/${LIB_NAME}ConfigVersion.cmake @ONLY )
	# Install files into the root of the install directory
	install(FILES cmake_scripts/${LIB_NAME}Config.cmake cmake_scripts/${LIB_NAME}ConfigVersion.cmake DESTINATION . )
endfunction(configure_find_script)

#
# Function to add a library or executable from a subdirectory
#
function(add_subproject SUBPROJECT_NAME SUBPROJECT_CATEGORY BUILD_BY_DEFAULT)
	string(TOUPPER ${SUBPROJECT_NAME} SUBPROJECT_NAME_CAPS)
	string(TOUPPER ${SUBPROJECT_CATEGORY} SUBPROJECT_CATEGORY_CAPS)
	option(BUILD${SUBPROJECT_CATEGORY_CAPS}_${SUBPROJECT_NAME_CAPS} "Build ${SUBPROJECT_CATEGORY}/${SUBPROJECT_NAME}?" ${BUILD_BY_DEFAULT})
	if(BUILD${SUBPROJECT_CATEGORY_CAPS}_${SUBPROJECT_NAME_CAPS})
		add_subdirectory(${SUBPROJECT_CATEGORY}/${SUBPROJECT_NAME} "${CMAKE_CURRENT_BINARY_DIR}/${SUBPROJECT_NAME}")
	endif(BUILD${SUBPROJECT_CATEGORY_CAPS}_${SUBPROJECT_NAME_CAPS})
endfunction(add_subproject)

#
# Function to add a library from a subdirectory and configre find script for find_package
#
function(add_package SUBPROJECT_NAME SUBPROJECT_CATEGORY VERSION_MAJOR VERSION_MINOR VERSION_PATCH)
	string(TOUPPER ${SUBPROJECT_NAME} SUBPROJECT_NAME_CAPS)
	string(TOUPPER ${SUBPROJECT_CATEGORY} SUBPROJECT_CATEGORY_CAPS)
	set(BUILD${SUBPROJECT_CATEGORY_CAPS}_${SUBPROJECT_NAME_CAPS} ON)
	if(BUILD${SUBPROJECT_CATEGORY_CAPS}_${SUBPROJECT_NAME_CAPS})
		add_subdirectory(${SUBPROJECT_CATEGORY}/${SUBPROJECT_NAME} "${CMAKE_CURRENT_BINARY_DIR}/${SUBPROJECT_NAME}")
		configure_find_script(${SUBPROJECT_NAME} ${VERSION_MAJOR} ${VERSION_MINOR} ${VERSION_PATCH})
	endif(BUILD${SUBPROJECT_CATEGORY_CAPS}_${SUBPROJECT_NAME_CAPS})
endfunction(add_package)
