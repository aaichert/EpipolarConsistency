Copyrights Andre Aichert, aaichert@gmail.com, andre.aichert@cs.fau.de

Epipolar Consistency of X-Ray Images

This project provides both GPU and CPU implementations in C++/CUDA of the Epipolar Consistency Metric. The CPU version relies on Eigen 3 library and follows the 2016 CT-Meeting paper "Efficient Epipolar Consistency" by Aichert et al.

The projects contained in this repository use the CMake build environment. You can use CMake to generate Visual Studio projects for Windows or makefiles for Linux and MacOS. All libraries used are available on all three platforms and the projects should build with very few fixes in source code.

############
### Building LibEpipolarConsietency using Microsoft Windows, Visual Studio and CMake.
### I'm sure Linux folks will manage on their own.

Prerequisites
- VisualStudio
- CMake https://cmake.org/
- Qt5 https://www.qt.io/download/ (Community)
- Eigen http://eigen.tuxfamily.org/
- CUDA https://developer.nvidia.com/cuda-zone
  (CPU Version also builds without CUDA, just leave CUDA_HAVE_GPU unset)

This tutorial includes the following setps

1) Building NLopt

2) Building LibGetSet from SourceForge

3) Building LibEpipolarConsietency

4) Notes
	4.1) Notes on using Eigen
	4.2) Notes on using Qt
	4.3) Notes on using LibEpipolarConsietency from a different project

5) Debugging from within Visual Studio
	5.1) Shared Libraries
	5.2) Configuration
	5.3) Running the EpipolarConsistencyDemo (no CUDA)
	
#####
# 1) Building NLopt

As of June 2016, NLopt binaries are provided on Windows only for MinGW. While the C-interface can be used directly, the c++ interface (which we will use) is not binary compatible. See also https://chadaustin.me/cppinterface.html for more info.

We have to build it ourselves:

Step   I,   Download and uncompress latest sources from http://ab-initio.mit.edu/wiki/index.php/NLopt
Step   II,  Go to http://ab-initio.mit.edu/wiki/index.php/NLopt_on_Windows and download CMakeLists.txt config.cmake.h.in to your NLopt directory.
Step   III, Run Cmake GUI and specify your NLopt directory and a build directory
            (example:)
                Where is the source code: -> C:/Development/nlopt-2.4.2
		        Where to build the binaries:  -> C:/Development/nlopt-2.4.2/build
Step IV,    Click configure and select your version of Visual Studio.
Step  V,    Set CMAKE_INSTALL_PREFIX to where you would like to install nlopt to.
            (example:)
                CMAKE_INSTALL_PREFIX -> C:/Development/extern/nlopt-2.4.2
Step  VI,   Click Configure and Generate again.
            Open your nlopt-2.4.2/build/NLOPT.sln in Visual Studio. Change your configuration type to Release in the Configuration Manager
Step  VII,  In the Solution Explorer, right-click the "INSTALL" project and select "Build".
            (example: the following files should be created:)
                C:/Development/extern/nlopt-2.4.2/include/nlopt.h
                C:/Development/extern/nlopt-2.4.2/include/nlopt.hpp
                C:/Development/extern/nlopt-2.4.2/include/nlopt.f
                C:/Development/extern/nlopt-2.4.2/lib/nlopt-0.dll
				

Note that as of June 2016 the CMakeLists.txt file does NOT create a proper config.cmake file for FIND_PACKAGE in the export directory.
Note also that the nlopt-0.dll is actually a static library if you left BUILD_SHARED_LIBS unchecked (preferably).
Basically, someone should really fix the CMakeLists.txt file. (Perhaps I will...)

At this point, you can delete the sources.
(example:)
    delete "C:/Development/nlopt-2.4.2" but keep "C:/Development/extern/nlopt-2.4.2"

IF YOU ENCOUNTER THE ERROR
    1>nlopt.dir/Release/cobyla.obj : fatal error LNK1136: invalid or corrupt file
    Work-around: There appears to be a bug in the optimizer of cl.exe. Disable optimization for nlopt/cobyla.c by right-clicking in the solution explorer and selecting properties.
    "Configuration Properties/C-C++/Optimization" to "Disabled (/Od)" Then rebuild.

#####
# 
# 2) Building LibGetSet from SourceForge

If you would like to use the test/demo program, you will need to build another library straight from SourceForge.

Step   I,   Go to https://sourceforge.net/projects/getset/ and download the source code.
			(example:)
			    Go to https://sourceforge.net/p/getset/code/ci/master/tree/ and click "Download Snapshot" and unpack to C:/Development/GetSet
			    alternative:
			    git clone git://git.code.sf.net/p/getset/code "C:/Development/GetSet"
Step   II,  Run Cmake GUI and specify your GetSet directory and a build directory
            (example:)
                Where is the source code: -> C:/Development/GetSet
		        Where to build the binaries:  -> C:/Development/GetSet/build
Step   III, Click Configure and select your version of Visual Studio.
Step IV,    Set CMAKE_INSTALL_PREFIX to where you would like to install GetSet to.
            (example:)
                CMAKE_INSTALL_PREFIX -> C:/Development/extern/GetSet
			(Make sure that Qt is found. Example:)
			    Qt5Code_DIR -> C:/Qt/5.6/msvc2013_64/lib/cmake/Qt5Core
Step  V,    Click Configure and Generate again.
            Open your GetSet/build/GetSet.sln in Visual Studio.
Step  VI,   In the Solution Explorer, right-click the "INSTALL" project and select "Build". (This is the Debug Build.)
            Change your configuration type to Release in the configuration manager and build project "INSTALL" again
            (example: the following files should be created:)
                13>  -- Installing: C:/Development/extern/GetSet/cmake/GetSetTargets.cmake
                13>  -- Installing: C:/Development/extern/GetSet/cmake/GetSetTargets-release.cmake
                13>  -- Installing: C:/Development/extern/GetSet/./GetSetConfig.cmake
                13>  -- Installing: C:/Development/extern/GetSet/./GetSetConfigVersion.cmake
                13>  -- Installing: C:/Development/extern/GetSet/include/GetSet/<...>.h/.hxx
                13>  -- Installing: C:/Development/extern/GetSet/include/GetSetGui/<...>.h
                13>  -- Installing: C:/Development/extern/GetSet/lib/GetSet.lib
                13>  -- Installing: C:/Development/extern/GetSet/lib/GetSet.dll
                13>  -- Installing: C:/Development/extern/GetSet/lib/GetSetGui.lib
                13>  -- Installing: C:/Development/extern/GetSet/lib/GetSetGui.dll
                13>  -- Installing: C:/Development/extern/GetSet/lib/GetSetd.lib (<-for debug)
                13>  -- Installing: C:/Development/extern/GetSet/lib/GetSetd.dll (<-for debug)
                13>  -- Installing: C:/Development/extern/GetSet/lib/GetSetGuid.lib (<-for debug)
                13>  -- Installing: C:/Development/extern/GetSet/lib/GetSetGuid.dll (<-for debug)

At this point, you can delete the sources. Anyone with the same compiler and same version of Qt will be able to use C:/Development/extern/GetSet directly.
(example:)
    delete "C:/Development/GetSet" but keep "C:/Development/extern/GetSet"

#####
# 3) Building LibEpipolarConsietency

Step   I,   Uncompress the LibEpipolarConsietency sources
            (example:)
                C:/Development/LibEpipolarConsietency
Step   II,  Run Cmake GUI and specify your LibEpipolarConsietency directory and a build directory
            (example:)
                Where is the source code: -> C:/Development/LibEpipolarConsietency
		        Where to build the binaries:  -> C:/Development/LibEpipolarConsietency/build
Step   III, Click Configure and select your version of Visual Studio.
Step IV,    Due to the lack of a proper CMakeLists.txt file we need to select the paths to NLopt include directory and library manually.
            (example:)
                NLOPT_INCLUDE_PATH -> C:/Development/extern/nlopt-2.4.2/include
				NLOPT_INCLUDE_LIBRARY -> C:/Development/extern/nlopt-2.4.2/lib/nlopt-0.dll
Step  V,    Make sure other libraries (CUDA (optional), Eigen) could be located
            (example:)
                EIGEN_INCLUDE_DIR -> C:/Development/extern/eigen-3.2.4
Step  VI,   Click Configure and Generate again.
            Open your LibEpipolarConsietency/build/EpipolarConsietency.sln in Visual Studio.
Step  VII,  In the Solution Explorer, right-click the "INSTALL" project and select "Build". (This is the debug build)
            Change your configuration type to Release in the Configuration Manager and build project "INSTALL" again

At this point, anyone with the same compiler (i.e. version of Visual Studio) will be able to use LibEpipolarConsietency if you send them ONLY the directory that you specified as CMAKE_INSTALL_PREFIX. As mentioned earlier, see also https://chadaustin.me/cppinterface.html for more info.
(example:)
    compress the directory "C:/Development/EpipolarConsietency/export" and send compressed file by mail.



#####
# 4) Notes

# 4.1) Notes on using Eigen
The Eigen library is entirely header-only. All you need to do is download a source zip from http://eigen.tuxfamily.org/ and unpack it.


# 4.1) Notes on using Qt
Go to https://www.qt.io/download-open-source/ and download the community OpenSource version. Windows installers for Visual Studio exist.
It saves some time later on if you set the environment variable CMAKE_PREFIX_PATH to include your version of Qt. If you skip this step you need to specify the paths "Qt5Code_DIR" etc. in CMake GUI manually.
(example:)
    CMAKE_PREFIX_PATH -> C:/Qt/5.6/msvc2013_64/lib/cmake


# 4.2) Notes on using LibEpipolarConsietency from a different project

Section 3) mentions, that it is possibly to ship LibEpipolarConsietency just by copying the files in the install directory. Setting up a project with CMake that includes LibEpipolarConsietency should be straight-forward.

Step   I,   Create your c++ file
            (example: main.cpp)
Step   II,  Create a CMakeLists.txt file and add the following information to it:
            cmake_minimum_required(VERSION 3.5.2)
			project(ECC_Test)
            find_package(LibProjectiveGeometry)
            find_package(LibEpipolarConsietency)
			add_executable(ECC main.cpp)
			target_link_libraries(ECC LibProjectiveGeometry LibEpipolarConsietency)
Step   III, Run Cmake GUI and specify your source directory and a build directory as always.
Step IV,    Click Configure and set LibEpipolarConsietency_DIR to where you placed the library
 			(example:)
			    LibEpipolarConsietency_DIR -> C:/Development/extern/LibEpipolarConsietency
Step  V,    Click Configure and Generate again.
            Open your build/ECC_Test.sln in Visual Studio. And have fun.

			

#####
# 5) Debugging from within Visual Studio

# 5.1) Shared Libraries
Executables will be build to the directory ./EpipolarConsietency/bin/Debug . If you wish to run these executables from within Visual Studio, you will need to make sure all DLLs are found. The best way to do this is to copy the relevant DLLs to this directory.

For example:

./EpipolarConsietency/bin/Debug/platforms/qwindowsd.dll
./EpipolarConsietency/bin/Debug/ ...
	cudart64_70.dll
	GetSetd.dll
	GetSetGuid.dll
	Qt5Cored.dll
	Qt5Guid.dll
	Qt5OpenGLd.dll
	Qt5PrintSupportd.dll
	Qt5Widgetsd.dll
(depending on actual Qt Version, additional DLLs may be required, e.g. Unicode support.)

The same can be done for ./EpipolarConsietency/bin/Release accordingly (without the "d" for debug in the filename obviously). For shipping binaries, you will also need:
	msvcp120.dll
	msvcr120.dll
	vcomp120.dll
	
# 5.2) Configuration
I recommend creating a directory ./EpipolarConsietency/config and select it as current working directory for all configurations. All .ini files and temporary files will be placed there.
