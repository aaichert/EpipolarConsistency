# Epipolar Consistency of X-Ray Images

Created by Andre Aichert, aaichert@gmail.com, andre.aichert@fau.de

This project provides a GPU implementation in C++/CUDA of the Epipolar Consistency Metric. It relies on Eigen 3 library and follows the 2016 CT-Meeting paper "Efficient Epipolar Consistency" by Aichert et al.

The projects contained in this repository use the CMake build environment. You can use CMake to generate Visual Studio projects for Windows or makefiles for Linux and MacOS. All libraries used are available on all three platforms and the projects should build with very few fixes in source code.




# Building LibEpipolarConsistency using Microsoft Windows, Visual Studio and CMake.

I'm sure Linux folks will manage on their own.

Prerequisites
```
- VisualStudio
- CMake https://cmake.org/
- Qt5 https://www.qt.io/download/ (Community)
- Eigen http://eigen.tuxfamily.org/
- CUDA https://developer.nvidia.com/cuda-zone
  (CPU Version also builds without CUDA, just leave CUDA_HAVE_GPU unset)
```
	
## 1) Building NLopt

You can download NLopt from https://nlopt.readthedocs.io/en/latest/ 
It now comes perbuilt with cmake.

## 2) Building LibGetSet

If you would like to use the test/demo program, you will need to build another library straight from SourceForge.

1. Step   I,   Go to https://github.com/aaichert/LibGetSet and download the source code.

2. Step   II,  Run Cmake GUI and specify your GetSet directory and a build directory
```
            (example:)
                Where is the source code: -> C:/Development/GetSet
		        Where to build the binaries:  -> C:/Development/GetSet/build
```
3. Step   III, Click Configure and select your version of Visual Studio.
4. Step IV,    Set CMAKE_INSTALL_PREFIX to where you would like to install GetSet to.
```
            (example:)
                CMAKE_INSTALL_PREFIX -> C:/Development/extern/GetSet
			(Make sure that Qt is found. Example:)
			    Qt5Code_DIR -> C:/Qt/5.6/msvc2013_64/lib/cmake/Qt5Core
```
5. Step  V,    Click Configure and Generate again. Open your GetSet/build/GetSet.sln in Visual Studio.
6. Step  VI,   In the Solution Explorer, right-click the "INSTALL" project and select "Build". (This is the Debug Build.) Change your configuration type to Release in the configuration manager and build project "INSTALL" again
```
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
```
At this point, you can delete the sources. Anyone with the same compiler and same version of Qt will be able to use C:/Development/extern/GetSet directly.
```
(example:)
    delete "C:/Development/GetSet" but keep "C:/Development/extern/GetSet"
```
## 3) Building LibEpipolarConsistency

1. Step   I,   Uncompress the LibEpipolarConsistency sources
```
            (example:)
                C:/Development/LibEpipolarConsistency
```
2. Step   II,  Run Cmake GUI and specify your LibEpipolarConsistency directory and a build directory
```
            (example:)
                Where is the source code: -> C:/Development/LibEpipolarConsistency
		        Where to build the binaries:  -> C:/Development/LibEpipolarConsistency/build
```
3. Step   III, Click Configure and select your version of Visual Studio.
4. Step IV,    The NLOPT_DIR has to be set to the cmake directory deep inside the nlopt directory.

5. Step  V,    Make sure other libraries (CUDA (optional), Eigen) could be located
```
            (example:)
                EIGEN_INCLUDE_DIR -> C:/Development/extern/eigen-3.3.3
```
6. Step  VI,   Click Configure and Generate again. Open your LibEpipolarConsistency/build/EpipolarConsietency.sln in Visual Studio.
7. Step  VII,  In the Solution Explorer, right-click the "INSTALL" project and select "Build". (This is the debug build) Change your configuration type to Release in the Configuration Manager and build project "INSTALL" again

At this point, anyone with the same compiler (i.e. version of Visual Studio) will be able to use LibEpipolarConsistency if you send them ONLY the directory that you specified as CMAKE_INSTALL_PREFIX. As mentioned earlier, see also https://chadaustin.me/cppinterface.html for more info.
```
(example:)
    compress the directory "C:/Development/EpipolarConsietency/export" and send compressed file by mail.
```

## 4) Notes

# 4.1) Notes on using Eigen
The Eigen library is entirely header-only. There have been breaking changes since version 3.3 . To prevent linking errors when building this repo, please resort to using this old version here: https://gitlab.com/libeigen/eigen/-/releases/3.3.0


# 4.1) Notes on using Qt
Go to https://www.qt.io/download-open-source/ and download the community OpenSource version. Windows installers for Visual Studio exist.
It saves some time later on if you set the environment variable CMAKE_PREFIX_PATH to include your version of Qt. If you skip this step you need to specify the paths "Qt5Code_DIR" etc. in CMake GUI manually.
```
(example:)
    CMAKE_PREFIX_PATH -> C:/Qt/5.6/msvc2013_64/lib/cmake
```

# 4.2) Notes on using LibEpipolarConsistency from a different project

Section 3) mentions, that it is possibly to ship LibEpipolarConsistency just by copying the files in the install directory. Setting up a project with CMake that includes LibEpipolarConsistency should be straight-forward.

1. Step   I,   Create your c++ file (example: main.cpp)
2. Step   II,  Create a CMakeLists.txt file and add the following information to it:
```
            cmake_minimum_required(VERSION 3.5.2)
			project(ECC_Test)
            find_package(LibProjectiveGeometry)
            find_package(LibEpipolarConsistency)
			add_executable(ECC main.cpp)
			target_link_libraries(ECC LibProjectiveGeometry LibEpipolarConsistency)
```
3. Step   III, Run Cmake GUI and specify your source directory and a build directory as always.
4. Step IV,    Click Configure and set LibEpipolarConsistency_DIR to where you placed the library
```
 			(example:)
			    LibEpipolarConsistency_DIR -> C:/Development/extern/LibEpipolarConsistency
```
5. Step  V,    Click Configure and Generate again. Open your build/ECC_Test.sln in Visual Studio. And have fun.

			

## 5) Debugging from within Visual Studio

# 5.1) Shared Libraries
Executables will be build to the directory ./EpipolarConsietency/bin/Debug . If you wish to run these executables from within Visual Studio, you will need to make sure all DLLs are found. The best way to do this is to copy the relevant DLLs to this directory.

For example:
```
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
```
(depending on actual Qt Version, additional DLLs may be required, e.g. Unicode support.)

The same can be done for ./EpipolarConsietency/bin/Release accordingly (without the "d" for debug in the filename obviously). For shipping binaries, you will also need:
```
	msvcp120.dll
	msvcr120.dll
	vcomp120.dll
```
	
# 5.2) Configuration
I recommend creating a directory ./EpipolarConsietency/config and select it as current working directory for all configurations. All .ini files and temporary files will be placed there.
