# Epipolar Consistency of X-Ray Images

**Created by Andre Aichert (aaichert@gmail.com, andre.aichert@fau.de)**

This project provides a GPU implementation in C++/CUDA of the Epipolar Consistency Metric. It relies on the Eigen 3 library and follows the 2016 CT-Meeting paper *"Efficient Epipolar Consistency"* by Aichert et al.

---

## Prerequisites

The project uses a modern CMake build environment and requires:
* **CMake** (version >= 3.16)
* **C++17 Compiler** (GCC >= 8, MSVC >= 2019, or Clang)
* **Qt6** (Components: `Core`, `Gui`, `Widgets`, `Svg`, `PrintSupport`, `OpenGLWidgets`, `OpenGL`)
* **Eigen 3** (Header-only library, version >= 3.3.0)
* **NLopt** (Non-linear optimization library)
* **CUDA Toolkit** (Optional; required for GPU/CUDA backends)
* **LibGetSet** (Required for GUI tools, plot viewers, and property configuration panels)

---

## Dependency Setup: LibGetSet
Since this repository integrates with the **LibGetSet** properties configuration system, you will need to build and install it beforehand:
1. Clone the repository from [aaichert/LibGetSet](https://github.com/aaichert/LibGetSet).
2. Configure, build, and install it to an install directory (e.g., `<LibGetSet-source>/install`).
3. Pass the path to this install directory to `EpipolarConsistency` via the CMake parameter `-DGetSet_DIR=<path/to/LibGetSet/install>`.

---

## Building and Running on Linux

### 1. Build Instructions
Create a build directory, configure the project with CMake, and build:

```bash
# Configure the build (set GetSet_DIR to the installation path of LibGetSet)
cmake -B build -S . \
  -DGetSet_DIR=/home/aaichert/nc_local/work/LibGetSet/install \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN/../lib"

# Compile all enabled targets
cmake --build build -j$(nproc)

# Install files to a local directory (optional)
cmake --install build --prefix ./install
```

### 2. Running Instructions

#### From the Build Tree (Development)
During build-time, CMake configures the `RPATH` of the binaries automatically, allowing them to run directly without modifying library paths:
```bash
# Run the plot/nrrd viewer
./build/nrrdView/nrrdView

# Run the trajectory visualizer
./build/TrajectoryView/TrajectoryView

# Run the Radon intermediate visualizer
./build/VisualizeECC/VisualizeECC
```

#### From the Install Tree
If running the installed binaries (where RPATH may be stripped), make sure the dynamic linker can locate the shared libraries:
```bash
# Export the library path
export LD_LIBRARY_PATH=/path/to/install/lib:$LD_LIBRARY_PATH

# Run the binary
/path/to/install/bin/nrrdView
```

#### Recommended Working Directory
We recommend running all tools with their working directory set to `config/`. This keeps generated `.ini` property files, logs, and temp configurations organized:
```bash
cd config
../build/nrrdView/nrrdView
```

---

## Building and Running on Windows

### 1. Build Instructions (Visual Studio / MSBuild)
Use CMake to generate Visual Studio solution files or build directly from the command line:

```cmd
:: Configure the project (set GetSet_DIR to your LibGetSet install path)
cmake -B build -S . -DGetSet_DIR=C:/Development/extern/GetSet

:: Build the project in Release configuration
cmake --build build --config Release

:: Install the project files (optional)
cmake --install build --prefix C:/Development/export/EpipolarConsistency --config Release
```

If you prefer a graphical interface, run the **CMake GUI**, specify the source directory and a build directory, configure (using Visual Studio 2019/2022 generator), set the required directory variables (such as `GetSet_DIR` and `Eigen3_DIR`), and click **Generate**. Open the generated `EpipolarConsistency.sln` solution file in Visual Studio.

### 2. Running Instructions
To run the executables on Windows:
* Ensure that the **Qt6 DLLs**, **CUDA runtime DLLs**, and **LibGetSet DLLs** are in your system's `PATH`, or copy them into the directory containing your executables (e.g., `build/bin/Release` or `install/bin`).
* Set your working directory to the `./config` folder to keep your environment tidy.
