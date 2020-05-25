// Optimization of Epipolar Consistency over FD-CT Geometry. Gradient Free Version.
// Created by A. Aichert on Mon Aug 04th 2014
// Adapted to motion correction on Thu May 22nd 2015.
// Rewritten Mon May 8th 2017
// Updated to OpenSource version May 25th 2020.


// Managing saving/loading parametzers and automatic GUI
#include <GetSetGui/GetSetGui.h>
#include <GetSetGui/GetSetTabWidget.h>

// Plotting and Visualization
#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>

// Timing cost function evaluations when plotting
#include <Utils/TimerWin32.hxx>

// Utility functions for frequently used loading/visualization routines and GUI.
#include <LibEpipolarConsistency/Gui/InputDataRadonIntermediate.h>
#include <LibEpipolarConsistency/Gui/Visualization.h>
#include <LibEpipolarConsistency/Gui/SingleImageMotion.h>

// Optimization
#include <LibOpterix/WrapNLOpt.hxx>

// Parametrization of Transformation
#include <LibProjectiveGeometry/Models/ModelCameraSimilarity2D3D.hxx>
typedef Geometry::ModelCameraSimilarity2D3D ParameterModel;

GetSetGui::Application g_app("FDCTMotionCompensation");
