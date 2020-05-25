// Created by A. Aichert on Mon Aug 04th 2014
// Test and view multi-projection fluoro data
// Rewritten Mon May 8th 2017

// Managing saving/loading parametzers and automatic GUI
#include <GetSetGui/GetSetGui.h>
#include <GetSetGui/GetSetTabWidget.h>

// Plotting and Visualization
#include <LibUtilsQt/QCameraView.h>
#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>
#include <LibUtilsQt/QGLPlot3D.h>

// Timing cost function evaluations when plotting
#include <Utils/TimerWin32.hxx>

// Utility functions for frequently used loading/visualization routines and GUI.
#include <LibEpipolarConsistency/Gui/InputDataRadonIntermediate.h>
#include <LibEpipolarConsistency/Gui/Visualization.h>

GetSetGui::Application g_app("VisualizeECC_ComputedTomography");

EpipolarConsistency::InputDataRadonIntermediateGui g_data("Input",&g_app);

void gui(const GetSetInternal::Node& node)
{
	// Allow user to edit and save plots as pdf
	if (node.name=="Show Plot Editor...")
		UtilsQt::showPlotEditor();

	if (node.name=="Plot Normal Cost Image")
	{
		if (!g_data.loadData(false)) return;
		EpipolarConsistency::MetricRadonIntermediate ecc(g_data.getProjectionMatrices(),g_data.getRadonIntermediateFunctions());
		EpipolarConsistency::showCostImage("Normal",ecc,true);
	}

	if (node.name=="Plot Cost Image with K=I")
	{
		if (!g_data.loadData(false)) return;
		auto Ps=g_data.getProjectionMatrices();
		// Compute K of first matrix
		Eigen::Matrix3d K,R;
		Eigen::Vector3d t;
		Geometry::projectionMatrixDecomposition(Ps[0],K,R,t,g_data.getPixelSpacing()>0);

		// Compute fundamental matrices, and from that, compute pseudo-projection matriecs
		std::vector<Geometry::ProjectionMatrix> Ps_pairs;
		for (int j=0;j<=(int)Ps.size();j++)
			for (int i=0;i<=(int)Ps.size();i++)
				{
					int pindex=Ps_pairs.size();
					Eigen::Vector4i indices(pindex,pindex+1,i,j);
					auto F=Geometry::computeFundamentalMatrix(Ps[i],Ps[j]);

				}

		// Finally, show cost image
		EpipolarConsistency::MetricRadonIntermediate ecc(Ps,g_data.getRadonIntermediateFunctions());
//		EpipolarConsistency::showCostImage("Pseudo (K=Ki)",ecc,true); // replace
	}

	
	// Save settings to ini-file
	g_app.saveSettings();
}

int main(int argc, char ** argv)
{
	// Investigation of symmetry properties of ECC w.r.t. Affine reconstructinos.
	GetSetGui::Button("Symmetry/Plot Normal Cost Image");
	GetSetGui::Button("Symmetry/Plot Cost Image with K=I");
	GetSetGui::Section("Symmetry").setGrouped();

	// Input Data
	g_data.gui_init();

	// Menu and about text
	g_app.init(argc,argv,gui);
	g_app.window().addMenuItem("File","Show Plot Editor...");
	g_app.window().addMenuItem("File"),
	g_app.window().addDefaultFileMenu();
	g_app.window().aboutText()=
		"<h4>Tracking unknown objects under fluoroscopy.</h4>\n\n"
		"Copyright 2014-2017 by <a href=\"mailto:aaichert@gmail.com?Subject=[Epipolar Consistency]\">Andre Aichert</a> <br>"
		"<h4>Epipolar Consistency:</h4>\n\n"
		"Any two ideal transmission images with perfectly known projection geometry contain redundant information. "
		"Inconsistencies, i.e., motion, truncation, scatter radiation or beam-hardening can be observed using Epipolar Consistency. "
		"<br>"
		"<br>"
		"See also: "
		"<br>"
		"<a href=\"https://www5.cs.fau.de/research/software/epipolar-consistency/\">Pattern Recognition Lab at Friedrich-Alexander University of Erlangen-Nuremberg</a> "
		"<br>"
		"<h4>Licensed under the Apache License, Version 2.0 (the \"License\")</h4>\n\n"
		"You may not use this file except in compliance with the License. You may obtain a copy of the License at "
		"<a href=\"http://www.apache.org/licenses/LICENSE-2.0\">http://www.apache.org/licenses/LICENSE-2.0</a><br>"
		;

	return g_app.exec();
}
