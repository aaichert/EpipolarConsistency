// Created by A. Aichert on Thu Jan 18th 2018

// NRRD Image File Format
#include <NRRD/nrrd_image.hxx>

// Simple CUDA Wrappers
#include <LibUtilsCuda/CudaBindlessTexture.h>
typedef UtilsCuda::BindlessTexture2D<float> CudaTexture;

// Utilities for Displaying Images and Plots
#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>
using UtilsQt::Figure;
using UtilsQt::Plot;

#include "draw_epipolar_lines.hxx"

// A Simple QT Utility to Show a Settings Window
#include <GetSetGui/GetSetGui.h>
GetSetGui::Application g_app("VisualizeECC");

// Transforming line coordinates
#include <LibEpipolarConsistency/EpipolarConsistencyDirect.h>

/// A call-back function to handle GUI-input
void gui(const GetSetInternal::Node& node)
{
	using namespace EpipolarConsistency;

	// When the button has been clicked
	if (node.name=="Update")
	{
		// Load Images (and make sure they are single channel 2D)
		NRRD::Image<float> I0(GetSet<>("Epipolar Consistency/Images/Image 0"));
		NRRD::Image<float> I1(GetSet<>("Epipolar Consistency/Images/Image 1"));
		if (I0.dimension()!=2 || I1.dimension()!=2) {
			g_app.warn("Failed to Load Input Images", "Images must be uncompressed single-channel projections given 2D NRRD files.");
			return;
		}
		// Load Projection Matrices
		if (I0.meta_info.find("Projection Matrix")==I0.meta_info.end() || I1.meta_info.find("Projection Matrix")==I1.meta_info.end()) {
			g_app.warn("Failed to Load Input Images", "The \"Projection Matrix\" tag must be set in the NRRD header.");
			return;
		}

		// Get data
		auto P0=stringTo<ProjectionMatrix>(I0.meta_info["Projection Matrix"]);
		auto P1=stringTo<ProjectionMatrix>(I1.meta_info["Projection Matrix"]);
		auto C0=Geometry::getCameraCenter(P0);
		auto C1=Geometry::getCameraCenter(P1);
		double spacing=GetSet<double>("Epipolar Consistency/Images/Pixel Spacing");

		// Download image data to GPU
		CudaTexture I0_tex(I0.size(0),I0.size(1),I0);
		CudaTexture I1_tex(I1.size(0),I1.size(1),I1);

		// Main work done here: Compute redundant signals
		std::vector<float> v0s,v1s,kappas;
		computeForImagePair(
			P0,P1,I0_tex,I1_tex,
			GetSet<double>("Epipolar Consistency/Sampling/Angle Step (deg)")/180*Pi,
			GetSet<double>("Epipolar Consistency/Sampling/Object Radius"),
			GetSet<int>   ("Epipolar Consistency/Sampling/Type")==1,
			&v0s,&v1s,&kappas
			);
		int n_lines=(int)kappas.size();

		// Plot
		Plot plot("Epipolar Consistency",true);
		plot.setAxisLabels("Epipolar Plane Angle",GetSet<>("Epipolar Consistency/Sampling/Type").getString()+" [a.u.]")
			.setAxisAngularX()
			.showLegend();
		plot.graph().setData(n_lines,kappas.data(),v0s.data()).setName("Image 0").setColor(1,0,0);
		plot.graph().setData(n_lines,kappas.data(),v1s.data()).setName("Image 1").setColor(0,1,0);

		// Show images
		showImages(I0,I1,P0,P1,spacing);

	}
	
	if (node.name=="Check Derivative")
	{
		
		// Load Images (and make sure they are single channel 2D)
		NRRD::Image<float> I0(GetSet<>("Epipolar Consistency/Images/Image 0"));
		NRRD::Image<float> I1(GetSet<>("Epipolar Consistency/Images/Image 1"));
		if (I0.dimension()!=2 || I1.dimension()!=2) {
			g_app.warn("Failed to Load Input Images", "Images must be uncompressed single-channel projections given 2D NRRD files.");
			return;
		}

		// Load Projection Matrices
		if (I0.meta_info.find("Projection Matrix")==I0.meta_info.end() || I1.meta_info.find("Projection Matrix")==I1.meta_info.end()) {
			g_app.warn("Failed to Load Input Images", "The \"Projection Matrix\" tag must be set in the NRRD header.");
			return;
		}
		auto P0=stringTo<ProjectionMatrix>(I0.meta_info["Projection Matrix"]);
		auto P1=stringTo<ProjectionMatrix>(I1.meta_info["Projection Matrix"]);
				// Download image data to GPU
		CudaTexture I0_tex(I0.size(0),I0.size(1),I0);
		CudaTexture I1_tex(I1.size(0),I1.size(1),I1);

		std::vector<float> v0s,v1s,kappas;
		std::vector<float> v0s_ecc,v1s_ecc;


		computeForImagePair(
			P0,P1,I0_tex,I1_tex,
			GetSet<double>("Epipolar Consistency/Sampling/Angle Step (deg)")/180*Pi,
			GetSet<double>("Epipolar Consistency/Sampling/Object Radius"),
			true,
			&v0s,&v1s,&kappas
			);

		computeForImagePair(
			P0,P1,I0_tex,I1_tex,
			GetSet<double>("Epipolar Consistency/Sampling/Angle Step (deg)")/180*Pi,
			GetSet<double>("Epipolar Consistency/Sampling/Object Radius"),
			false,
			&v0s_ecc,&v1s_ecc
			);
	
		int n=(int)kappas.size();
		std::vector<float> v0s_ecc2(n,0),v1s_ecc2(n,0);
		for (int i=1;i<n-1;i++) {
			v0s_ecc2[i]=v0s[i-1]-v0s[i+1];
			v1s_ecc2[i]=v1s[i-1]-v1s[i+1];
		}

		double max_ecc=0,max_ecc2=0;
		for (int i=1;i<n-1;i++) {
			if (max_ecc <std::abs(v0s_ecc [i])) max_ecc =std::abs(v0s_ecc [i]);
			if (max_ecc2<std::abs(v0s_ecc2[i])) max_ecc2=std::abs(v0s_ecc2[i]);
		}
		double scale=max_ecc/max_ecc2;

		for (int i=1;i<n-1;i++){
			v0s_ecc2[i]*=scale;
			v1s_ecc2[i]*=scale;
			// Dirty sign fix (actually, order matters...)
			v0s_ecc[i]*=-1;
			v1s_ecc[i]*=-1;
			v0s_ecc2[i]*=-1;
			v1s_ecc2[i]*=-1;
		}

		using UtilsQt::Plot;
		Plot plot("Comparison of ECC and deriv FBCC",true);
		plot.graph().setData(n,kappas.data(),v0s_ecc.data()).setName("Grangeat 0");
		plot.graph().setData(n,kappas.data(),v1s_ecc.data()).setName("Grangeat 1");
		plot.graph().setData(n,kappas.data(),v0s_ecc2.data()).setName("deriv. FBCC 0");
		plot.graph().setData(n,kappas.data(),v1s_ecc2.data()).setName("deriv. FBCC 1");
		plot.setAxisLabels("Epipolar Plane Angle","Cosnsistency Metric [a.u.]")
			.setAxisAngularX()
			.showLegend();
	}
		
	// Allow user to edit and save plots as pdf
	if (node.name=="Show Plot Editor...")
		UtilsQt::showPlotEditor();

	// Write ini-File
	g_app.saveSettings();
}

/// Main: Show little window with settings.
int main(int argc, char ** argv)
{
	// Define default settings
	GetSetGui::File   ("Epipolar Consistency/Images/Image 0"           ).setExtensions("2D NRRD image (*.nrrd);All Files (*)");
	GetSetGui::File   ("Epipolar Consistency/Images/Image 1"           ).setExtensions("2D NRRD image (*.nrrd);All Files (*)");
	GetSet<double>    ("Epipolar Consistency/Images/Pixel Spacing"     )=.308;
	GetSetGui::Section("Epipolar Consistency/Images"                   ).setGrouped();
	GetSet<double>    ("Epipolar Consistency/Sampling/Object Radius"   )=50;
	GetSet<double>    ("Epipolar Consistency/Sampling/Angle Step (deg)")=0.01;
	GetSetGui::Enum   ("Epipolar Consistency/Sampling/Type"            ).setChoices("Derivative (Grangeat);FBCC (weighted integrals)");
	GetSetGui::Section("Epipolar Consistency/Sampling"                 ).setGrouped();
	GetSetGui::Button ("Epipolar Consistency/Update"                   )="Update Plots";

	GetSetGui::Button("Epipolar Consistency/Check Derivative")="Check";

	// Run application
	g_app.init(argc,argv,gui);
	g_app.window().addMenuItem("File","Show Plot Editor...");
	g_app.window().addMenuItem("File"),
	g_app.window().addDefaultFileMenu();
	g_app.window().aboutText()=
		"<h4>Visualization of Epipolar Consistency and Fan-Beam Consistency.</h4>\n\n"
		"Copyright 2014-2018 by <a href=\"mailto:aaichert@gmail.com?Subject=[Epipolar Consistency]\">Andre Aichert</a> <br>"
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
