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

// A Simple QT Utility to Show a Settings Window
#include <GetSetGui/GetSetGui.h>
GetSetGui::Application g_app("VisualizeECC");

// Transforming line coordinates
#include <LibEpipolarConsistency/EpipolarConsistencyDirect.h>

/// Function called when user clicks into projection images. Epipolar lines are drawn in 2 and 3 dimensions.
void updateEpipolarLines(const std::string& figure_name, bool is_blue, std::vector<Eigen::Vector4d>& selections)
{
	// Number of selections
	int n=(int)selections.size();
	if (is_blue || n==0 || n>10) {
		selections.clear();
		return;
	}
	// Do not allow ractangular selections
	if (!selections.back().tail(2).isConstant(1)) {
		selections.pop_back();
		return;
	}
	// Figure out which image has been clicked on with what mouse button
	Figure figure0(figure_name);
	Figure figure1((figure_name=="Image 0")?"Image 1":"Image 0");
	figure1.clearSelection(is_blue);

	// Figure out epipolar geometry
	using namespace Geometry;
	ProjectionMatrix P0=figure0.getProjectionMatrix();
	ProjectionMatrix P1=figure1.getProjectionMatrix();
	// Figure out projection to epipolar line 1 and plane
	auto       F=computeFundamentalMatrix(P0,P1);
	auto  P0invT=pseudoInverse(P0).transpose().eval();
	RP3Line    B=join_pluecker(getCameraCenter(P0),getCameraCenter(P1));
	// Figure out epipolar planes at 0 and 90 degrees w.r.t. the origin.
	RP3Plane  E0=join_pluecker(B,origin3);
	RP3Plane E90=join_pluecker(B,E0);
	// Convert to Hessian normal form
	E0/=E0.head(3).norm();
	E90/=E90.head(3).norm();
	// Figure out image planes
	double spacing=GetSet<double>("FBCC/Images/Pixel Spacing");
	auto I0=getCameraImagePlane(P0,spacing);
	auto I1=getCameraImagePlane(P1,spacing);
	// Epipoles in 3D
	auto Ep1=meet_pluecker(B,I0);
	auto Ep0=meet_pluecker(B,I1);
	// Intersection line of image planes
	auto I=meet_pluecker(I0,I1);
	// Remove all overlas from figures, except for 3D geometry
	GraphicsItems::Group geom3d=figure0.overlay().group("3D");
	figure0.overlay().clear().set("3D",geom3d);
	figure1.overlay().clear().set("3D",geom3d);
	// Prepare 2D plot
	Plot plot("Epipolar Consistency");
	plot.clearItems();
	// Prepare info for 3D plot
	auto& line3d=Figure("Geometry").overlay().group("3D Lines");
	line3d.clear();
	line3d.add(GraphicsItems::PlueckerLine3D(B,2,QColor(0,0,0)));
	for (int i=0;i<n;i++)
	{
		RP2Point x0(selections[i][0],selections[i][1],1);
		RP2Line  l1=F*x0;
		RP3Plane  E=P1.transpose()*l1;
		RP2Line  l0=P0invT*E;
		double kappa=plane_angle_in_pencil(E,E0,E90);
		auto color=GraphicsItems::colorByIndex(i+2);
		plot.drawVerticalLine(kappa,color,1);
		figure0.overlay().add(GraphicsItems::PlueckerLine2D(l0,1,color));
		figure1.overlay().add(GraphicsItems::PlueckerLine2D(l1,1,color));
		auto corner=meet_pluecker(I,E);
		line3d.add(GraphicsItems::Line3D(Ep0,corner,1,color));
		line3d.add(GraphicsItems::Line3D(Ep1,corner,1,color));
	}
}

/// A call-back function to handle GUI-input
void gui(const GetSetInternal::Node& node)
{
	using namespace EpipolarConsistency;

	// When the button has been clicked
	if (node.name=="Update")
	{
		// Load Images (and make sure they are single channel 2D)
		NRRD::Image<float> I0(GetSet<>("FBCC/Images/Image 0"));
		NRRD::Image<float> I1(GetSet<>("FBCC/Images/Image 1"));
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
		auto C0=Geometry::getCameraCenter(P0);
		auto C1=Geometry::getCameraCenter(P1);

		// Visualize Projection Matrices
		double spacing=GetSet<double>("FBCC/Images/Pixel Spacing");
		Eigen::Vector4d image_rect(0,0,I0.size(0),I0.size(1));
		GraphicsItems::Group static_3d_geometry;
		auto cube=GraphicsItems::ConvexMesh::Cube();
		static_3d_geometry
			.add(GraphicsItems::CoordinateAxes())
			.add(cube);
		cube.q_color.front()=QColor(0,0,0,64);
		cube.l_color=QColor(0,0,0,255);
		auto proj_to_plane0=Geometry::centralProjectionToPlane(C0,Geometry::SourceDetectorGeometry(P0,spacing).image_plane);
		auto proj_to_plane1=Geometry::centralProjectionToPlane(C1,Geometry::SourceDetectorGeometry(P1,spacing).image_plane);
		GraphicsItems::Group image_plane_0(proj_to_plane0);
		GraphicsItems::Group image_plane_1(proj_to_plane1);
		image_plane_0.add(cube);
		image_plane_1.add(cube);
		Figure("Geometry",800,600).overlay()
			.add(static_3d_geometry)
			.add(image_plane_0)
			.add(image_plane_1)
			.add(GraphicsItems::ConvexMesh::Camera(P0,image_rect,spacing,true,QColor(255,0,0,32)))
			.add(GraphicsItems::ConvexMesh::Camera(P1,image_rect,spacing,true,QColor(0,255,0,32)));

		// Visualize Images
		Figure figure0("Image 0",I0);
		Figure figure1("Image 1",I1);
		figure0.showTiled(0,512,420).setProjectionMatrix(P0).overlay().group("3D").add(static_3d_geometry);
		figure1.showTiled(1,512,420).setProjectionMatrix(P1).overlay().group("3D").add(static_3d_geometry);

		// Download image data to GPU
		CudaTexture I0_tex(I0.size(0),I0.size(1),I0);
		CudaTexture I1_tex(I1.size(0),I1.size(1),I1);

		// Main work done here: Compute redundant signals
		std::vector<float> v0s,v1s,kappas;
		computeForImagePair(
			P0,P1,I0_tex,I1_tex,
			GetSet<double>("FBCC/Sampling/Angle Step (deg)")/180*Pi,
			GetSet<double>("FBCC/Sampling/Object Radius"),
			GetSet<int>   ("FBCC/Sampling/Mode")==1,
			&v0s,&v1s,&kappas
			);
		int n_lines=(int)kappas.size();

		// Plot
		Plot plot("Epipolar Consistency",true);
		plot.setAxisLabels("Epipolar Plane Angle","Cosnsistency Metric [a.u.]")
			.setAxisAngularX()
			.showLegend();
		plot.graph().setData(n_lines,kappas.data(),v0s.data()).setName("Image 0").setColor(1,0,0);
		plot.graph().setData(n_lines,kappas.data(),v1s.data()).setName("Image 1").setColor(0,1,0);
		
		figure0.setCallback(updateEpipolarLines);
		figure1.setCallback(updateEpipolarLines);
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
	GetSetGui::File   ("FBCC/Images/Image 0"           ).setExtensions("2D NRRD image (*.nrrd);All Files (*)");
	GetSetGui::File   ("FBCC/Images/Image 1"           ).setExtensions("2D NRRD image (*.nrrd);All Files (*)");
	GetSet<double>    ("FBCC/Images/Pixel Spacing"     )=.308;
	GetSetGui::Section("FBCC/Images"                   ).setGrouped();
	GetSet<double>    ("FBCC/Sampling/Object Radius"   )=50;
	GetSet<double>    ("FBCC/Sampling/Angle Step (deg)")=0.01;
	GetSetGui::Enum   ("FBCC/Sampling/Mode"            ).setChoices("ECC;FBCC");
	GetSetGui::Section("FBCC/Sampling"                 ).setGrouped();
	GetSetGui::Button ("FBCC/Update"                   )="Update Plots";

	GetSetGui::Button("FBCC/Check Derivative")="Check";

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
		"<a href=\"https://www5.cs.fau.de/research/software/epipolar-consistency/\">Pattern Recognition Lab at Technical University of Erlangen-Nuremberg</a> "
		"<br>"
		"<h4>Licensed under the Apache License, Version 2.0 (the \"License\")</h4>\n\n"
		"You may not use this file except in compliance with the License. You may obtain a copy of the License at "
		"<a href=\"http://www.apache.org/licenses/LICENSE-2.0\">http://www.apache.org/licenses/LICENSE-2.0</a><br>"
		;
	return g_app.exec();
}
