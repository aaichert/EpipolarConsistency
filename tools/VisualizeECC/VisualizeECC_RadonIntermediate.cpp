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
GetSetGui::Application g_app("VisualizeECC_RadonIntermediate");

// Computing  Radon transform and evaluating Epipolar Consistency-
#include <LibEpipolarConsistency/Gui/ComputeRadonIntermediate.hxx>
#include <LibEpipolarConsistency/EpipolarConsistencyRadonIntermediate.h>
#include <LibEpipolarConsistency/EpipolarConsistencyRadonIntermediateCPU.hxx>

/// A call-back function to handle GUI-input
void gui(const GetSetInternal::Node& node)
{
	using namespace EpipolarConsistency;

	// When the button has been clicked
	if (node.name=="Update")
	{
		g_app.progressStart("Epipolar Consistency","Evaluating via Radon intermediate functions...",7);

		// Load image files
		std::string path0=GetSet<>("Epipolar Consistency/Images/Image 0");
		std::string path1=GetSet<>("Epipolar Consistency/Images/Image 1");
		NRRD::Image<float> image0(path0);
		g_app.progressUpdate(1);
		NRRD::Image<float> image1(path1);
		g_app.progressUpdate(2);

		// Chack if data is ok
		if (image0.dimension()!=2 || image1.dimension()!=2) {
			g_app.warn("Failed to Load Input Images", "Images must be uncompressed single-channel projections given 2D NRRD files.");
			return;
		}
		// Load Projection Matrices
		if (image0.meta_info.find("Projection Matrix")==image0.meta_info.end() || image1.meta_info.find("Projection Matrix")==image1.meta_info.end()) {
			g_app.warn("Failed to Load Input Images", "The \"Projection Matrix\" tag must be set in the NRRD header.");
			return;
		}

		// Get pixel spacing and projection matrices
		using Geometry::ProjectionMatrix;
		double spx=GetSet<double>("Epipolar Consistency/Images/Pixel Spacing");
		ProjectionMatrix P0=stringTo<ProjectionMatrix>(image0.meta_info["Projection Matrix"]);
		ProjectionMatrix P1=stringTo<ProjectionMatrix>(image1.meta_info["Projection Matrix"]);


		// Compute both Radon intermediate functions
		EpipolarConsistency::RadonIntermediateFunction compute_dtr;
		using EpipolarConsistency::RadonIntermediate;
		compute_dtr.gui_retreive_section("Epipolar Consistency/Radon Intermediate");
		g_app.progressUpdate(3);
		RadonIntermediate *rif0=compute_dtr.compute(image0,&P0,&spx);
		g_app.progressUpdate(4);
		RadonIntermediate *rif1=compute_dtr.compute(image1,&P1,&spx);
		g_app.progressUpdate(5);
		
		// Evaluate ECC 
		// (this way of evaluating ECC is usually intended for many more than 2 projections, hence the std::vectors)
		std::vector<ProjectionMatrix> Ps;
		Ps.push_back(P0);
		Ps.push_back(P1);
		std::vector<RadonIntermediate*> rifs;
		rifs.push_back(rif0);
		rifs.push_back(rif1);
		// Output data
		std::vector<float> redundant_samples0, redundant_samples1;
		std::vector<float> kappas;
		std::vector<std::pair<float,float> > rif_samples0, rif_samples1;
		// Slow CPU evaluation for just two views
		// (usually, you just call ecc.evaluate(...) which uses the GPU to compute metric for all projections)
		EpipolarConsistency::MetricRadonIntermediate ecc(Ps,rifs);
		ecc.setdKappa(GetSet<double>("Epipolar Consistency/Sampling/Angle Step (deg)")/180*Geometry::Pi);
		double inconsistency=ecc.evaluateForImagePair(0,1, &redundant_samples0, &redundant_samples1,&kappas, &rif_samples0, &rif_samples1);
		g_app.progressUpdate(6);

		// Visualize redundant signals
		QColor red(255,0,0);
		QColor green(0,255,0);
		Plot plot("Epipolar Consistency",true);
		RadonIntermediate::Filter filter=rif0->getFilter();
		std::string filterName=filter==RadonIntermediate::None?"Line Integrals ":(filter==RadonIntermediate::Derivative?"Derivative (Grangeat)":"Ramp filter (Smith)");
		plot.graph()
			.setName("Radon Intermediate 0")
			.setData((int)kappas.size(),kappas.data(),redundant_samples0.data())
			.setColor(green);
		plot.graph()
			.setName("Radon Intermediate 1")
			.setData((int)kappas.size(),kappas.data(),redundant_samples1.data())
			.setColor(red);
		plot.showLegend();
		plot.setAxisAngularX();
		plot.setAxisLabels("Epipolar Plane Angle",filterName +" [a.u.]");

		// Show Radon intermediate functions.
		rif0->readback();
		Figure fig0("Radon Intermediate Function 0", rif0->data(),0,0,rif1->getFilter()!=RadonIntermediate::None);
		fig0.showTiled(0,512,512);
		rif1->readback();
		Figure fig1("Radon Intermediate Function 1", rif1->data(),0,0,rif1->getFilter()!=RadonIntermediate::None);
		fig1.showTiled(1,512,512);

		// Show projections images.
		showImages(image0,image1,P0,P1,spx);

		// Show sample locations (slow)
		double n_alpha1=rif0->data().size(0)-1;
		double n_t1    =rif0->data().size(1)-1;
		double step_alpha=rif1->getRadonBinSize(0);
		int step=(int)rif_samples0.size()/20;
		for (int i=step,i_old=0; i<(int)rif_samples0.size(); i+=step)
		{
			fig0.drawLine(	rif_samples0[i    ].first*n_alpha1, rif_samples0[i    ].second*n_t1,
							rif_samples0[i_old].first*n_alpha1, rif_samples0[i_old].second*n_t1,green);

			fig1.drawLine(	rif_samples1[i    ].first*n_alpha1, rif_samples1[i    ].second*n_t1,
							rif_samples1[i_old].first*n_alpha1, rif_samples1[i_old].second*n_t1,red);
			i_old=i;
		}
		g_app.progressEnd();

		// Bring Geometry to front and apply default rotation
		Figure("Geometry").projectionParameters().setViewingDistance(4500).setAngle(Eigen::Vector4d(-0.7854, 2.356, 0, 0));
		Figure("Geometry").update();
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
	GetSet<double>    ("Epipolar Consistency/Sampling/Angle Step (deg)")=0.01;

	EpipolarConsistency::RadonIntermediateFunction().gui_declare_section("Epipolar Consistency/Radon Intermediate");
	GetSetGui::Button("Epipolar Consistency/Update")="Update...";

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
