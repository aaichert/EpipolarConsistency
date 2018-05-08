// Created by A. Aichert on Mon Aug 04th 2014
// Test and view multi-projection fluoro data
// Rewritten Mon May 8th 2017

// Managing saving/loading parameters and automatic GUI
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
#include <LibEpipolarConsistency/Gui/SingleImageMotion.h>

// Optimization
#include <LibOpterix/WrapNLOpt.hxx>

// Parametrization of Transformation
#include <LibProjectiveGeometry/Models/ModelCameraSimilarity2D3D.hxx>
typedef Geometry::ModelCameraSimilarity2D3D ParameterModel;

GetSetGui::Application g_app("FluoroTracking");

class FluoroTrackingGUI : public GetSetGui::Object {
protected:
	EpipolarConsistency::InputDataRadonIntermediateGui   reference;     //<
	LibOpterix::ParameterGUI                             parameter_gui; //< 

	// Temporary data only available after call to assertData() returns true
	std::vector<Geometry::ProjectionMatrix>              Ps;            //< All projection matrices (including input)
	std::vector<EpipolarConsistency::RadonIntermediate*> dtrs;          //< All Radon intermediate functions (including input)
	Geometry::ProjectionMatrix                           P_input;       //< 
	EpipolarConsistency::RadonIntermediate*              dtr_input;     //< 

	// Not copy c-tible.
	FluoroTrackingGUI(const FluoroTrackingGUI&);

public:
	virtual void gui_init()
	{
		GetSetGui::Section optimization("Optimization",gui_section());
		GetSetGui::Button("Plot/Cost Function",optimization)="Plot...";
		optimization.subsection("Plot").setGrouped();
		parameter_gui.declareActive();
		parameter_gui.declareBounds();
		UtilsNLOpt::OptimizerSettings(UtilsNLOpt::algorithm_names("LN")).gui_declare_section(optimization);

		using Geometry::ProjectionMatrix;
		GetSetGui::Section input("Input",gui_section());
		EpipolarConsistency::InputDataDirect().gui_declare_section(input);
		GetSet<ProjectionMatrix>("Initial Guess", input, ProjectionMatrix::Identity() );
		GetSet<int             >("File Index"   , input);
		GetSet<bool            >("Advanced/Display Radon Intermediate Function", input, false);
		GetSet<bool            >("Advanced/Display Current Geometry", input, true);
	}

	bool assertData() {
		if (!reference.loadData(false) ) {
			app->warn("Failed to load data!","Please make sure that reference projections are correctly loaded.");
			return false;
		}
		int input_index=GetSet<int>("Input/File Index", gui_section());
		if (!dtr_input)
		{
			// Load current input image from fluoro sequence
			NRRD::Image<float> I_input;
			Geometry::ProjectionMatrix P;
			EpipolarConsistency::InputDataDirect input;
			input.gui_retreive_section(gui_section().subsection("Input"));
			if (!input.loadPreprocessedImage(I_input,P,input_index)) {
				app->warn("Failed to load data!","Input projection image could not be accessed.");
				return false;
			}
			// TODO visualize epipolar lines.
			bool show_proj_image=input.advanced.show_pre_processed_imgs;
			if (show_proj_image)
				UtilsQt::Figure("Input Projection Images", I_input)
					.overlay()
						.add(GraphicsItems::CoordinateAxes())
						.add(GraphicsItems::ConvexMesh::Cube());

			// Show the input projection image if desired.
			// If no initial guess is given, we just use the one of the current image (only happens the first time a data set is laoded)
			Geometry::ProjectionMatrix P_initial=GetSet<Geometry::ProjectionMatrix>("Input/Initial Guess",gui_section());
			if (P_initial.isZero() || P_initial.isIdentity())
				GetSet<Geometry::ProjectionMatrix>("Input/Initial Guess",gui_section())=P_initial=P;
			// If no input projection matrix is available (i.e. tracking is about to start) we use the initial guess.
			if (input_index==0 || P_input.isZero() || P_input.isIdentity())
				P_input=P_initial;
			// Next, we have to compute the Radon transform based on the same parameters as the reference data
			EpipolarConsistency::RadonIntermediateFunction dtr_processor;
			dtr_processor.gui_retreive_section(gui_section().subsection("Reference/Radon Intermediate"));
			dtr_input=dtr_processor.compute(I_input,&P_input);
			if (!dtr_input) 
			{
				app->warn("Failed to load data!","The Radon intermediate function of the input image could not be computed.");
				return false;
			}
			// Show the Radon intermediate function of the input image if desired
			bool show_radon_intermediate=GetSet<bool>("Advanced/Display Radon Intermediate Function", gui_section().subsection("Input"));
			if (show_radon_intermediate)
			{
				dtr_input->readback();
				UtilsQt::Figure("Input Radon Intermediate Function", dtr_input->data(),0,0, dtr_input->isDerivative() );
			}
		}
		// Store current data in Ps and dtrs
		Ps  =reference.getProjectionMatrices();
		dtrs=reference.getRadonIntermediateFunctions();
		Ps.push_back(P_input);
		dtrs.push_back(dtr_input);
		// Visualize Geometry
		bool show_geometry=GetSet<bool>("Advanced/Display Current Geometry", gui_section().subsection("Input"));
		if (show_geometry)
		{
			UtilsQt::Figure geometry("Geometry",800,600);
			auto& cams=geometry.overlay();
			Eigen::Vector4d image_rect(0,0,dtr_input->getOriginalImageSize(0),dtr_input->getOriginalImageSize(1));
			EpipolarConsistency::InputDataDirect input;
			input.gui_retreive_section(gui_section().subsection("Input"));
			double spacing=input.advanced.mm_per_px;
			for (int i=0;i<(int)Ps.size()-1;i++)
				cams.add(GraphicsItems::ConvexMesh::Camera(Ps[i],image_rect,spacing,false,GraphicsItems::colorByIndex(i,64)));
			cams.add(GraphicsItems::FancyCamera(Ps.back(),image_rect,spacing,QColor(0,0,0,192)));
			cams.add(GraphicsItems::ConvexMesh::Cube());
		}
		return true;
	}

	virtual void gui_notify(const std::string& section, const GetSetInternal::Node& node)
	{

		if (section=="Input/Projection" && node.name=="Images")
			GetSet<int>("File Index", gui_section().subsection("Input"))=0;
	
		if (node.name=="File Index")
		{
			// Make sure we reload shit
			if (dtr_input)
				delete dtr_input;
			dtr_input=0x0;
			assertData();
		}
		if (node.name=="Cost Function")
		{
			if (!assertData()) return;
			auto active_set=parameter_gui.getActiveSet();
			std::cout << P_input << std::endl << std::endl;
			ParameterModel model(P_input,active_set);
			EpipolarConsistency::SingleImageMotion track(model,Ps,dtrs,(int)Ps.size()-1);
			EpipolarConsistency::plotCostFunction(g_app,"",track,100,
				LibOpterix::restrict_vector(parameter_gui.getLowerBounds(),active_set),
				LibOpterix::restrict_vector(parameter_gui.getUpperBounds(),active_set));
		}
	}
		
	/// Temporarily ignore notifications
	virtual void gui_ignore_notify(bool ignore=true) const
	{
		reference.gui_ignore_notify(ignore);
		GetSetGui::Object::gui_ignore_notify(ignore);
	}

	FluoroTrackingGUI(const GetSetGui::Section& section, GetSetGui::ProgressInterface* app=0x0)
		: GetSetGui::Object(section,app)
		, reference(section.subsection("Reference"),app)
		, parameter_gui(ParameterModel::ParameterNames(), section.subsection("Optimization/Parameters") )
		, dtr_input(0x0)
	{
		gui_init();
	}

	~FluoroTrackingGUI()
	{
		if (dtr_input)
			delete dtr_input;
	}

};

FluoroTrackingGUI tracking_gui("",&g_app);

/// User info during optimization.
void debug_hook(double current_cost, LibOpterix::AbstractOptimizationProblem* problem)
{


}

void gui(const GetSetInternal::Node& node)
{
	// Allow user to edit and save plots as pdf
	if (node.name=="Show Plot Editor...")
		UtilsQt::showPlotEditor();
	
	// Save settings to ini-file
	g_app.saveSettings();
}

int main(int argc, char ** argv)
{
	// Menu and about text
	tracking_gui.gui_ignore_notify(true);
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
		"<a href=\"https://www5.cs.fau.de/research/software/epipolar-consistency/\">Pattern Recognition Lab at Technical University of Erlangen-Nuremberg</a> "
		"<br>"
		"<h4>Licensed under the Apache License, Version 2.0 (the \"License\")</h4>\n\n"
		"You may not use this file except in compliance with the License. You may obtain a copy of the License at "
		"<a href=\"http://www.apache.org/licenses/LICENSE-2.0\">http://www.apache.org/licenses/LICENSE-2.0</a><br>"
		;
	tracking_gui.gui_ignore_notify(false);
	return g_app.exec();
}
