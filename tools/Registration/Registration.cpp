// Created by A. Aichert on Mon Aug 04th 2014
// Raw data domain registration of CT scans

// Managing saving/loading parametzers and automatic GUI
#include <GetSetGui/GetSetGui.h>
#include <GetSetGui/GetSetTabWidget.h>

// Plotting and Visualization
#include <LibUtilsQt/QCameraView.h>
#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>
#include <LibUtilsQt/QGLPlot3D.h>

using UtilsQt::Figure;
using UtilsQt::Plot;

// Timing cost function evaluations when plotting
#include <Utils/TimerWin32.hxx>

// Utility functions for frequently used loading/visualization routines and GUI.
#include <LibEpipolarConsistency/Gui/PreProccess.h>
#include <LibEpipolarConsistency/Gui/InputDataRadonIntermediate.h>
#include <LibEpipolarConsistency/Gui/Visualization.h>
#include "Registration3D3D.hxx"

using Geometry::ProjectionMatrix;

// Optimization
#include <LibOpterix/WrapNLOpt.hxx>

// Parametrization of Transformation
#include <LibProjectiveGeometry/Models/ModelSimilarity3D.hxx>
#include <LibOpterix/ParameterGui.hxx>
#include <LibOpterix/WrapNLOpt.hxx>

#include <Utils/Projtable.hxx>

GetSetGui::Application g_app("Registration");

inline Eigen::Vector3d pointOnSphere(double longitude, double latitude)
{
	using std::cos;
	using std::sin;
	return Eigen::Vector3d(cos(latitude)*cos(longitude),sin(latitude),cos(latitude)*sin(longitude));
}

std::vector<std::pair<double,double> > makeFibonacciSphere(int n)
{
	using Geometry::Pi;
	const double phi = 0.5*sqrt(5)-0.5; // golden ratio
	const double ga = phi*2*Pi; // golden angle
	std::vector<std::pair<double,double> > pts(n);
	for (int i = 1; i <= n; ++i)
	{
		double lng = ga*i;
		lng /= 2*Pi; lng -= floor(lng); lng *= 2*Pi;
		if (lng > Pi)  lng -= 2*Pi;
 		double lat = asin(-1 + 2*i/(double)n);
		pts[i-1] = std::make_pair(lng,lat);
	}
	return pts;
}

void showGeometry(const EpipolarConsistency::Registration3D3D& reg)
{
	Figure figure("Geometry",800,600);
	int n_source=reg.getTemporaryResults().size(0);
	int n_target=reg.getTemporaryResults().size(1);
	auto Ps=reg.getMetricPtr().getProjectionMatrices();
	auto& source=figure.overlay().set("Source",GraphicsItems::Group() );
	auto& target=figure.overlay().set("Target",GraphicsItems::Group() );

	double spacing=.308;               // FIXME HARDCODED
	Eigen::Vector4d llur(0,0,1024,860);// FIXME HARDCODED

	for (int i=0;i<n_source;i++)
		source.add(GraphicsItems::ConvexMesh::Camera(Ps[i],llur,spacing,false,QColor(0,0,0,255)) );
	for (int j=0;j<n_target;j++)
		source.add(GraphicsItems::ConvexMesh::Camera(Ps[n_source+j],llur,spacing,false,QColor(0,0,255,128)) );
}

/// User info during optimization.
bool debug_hook_slow=false;
void debug_hook(double current_cost, LibOpterix::AbstractOptimizationProblem* problem)
{
	auto *reg_ptr=dynamic_cast<EpipolarConsistency::Registration3D3D*>(problem);
	if (debug_hook_slow && reg_ptr)
	{
		auto& reg(*reg_ptr);
		UtilsQt::Plot("Cost by Iteration").graph(0).addData(std::log(current_cost));
		Figure("Cost Image",reg.getTemporaryResults());
		showGeometry(reg);
	}
	else
		QCoreApplication::processEvents();
}

class RegistrationGUI : public GetSetGui::Object {
protected:
	EpipolarConsistency::InputDataRadonIntermediateGui source;
	EpipolarConsistency::InputDataRadonIntermediateGui target;
	LibOpterix::ParameterGUI parameter_gui;
		
public:
	virtual void gui_init()
	{
		GetSetGui::Section optimization("Optimization",gui_section());
		GetSetGui::Button("Actions/Plot",optimization)="Plot...";
		GetSetGui::Button("Actions/Optimize",optimization)="Run Optimization...";
		GetSetGui::Button("Actions/Randomize",optimization)="Randomize Parameters...";
		GetSetGui::Button("Actions/Reset",optimization)="Reset Parameters...";
		GetSet<bool>("Show Intermediate Results",optimization)=true;
		GetSetGui::Enum("Signal Comparison",optimization).setChoices("SSD;Correlation");
		optimization.subsection("Actions").setGrouped();
		parameter_gui.declareValues();
		parameter_gui.declareActive();
		parameter_gui.declareBounds();
		UtilsNLOpt::OptimizerSettings(UtilsNLOpt::algorithm_names("LN")).gui_declare_section(optimization);
		GetSetGui::Section alignment("Alignment",gui_section());
		GetSetGui::Button("Actions/Optimize",alignment)="Run Optimization...";
		GetSetGui::Button("Actions/Randomize",alignment)="Randomize Parameters...";
		GetSetGui::Button("Actions/Plot",alignment)="Plot Ball...";
		alignment.subsection("Actions").setGrouped();
		GetSet<double>("Muti-Start/Translation (max)",alignment,100);
		GetSet<int>("Muti-Start/Orientatinos",alignment)=1001;
		GetSet<int>("Muti-Start/Roll",alignment)=180;
		alignment.subsection("Muti-Start").setGrouped();
		GetSet<Geometry::RP3Homography>("Parameters/Transformation",gui_section(),Geometry::RP3Homography::Identity());
	}

	bool assertData()
	{
		if (!source.loadData(false) || !target.loadData(false) ) {
			app->warn("Failed to evaluate cost function!","Please make sure that both source and target projections are loaded correctly.");
			return false;
		}	
		return true;
	}

	virtual void gui_notify(const std::string& section, const GetSetInternal::Node& node)
	{
		if (hasPrefix(section,"Parameters") && node.name!="Transformation")
		{
			Geometry::ModelSimilarity3D model;
			model.setValues(parameter_gui.getValues().data());
			GetSet<Geometry::RP3Homography>("Parameters/Transformation",gui_section())=model.getInstance();
		}

		if (node.name=="Algorithm")
		{
			std::cout << nlopt::algorithm_name(UtilsNLOpt::nlopt_algorithm_by_str(node.getString())) << std::endl;
			
			std::cout << "starting guess (expanded) : " << vectorToString(parameter_gui.getValues()) <<  std::endl;
			Geometry::ModelSimilarity3D model;
			model.setValues(parameter_gui.getValues().data());
			
			std::cout << "Tmm = [ " << model.getInstance() << " ]" <<  std::endl;
			return;
		}

		if (node.name!="Optimize"
		&&	node.name!="Randomize"
		&&	node.name!="Plot"
		&&	node.name!="Reset"
			) return;

		// Close old plots (if any)
		UtilsQt::Plot("Cost by Iteration").close();// no show
		UtilsQt::Plot("Cost by Iteration").setAxisLabels("Iteration Number","Log Epipolar Inconsistency [a.u.]");

		// Load data if required
		if (!assertData()) return;

		///////////////////////////
		// Global Optimization: (uses +/- Pi for rotation parameters)
		if (section == "Alignment" || section == "Alignment/Actions")
		{
			
			if (node.name=="Randomize")
			{
				auto lb_orig=parameter_gui.getLowerBounds();
				auto ub_orig=parameter_gui.getUpperBounds();
				GetSetGui::Section alignment("Alignment",gui_section());
				double t_max=GetSet<double>("Muti-Start/Translation (max)",alignment);
				std::vector<double> lb(7,-t_max), ub(7,t_max);
				lb[6]=ub[6]=0;
				lb[3]=lb[4]=lb[5]=-Geometry::Pi;
				ub[3]=ub[4]=ub[5]=+Geometry::Pi;
				parameter_gui.setLowerBounds(lb);
				parameter_gui.setUpperBounds(ub);
				parameter_gui.randomize();
				parameter_gui.setLowerBounds(lb_orig);
				parameter_gui.setUpperBounds(ub_orig);
			}

				
			if (node.name=="Plot")
			{
				std::set<int> translation3d; //  FIXME wh are ParaeterSets not working?!
				translation3d.insert(0);
				translation3d.insert(1);
				translation3d.insert(2);
				Geometry::ModelSimilarity3D model(translation3d);
				bool use_cc=GetSet<int>("Optimization/Signal Comparison",gui_section());
				EpipolarConsistency::Registration3D3D reg(
					model,use_cc,
					source.getProjectionMatrices(),
					source.getRadonIntermediateFunctions(),
					target.getProjectionMatrices(),
					target.getRadonIntermediateFunctions()
				);
				std::vector<double> param(6,0.0);
				int n_x=384;
				int n_y=512;
				int n_z=5;
				NRRD::Image<float> cost(n_x,n_y,n_z);
				bool cancel_clicked=false;
				g_app.progressStart("Computing Cost Images","Sampling all rotations.",n_z,&cancel_clicked);
				for (int z=0;z<n_z;z++)
				{
					for (int y=0;y<n_y;y++)
						for (int x=0;x<n_x;x++)
						{
							using Geometry::Pi;
							param[3]=(double)x/n_x*Pi*2-Pi;
							param[4]=(double)y/n_y*Pi*2-Pi;
							param[5]=(double)(z+1)/ (n_z*2)*Pi*2-Pi;
							model.expand(param.data());
							cost.pixel(x,y,z)=reg.evaluate(model.getInstance());
							showGeometry(reg);
							UtilsQt::Figure("Cost Image",NRRD::ImageView<float>(cost,z));
						}
					g_app.progressUpdate(z);
					if (cancel_clicked) break;
				}
				g_app.progressEnd();
				if (!cancel_clicked)
					UtilsQt::Figure("Cost Image",cost);

			}

			if (node.name=="Optimize")
			{
				std::cout << "Starting exhaustive search...\n";
				std::set<int> rigid3d; //  FIXME wh are ParaeterSets not working?!
				rigid3d.insert(0);
				rigid3d.insert(1);
				rigid3d.insert(2);
				rigid3d.insert(3);
				rigid3d.insert(4);
				rigid3d.insert(5);
				Geometry::ModelSimilarity3D model(rigid3d);
				bool use_cc=GetSet<int>("Optimization/Signal Comparison",gui_section());
				EpipolarConsistency::Registration3D3D reg(
					model,use_cc,
					source.getProjectionMatrices(),
					source.getRadonIntermediateFunctions(),
					target.getProjectionMatrices(),
					target.getRadonIntermediateFunctions()
				);
				GetSetGui::Section alignment("Alignment",gui_section());
				// For local optimization
				GetSetGui::Section optimization("Optimization",gui_section());
				UtilsNLOpt::OptimizerSettings opt_settings(UtilsNLOpt::algorithm_names("LN"));
				opt_settings.gui_retreive_section(optimization);
				// Establish rotation parameters for search
				using Geometry::Pi;
				double t_max=GetSet<double>("Muti-Start/Translation (max)",alignment);
				int n_ori=GetSet<int>("Muti-Start/Orientatinos",alignment);
				int n_roll=GetSet<int>("Muti-Start/Roll",alignment);
				NRRD::Image<Eigen::Vector3f> rparam(n_ori,n_roll);
				auto lat_long=makeFibonacciSphere(n_ori);
				for (int o=0;o<n_ori;o++)
					for (int r=0;r<n_roll;r++)
						rparam.pixel(o,r)=Eigen::Vector3f(lat_long[o].first,lat_long[o].second,Pi*2.0*(double)r/n_roll);
				// Start process of optimizing translation for each oriantation/roll
				bool cancel_clicked=false;
				NRRD::Image<float> opt_tx(n_ori,n_roll), opt_ty(n_ori,n_roll), opt_tz(n_ori,n_roll);
				NRRD::Image<float> ecc_by_angles(n_ori,n_roll);
				int best_idx=0;
				std::vector<double> lb(6,-t_max);
				std::vector<double> ub(6, t_max);
				lb[3]=lb[4]=lb[5]=-2.0*Pi/n_roll;
				ub[3]=ub[4]=ub[5]=+2.0*Pi/n_roll;

				UtilsQt::Figure("rparam",NRRD::ImageView<float>(3,n_ori,n_roll,(float*)&(rparam[0])));

				int l=rparam.length();
				g_app.progressStart("Searching Rotation Space...", "Sampling ECC over Fibonacci lattice and roll angle.",l,&cancel_clicked);
				Utils::TimerWin32 time;
				for (int i=0;i<l;i++)
				{
					std::vector<double> zero(7,0.0);
					model.setValues(zero.data());
					model.setValue(3,(double)rparam[i][0]);
					model.setValue(4,(double)rparam[i][1]);
					model.setValue(5,(double)rparam[i][2]);
					std::vector<double> x (4, 0.0);
					x[3]=(double)rparam[i][2];
					lb[3]=x[3]-2.0*Pi/n_roll;
					ub[3]=x[3]+2.0*Pi/n_roll;
					UtilsNLOpt::Optimizer opt(reg,opt_settings,lb,ub);
					GetSetGui::ProgressInterface tmp;
					opt.optimize(x,tmp);
					opt_tx[i]=x[0];
					opt_ty[i]=x[1];
					opt_tz[i]=x[2];
					rparam[i][2]=x[3];
					model.expand(x.data());
					double optimum=reg.evaluate(model.getInstance());
					ecc_by_angles[i]=optimum;
					if (ecc_by_angles[i]<ecc_by_angles[best_idx])
					{
						best_idx=i;
						showGeometry(reg);
						std::cout << "x = [" << vectorToString(x) << " ] % ECC = " << optimum << " \n";
						UtilsQt::Figure("Cost Image", reg.getTemporaryResults());
					}
					if (i%10==0)
						g_app.progressUpdate(i);
					if (cancel_clicked) break;
				}
				g_app.progressEnd();
				std::cout << "time = "  << (int) time.getTotalTime()/60 << " min " << ((int)time.getTotalTime())%60 << " sec\n";
				if (!cancel_clicked)
				{
					// Set result in GUI
					std::vector<double> result(7,0.0);
					result[0]=opt_tx[best_idx];
					result[1]=opt_ty[best_idx];
					result[2]=opt_tz[best_idx];
					result[3]=rparam[best_idx][0];
					result[4]=rparam[best_idx][1];
					result[5]=rparam[best_idx][2];
					std::cout << "result = ["  << vectorToString(result) << "]" << std::endl;
					parameter_gui.setValues(result); // FIXME setValues function is nonsensical. Use restrict(arg).
					model.setValues(result.data());
					std::cout << "best_ecc = " << reg.evaluate(model.getInstance()) << std::endl;
					showGeometry(reg);
					UtilsQt::Figure("Cost Image", reg.getTemporaryResults());
				}
			}
		}

		///////////////////////////
		// Local-only Optimization:
		if (section == "Optimization" || section == "Optimization/Actions")
		{	
			auto active_set=parameter_gui.getActiveSet();
			Geometry::ModelSimilarity3D model(active_set);
			bool use_cc=GetSet<int>("Optimization/Signal Comparison",gui_section());
			EpipolarConsistency::Registration3D3D reg(
				model,use_cc,
				source.getProjectionMatrices(),
				source.getRadonIntermediateFunctions(),
				target.getProjectionMatrices(),
				target.getRadonIntermediateFunctions()
				);
			
			Utils::TimerWin32 time;

			if (node.name=="Plot")
			{
				GetSetGui::Section optimization("Optimization",gui_section());
				debug_hook_slow=GetSet<bool>("Show Intermediate Results",optimization);
				EpipolarConsistency::plotCostFunction(g_app,"",reg,201,
					LibOpterix::restrict_vector(parameter_gui.getLowerBounds(),active_set),
					LibOpterix::restrict_vector(parameter_gui.getUpperBounds(),active_set),
					debug_hook,
					LibOpterix::restrict_vector(parameter_gui.getValues(),active_set) );
				std::cout << "ecc = " << reg.evaluate(Geometry::RP3Homography::Identity()) << std::endl;
				UtilsQt::Figure("Cost Image",reg.getTemporaryResults());
				return;
			}

			if (node.name=="Optimize")
			{
				GetSetGui::Section optimization("Optimization",gui_section());
				UtilsNLOpt::OptimizerSettings opt_settings(UtilsNLOpt::algorithm_names("LN"));
				opt_settings.gui_retreive_section(optimization);
				// Make bounds relative to current value and set bound constraints.
				auto x =parameter_gui.getActiveValues();
				auto lb=parameter_gui.getActiveLowerBounds();
				auto ub=parameter_gui.getActiveUpperBounds();
				for (int i=0;i<(int)x.size();i++) {
					lb[i]+=x[i];
					ub[i]+=x[i];
				}
				UtilsNLOpt::Optimizer opt(reg,opt_settings,lb,ub);
				// Set debug hook for intermediiate information.
				debug_hook_slow=GetSet<bool>("Show Intermediate Results",optimization);
				opt.setCallback(debug_hook);
				// Run optimization and show results in GUI
				std::cout << "before=[" << vectorToString(x) << "]\n";
				opt.optimize(x,g_app);
				std::cout << "after =[" << vectorToString(x) << "]\n";
				parameter_gui.setActiveValues(x);
			}
			if (node.name=="Randomize")
				parameter_gui.randomize();
			if (node.name=="Reset")
				parameter_gui.reset();
			
			// Update graphics.
			model.expand(parameter_gui.getActiveValues().data());
			std::cout << "Tmm  = \n" << model.getInstance() << std::endl;
			std::cout << "ecc  = " << reg.evaluate(model.getInstance()) << std::endl;
			std::cout << "time = " << time.getTotalTime() << std::endl;
			showGeometry(reg);
			ProjTable::saveProjectionsOneMatrixPerLine(reg.getMetricPtr().getProjectionMatrices(), "transformed_trajectory.ompl");
		}

	}
		
	/// Temporarily ignore notifications
	virtual void gui_ignore_notify(bool ignore=true) const
	{
		source.gui_ignore_notify(ignore);
		target.gui_ignore_notify(ignore);
		GetSetGui::Object::gui_ignore_notify(ignore);
	}

	RegistrationGUI(const GetSetGui::Section& section, GetSetGui::ProgressInterface* app=0x0)
		: GetSetGui::Object(section,app)
		, source(section.subsection("Source"),app)
		, target(section.subsection("Target"),app)
		, parameter_gui(Geometry::ModelSimilarity3D::ParameterNames(), section.subsection("Parameters"),true )
	{}

};

RegistrationGUI registration("",&g_app);

void gui(const GetSetInternal::Node& node)
{
	using UtilsQt::Figure;
	// Allow user to edit and save plots as pdf
	if (node.name=="Show Plot Editor...")
		UtilsQt::showPlotEditor();

	// Save settings to ini-file
	g_app.saveSettings();
}

int main(int argc, char ** argv)
{
	// Menu and about text
	registration.gui_ignore_notify(true);
	registration.gui_init();
	g_app.init(argc,argv,gui);
	g_app.window().addMenuItem("File","Show Plot Editor...");
	g_app.window().addMenuItem("File"),
	g_app.window().addDefaultFileMenu();
	g_app.window().aboutText()=
		"<h4>Registration of two CTs in projection data domain.</h4>\n\n"
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

	registration.gui_ignore_notify(false);

	return g_app.exec();
}
