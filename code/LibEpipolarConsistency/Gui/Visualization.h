#ifndef _ecc_visualization_hxx
#define _ecc_visualization_hxx

#include <algorithm>

#include <GetSet/GetSetObjects.h>

#include <LibEpipolarConsistency/EpipolarConsistencyRadonIntermediate.h>

#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>
#include <LibUtilsQt/QGLPlot3D.h>
#include <LibOpterix/Opterix.hxx>

namespace EpipolarConsistency {

	/// Show cost image and plot costs by view. Returns costs.
	double showCostImage(const std::string& name, EpipolarConsistency::MetricRadonIntermediate& ecc, bool mirror=false)
	{
		// Compute Epipolar Consistency
		int n=ecc.getNumberOfProjetions();
		NRRD::Image<float> cost_image(n,n);
		#pragma omp parallel for
		for (int i=0;i<n*n;i++)
			cost_image[i]=0.0f;
		double cost=ecc.evaluate(cost_image);
		float minv=cost_image[1],maxv=cost_image[1];
		#pragma omp parallel for
		for (int y=0;y<n;y++)
			for (int x=y+1;x<n;x++)
			{
				float sample=cost_image.pixel(y,x);
				if (sample<minv) minv=sample;
				if (sample>maxv) maxv=sample;
				if (mirror)
					cost_image.pixel(x,y)=sample;
			}

		// Display Image (at reasonable scale)
		double scale=1;
		while (n*scale<512)
			scale+=1.0;
		UtilsQt::Figure(name,cost_image,0,0,scale)
			.drawLine(0,0,n,n,QColor(255,0,0))
			.drawText(std::string("ecc = ")+toString(cost)+" (" + toString(minv) + " + " + toString(maxv-minv) + ")",0,0,QColor(255,0,0));

		// Plot By View
		std::vector<double> cost_by_view(n,0.0);
		for (int y=0;y<n;y++)
			for (int x=y+1;x<n;x++)
				cost_by_view[x]+=cost_image.pixel(y,x)/(double)n;
		UtilsQt::Plot(name+" Cost by View",cost_by_view)
			.setAxisLabels("Projection","Inconsistency")
			.graph(0).setName("Epipolar Consistency");
		return cost;
	}

	/// Plot cost function for active parameters
	void plotCostFunction(
		GetSetGui::ProgressInterface& app,
		std::string window_title,
		LibOpterix::AbstractOptimizationProblem& cost_function,
		int steps,
		const std::vector<double>& lower_bounds_active,
		const std::vector<double>& upper_bounds_active,
		void (*debug_hook)(double, LibOpterix::AbstractOptimizationProblem*)=0x0,
		std::vector<double> current_estimate=std::vector<double>()
		)
	{
		bool cancel_clicked=false;
		auto& parameter_model(cost_function.abstract_model);
		auto  active_set(parameter_model.activeParameters());
		auto  active_names=parameter_model.parameterNamesActive();
		int   n_active=(int)active_set.size();
		int   i_active=0;
		std::set<std::string> open_plots;
		// 1D cost function plot for each parameter p
		for (int p=0;p<(int)active_set.size();p++)
		{
			// FIXME check current_estimate for empty or size n_active
			if (cancel_clicked) break;
			std::string name=active_names[p];
			app.progressStart("Plotting...",std::string("Plotting ")+name+" in range [" + toString(lower_bounds_active[p])+", "+toString(upper_bounds_active[p]) +"] ...",n_active*steps,&cancel_clicked); // 2do info?
			std::vector<double> restricted(n_active,0.0);
			std::vector<double> plot_x(steps);
			std::vector<double> plot_y(steps);
			if (!current_estimate.empty())
				restricted=current_estimate;
			for (int i=0;i<steps;i++)
			{
				double rel_i=(double)i/(steps-1);
				plot_x[i]=lower_bounds_active[p]*(1.0-rel_i)+upper_bounds_active[p]*rel_i;
				restricted[p]=(current_estimate.empty()?0:current_estimate[p])+plot_x[i];
				plot_y[i]=cost_function.evaluate(restricted);
				if (debug_hook)
					debug_hook(plot_y[i],&cost_function);
				app.progressUpdate(i+p*steps);
			}
			std::vector<std::string> words=stringToVector<std::string>(name,' ');
			std::string plot_title=window_title+(words.empty()?"":" "+words[0]);
			if (open_plots.find(plot_title)==open_plots.end())
				UtilsQt::Plot(plot_title).close();
			open_plots.insert(plot_title);
			UtilsQt::Plot(plot_title)
				.setAxisLabels("Parameter Value (realtive)","Consistency [a.u.]")
				.addGraph(name,plot_x,plot_y)
				.showTiled((int)open_plots.size()-1);
			if (!words.empty() && words[0]=="Rotation")
				UtilsQt::Plot(plot_title).setAxisAngularX();
		}
		app.progressEnd();
	}

	struct Plot1D : public GetSetGui::Configurable
	{
		Plot1D(const std::vector<std::string>& _parameter_names) : parameter_names(_parameter_names) {}

		const std::vector<std::string> parameter_names;

		/// Parameters describing a 1D plot
		struct Axis {
			int param_index=0;
			Eigen::Vector2d range=Eigen::Vector2d(-100,100);
			int steps=200;
		} axis;

		/// Declare default values.
		void gui_declare_section(const GetSetGui::Section& section)
		{
			GetSet<Eigen::Vector2d>("Range"    ,section, axis.range);
			GetSet<int>            ("Steps"    ,section, axis.steps);
			GetSetGui::Enum        ("Parameter",section, axis.param_index)
				.setChoices(parameter_names);
		}
	
		// Retreive current values from GUI
		void gui_retreive_section(const GetSetGui::Section& section)
		{
			axis.param_index=GetSet<int>            ("Parameter",section);
			axis.range      =GetSet<Eigen::Vector2d>("Range"    ,section);
			axis.steps      =GetSet<int>            ("Steps"    ,section);
		}

		std::string plot(GetSetGui::ProgressInterface& app,LibOpterix::AbstractOptimizationProblem& cost_function)
		{
			bool cancel_clicked=false;
			auto& parameter_model(cost_function.abstract_model);
			auto& parameters=parameter_model.expand();
			double original_value=parameters[axis.param_index];
			std::string name=parameter_model.parameterNames()[axis.param_index];
			app.progressStart("Plotting...",name,axis.steps,&cancel_clicked);
			std::vector<double> plot_x(axis.steps);
			std::vector<double> plot_y(axis.steps);
			for (int i=0;i<axis.steps;i++)
			{
				if (cancel_clicked) break;
				double rel_i=(double)i/(axis.steps-1);
				plot_x[i]=parameters[axis.param_index]=axis.range[0]*(1.0-rel_i)+axis.range[1]*rel_i;
				plot_y[i]=cost_function.evaluate(parameter_model.restrict());
				app.progressUpdate(i);
			}
			app.progressEnd();
			UtilsQt::Plot(name,plot_x,plot_y).setAxisLabels("Parameter Value","Consistency [a.u.]");
			return name;
		}
		
	};
	
	struct Plot2D : public GetSetGui::Configurable
	{
		Plot2D(const std::vector<std::string>& _parameter_names)
			: axis0(_parameter_names), axis1(_parameter_names) {}


		/// Describe the 2D Plot by two PlotAxes
		Plot1D axis0;
		Plot1D axis1;

		/// Declare default values.
		void gui_declare_section(const GetSetGui::Section& section)
		{
			GetSetGui::Section s0("Axis 0",section);
			GetSetGui::Section s1("Axis 1",section);
			axis0.gui_declare_section(s0);
			axis1.gui_declare_section(s1);
			s0.setGrouped();
			s1.setGrouped();
		}
	
		// Retreive current values from GUI
		void gui_retreive_section(const GetSetGui::Section& section)
		{
			axis0.gui_retreive_section(GetSetGui::Section("Axis 0",section));
			axis1.gui_retreive_section(GetSetGui::Section("Axis 1",section));		
		}

		UtilsQt::QGLPlot3D * plot(GetSetGui::ProgressInterface& app, LibOpterix::AbstractOptimizationProblem& cost_function)
		{
			bool cancel_clicked=false;
			auto& parameter_model(cost_function.abstract_model);
			auto& parameters=parameter_model.expand();
			double original_values[2]={
				parameters[axis0.axis.param_index],
				parameters[axis1.axis.param_index]
				};
			NRRD::Image<float> plot(axis0.axis.steps,axis1.axis.steps);
			std::string name=
				  parameter_model.parameterNames()[axis0.axis.param_index]+" versus "
				+ parameter_model.parameterNames()[axis1.axis.param_index];
			app.progressStart("Plotting...",name,axis1.axis.steps,&cancel_clicked);
			for (int y=0;y<axis1.axis.steps;y++)
			{
				app.progressUpdate(y);
				for (int x=0;x<axis0.axis.steps;x++)
				{
					if (cancel_clicked) break;
					double rel_x=(double)x/(axis0.axis.steps-1);
					double rel_y=(double)y/(axis1.axis.steps-1);
					parameters[axis0.axis.param_index]=axis0.axis.range[0]*(1.0-rel_x)+axis0.axis.range[1]*rel_x;
					parameters[axis1.axis.param_index]=axis1.axis.range[0]*(1.0-rel_y)+axis1.axis.range[1]*rel_y;
					plot.pixel(x,y)=cost_function.evaluate(parameter_model.restrict());
				}
			}
			app.progressEnd();
			// Show 3D plot
			UtilsQt::QGLPlot3D *plot_window=new UtilsQt::QGLPlot3D(plot.size(0),plot.size(1),plot,axis0.axis.range[0],axis0.axis.range[1],axis1.axis.range[0],axis1.axis.range[1]);
			plot_window->setLabel(parameter_model.parameterNames()[axis0.axis.param_index],parameter_model.parameterNames()[axis1.axis.param_index]);
			plot_window->show();
			return plot_window;
		}
		
	};


} // EpipolarConsistency

#endif // _ecc_visualization_hxx

