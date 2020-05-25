// Created by Andre Aichert

#include <iostream>
#include <vector>

// Automatic GUI
#include <GetSetGui/GetSetGui.h>
#include <GetSetGui/GetSetTabWidget.h>
GetSetGui::Application g_app("FDCTCalibrationCorrection");

// Libaries for Projective Geometry
#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibProjectiveGeometry/Models/ModelFDCTCalibrationCorrection.hxx>

// GPU
#include <LibEpipolarConsistency/EpipolarConsistencyRadonIntermediate.h>
#include <LibEpipolarConsistency/EpipolarConsistency.h>

////////////
// Utility
////////////

// File format for projection matrices
#include <Utils/Projtable.hxx>
// Display NRRD images
#include <Utils/QtNrrd.hxx>
// Plot 1D arrays
#include <Utils/QCPUtils.hxx>
// Timing
#include <Utils/TimerWin32.hxx>
// Optimize using NLOpt library
#include <Utils/Optimization/NLOptUtils.hxx>

#include <Utils/Optimization/Parametrization/Model.hxx>

// Global input data
std::vector<Geometry::ProjectionMatrix>            g_Ps;   //< Projection matrices per image
std::vector<EpipolarConsistency::RadonDerivative*> g_dtrs; //< Radon intermediate function of projection images

/////////////////////////////////////////////////////////////////////////////////////////////
// OPTIMIZATION

struct ObjectiveFunction : public OptimizationOf<std::vector<Geometry::ProjectionMatrix>
{
	std::vector<Geometry::ProjectionMatrix>&	Ps;				//< Input Prjection Matrix
	EpipolarConsistency::Metric					metric;			//< Consistency metric
	NRRD::Image<float>							cost_image;		//< Representation of pairwise consistency

	/// Callback fuction for each optimization step. Parameters are those of objective and its return value
	void(*debug_hook)(const std::vector<double> &params, std::vector<double> &grad, void *data, double cost);

	// Constructor
	ObjectiveFunction(
		// private member variables 
		std::vector<Geometry::ProjectionMatrix>& _Ps,
		std::vector<EpipolarConsistency::RadonDerivative*> _dtrs,
		const Geometry::ModelFDCTCalibrationCorrection& _HT)
		: // Member Initializer List
		Ps(_Ps)
		, metric(_Ps,_dtrs)
		, HT(_HT)
		, cost_image((int)_dtrs.size(), (int)_dtrs.size(), 1)
		, debug_hook(0x0)
	{
		metric.dkappa = GetSet<double>("Data/Value Kappa").getValue();

		// Initial cost image
		int l = cost_image.length();
		for (int i = 0; i < l; i++)
			cost_image[i] = 0;
		metric.evaluate(cost_image);
		for (int i = 0; i < cost_image.size(0); i++)
			for (int j = i; j < cost_image.size(1); j++)
				cost_image.pixel(j, i) = cost_image(i,j);

	}

	double evaluate(const std::vector<double> &params)
	{
		Ps = g_Ps;
		HT.expand(params);
		HT.transform(Ps, GetSet<Eigen::Matrix3d>("Optimize/Initial Transform"));
		metric.setProjectionMatrices(Ps);
		return metric.evaluate(cost_image);
	}

	void gradient(const std::vector<double> &params, std::vector<double> &grad)
	{

		for (int i = 0; i < grad.size(); i++)
		{
			// Transform projection matrices with set of parameters
			Geometry::ModelFDCTCalibrationCorrection HT_prime = HT;
			HT_prime.expand(params);
			Ps = g_Ps;
			HT_prime.transform(Ps);
			metric.setProjectionMatrices(Ps);
			double x_i = metric.evaluate();

			//calculate parameter with step size epsilon
			std::vector<double> epsilon_estimate = params;
			epsilon_estimate[i] += g_epsilon[i];

			// Transform projection matrices with set of parameters
			HT_prime = HT;
			HT_prime.expand(epsilon_estimate);
			Ps = g_Ps;
			HT_prime.transform(Ps);
			metric.setProjectionMatrices(Ps);
			double x_iplus1 = metric.evaluate();

			grad[i] = (x_iplus1 - x_i) / g_epsilon[i]; //forward differences
		}
	}

	static double evaluate(const std::vector<double> &params, std::vector<double> &grad, void *my_func_data){
	
		ObjectiveFunction *obj = static_cast<ObjectiveFunction*>(my_func_data);

		// Evaluate
		double cost = obj->evaluate(params);

		// Gradient calculation
		if (!grad.empty())
		{
			obj->gradient(params, grad);
		}

		// call optimizer hook via debug hook
		if (obj->debug_hook) obj->debug_hook(params, grad, my_func_data, cost);

		return cost;
	}
};

/////////////////////////////////////////////////////////////////////////////////////////////////
//// OPTIMIZATION FUNCTIONS

void optimizer_hook(const std::vector<double> &params, std::vector<double> &grad, void * my_func_data, double cost)
{

	ObjectiveFunction *obj = static_cast<ObjectiveFunction*>(my_func_data);

	// show Consistency image
	int scale = 1;
	int n = obj->cost_image.size(0);
	while (obj->cost_image.size(0)*scale < 500) scale++;
	QtUtils::Figure("Consistency image", obj->cost_image, 0, 0.5/cost, scale)
		.drawLine(0,0,n,n,1,0,0);
		
	// plot cost
	g_cost_by_time.push_back(cost);
	QtUtils::plot("Cost Function", 0, (int)g_cost_by_time.size(), &g_cost_by_time[0]);

	// plot values of different number of active parameters (in same graph as the cost)
	for (int i = 0; i < params.size(); i++)
	{
		g_params[i].push_back(params[i]);
		QtUtils::plot("Cost Function", i+1, (int)g_params[i].size(), &g_params[i][0]);
	}

	// watch gradient
	if (!grad.empty())
	{
		std::cout << "gradient :" << toString(grad) << std::endl;
	}
}

void optimization()
{
	double pixel_spacing = GetSet<double>("Trajectory/Pixel Spacing [mm per pix]");
	Geometry::ModelFDCTCalibrationCorrection HT(pixel_spacing, g_Ps);

	// Retreive optimization parameters
	std::vector<double> lower_bounds;
	std::vector<double> upper_bounds;
	std::vector<double> params_settings;
	getGuiOptimizerSettings(HT, lower_bounds, upper_bounds, params_settings);
	HT.expand(params_settings); 

	// Parameter vector with number of free parameters
	std::vector<int> active = HT.getActiveParamIndices();
	int n_active = (int)active.size();
	auto active_parameters = HT.getActiveParamNames();
	if (active_parameters.size() == 0)
	{
		error("No Parameters active!", "Please select active parameters first!");
		return;
	}

	//prepare plots
	g_cost_by_time.clear();
	g_params.clear();

	QtUtils::plotDelete("Cost Function");
	QtUtils::plotGraphSetName("Cost Function", 0, "ECC");
	QCustomPlot &plot = QtUtils::plotByName("Cost Function");
	plot.rescaleAxes();
	
	QVector<QString> colors = QColor::colorNames().toVector();
	QVector<QColor> col = { Qt::red, Qt::green, Qt::blue, Qt::cyan, Qt::magenta, Qt::yellow, Qt::darkRed, Qt::darkGreen, Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow };

	for (int i = 0; i < n_active; i++)
	{
		plot.addGraph(plot.xAxis2, plot.yAxis2);
		plot.graph(i + 1)->setName(QString::fromUtf8(active_parameters[i].c_str()));
		plot.graph(i + 1)->setPen(col[i]);
	}
	plot.xAxis->setLabel("Evaluation step");
	plot.yAxis->setLabel("Cost");
	plot.yAxis2->setVisible(true);
	plot.yAxis2->setLabel("Variables");

	//setup nlopt
	nlopt::algorithm optimization_method = NLOptUtil::Algorithm::nlopt_algorithm_by_str(GetSet<>("Optimize/Method"));
	nlopt::opt opt(optimization_method, n_active);
	std::cout << "Optimizer: " << opt.get_algorithm_name() << std::endl;

	//set Bound constraints
	opt.set_lower_bounds(lower_bounds);
	opt.set_upper_bounds(upper_bounds);

	// Set up ObjectiveFunction object
	std::vector<Geometry::ProjectionMatrix> Ps = g_Ps;
	ObjectiveFunction problem(Ps, g_dtrs, HT);
	problem.debug_hook = optimizer_hook;

	// calculate epsilon (step size) for gradient calculation
	int samples = GetSet<int>("Visualize/Plot Samples");
	g_epsilon.resize(n_active);
	for (int i = 0; i < n_active; i++)
	{
			g_epsilon[i] = (upper_bounds[i] - lower_bounds[i]) / samples ; // Range/Plotsamples
	}

	// setup objective function & stopping criteria
	opt.set_min_objective(ObjectiveFunction::evaluate, (void*)&problem); //specifing objective function
	opt.set_xtol_abs(GetSet<double>("Optimize/Tolerance xtol_abs").getValue());
	opt.set_ftol_abs(GetSet<double>("Optimize/Tolerance ftol_abs").getValue());

	Utils::TimerWin32 time;
	// run optimization
	double minf;
	try
	{
		nlopt::result result = opt.optimize(params_settings, minf);

		GetSetGui::ReadOnlyText("Optimize/Total Time") = toString(time.getTotalTime());

		// get exit status
		std::string res = NLOptUtil::Algorithm::nlopts_exit_status(result);
		std::cout << "stopped optimitation : " << res << std::endl;
	}
	catch (int e)
	{
		GetSetGui::ReadOnlyText("Optimize/Total Time") = "0";

		std::cout << "An nlopt exception occurred. Exception Nr: " << e << std::endl;
	}

	HT.expand(params_settings);
	setGuiOptimizerSettings(HT);

	std::cout << "Result: \nECC min: " << minf << std::endl; 
	GetSetGui::ReadOnlyText("Optimize/ECC Value") = toString(minf);
	for (int i = 0; i < n_active; i++)
	{
		std::cout << active_parameters[i] << ": " << params_settings[i] << std::endl;
	}

	std::string mypath = GetSetGui::Directory("Data/Saving path").getString();
	mypath += "/ConsistencyImage_optimized.nrrd";
	problem.cost_image.save(mypath);
	std::cout << "Consistency image saved as: " << mypath << std::endl;
}


/////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS

double calculateEquivalentOf(std::vector<Geometry::ProjectionMatrix> Ps, int param, double value)
{
	double equi = 0;

	std::vector<double> sdds;
	ProjTable::ctCircularTrajectoryToParameters(Ps, GetSet<double>("Data/Pixel Spacing"), 0x0, 0x0, 0x0, 0x0, &sdds);
	// calculate mean
	double sumSDD = std::accumulate(sdds.begin(), sdds.end(), 0.0);
	double sdd = sumSDD / sdds.size();

	switch (param)
	{
	case 0: // Translation u -> Yaw
		equi = atan(value / sdd);
		break;
	case 1: // Translation v -> Pitch
		equi = atan(value / sdd);
		break;
	case 2: // Yaw -> Translation u
		equi = tan(value)*sdd; 
		break;
	case 3: // Pitch -> Translation v
		equi = tan(value)*sdd;
		break;
	default:
		std::cout << "No valid param Number." << std::endl;
		break;
	}
	return equi;
}

double consistency(NRRD::Image<float> &img)
{
	// Allocate black image
	img.set((int)g_dtrs.size(), (int)g_dtrs.size());
	int l = img.length();
	for (int i = 0; i < l; i++)
		img[i] = 0;

	Utils::TimerWin32 time;

	std::vector<Geometry::ProjectionMatrix> Ps = g_Ps;

	// Prepare ECC metric on GPU
	EpipolarConsistency::Metric ecc(Ps, g_dtrs);
	ecc.dkappa = GetSet<double>("Data/Value Kappa");

	std::cout << "preproc:" << time.getElapsedTime() << std::endl;

	// Evaluate
	double ecc_value= ecc.evaluate(img);
	std::cout << "ecc = " << ecc_value << std::endl;
	std::cout << "time:" << time.getTotalTime() << std::endl;

	img.save("ConsistencyImage.nrrd");
	std::cout << "Consistency image saved as: ConsistencyImage.nrrd" << std::endl;
	return ecc_value;
}

void circularTrajectory()
{
	double sid = GetSet<double>("Trajectory/Source Isocenter Distance [mm]");
	double sdd = GetSet<double>("Trajectory/Source Detector Distance [mm]");
	int n_u = std::stoi(GetSetGui::ReadOnlyText("Trajectory/Detectorsize Horizontal [pix]").getString());
	int n_v = std::stoi(GetSetGui::ReadOnlyText("Trajectory/Detectorsize Vertical [pix]").getString());
	double max_angle = GetSet<double>("Trajectory/Max Angle [deg]");
	double pixel_spacing = GetSet<double>("Trajectory/Pixel Spacing [mm per pix]");

	std::vector<Geometry::ProjectionMatrix> Ps = ProjTable::makeCircularTrajectory((int)g_dtrs.size(), sid, sdd, n_u, n_v, max_angle, pixel_spacing);
	
	// save circular Trajectory
	std::string mypath = GetSetGui::Directory("Data/Saving path").getString();
	mypath += "/circularTrajectory";
	ProjTable::saveProjectionsOneMatrixPerLine(Ps, mypath);
	std::cout << "saved " << Ps.size() << " matrices to " << mypath << std::endl;
	ProjTable::saveProjectionsOneMatrixPerLine(Ps, mypath + ".ompl");
	ProjTable::saveProjtable(Ps, mypath + ".txt");

	// reload it for optimization
	std::string file = mypath + ".ompl";
	GetSet<>("Data/Projection Matrices") = file;
	g_Ps = Ps;
	std::cout << "Loaded " << g_Ps.size() << " circular trajectory matrices." << std::endl;
}

///////////////// plot functions ////////////////////////////////////
double plot_constistency(NRRD::Image<float> &cost_image){

	// Initial cost image
	int l = cost_image.length();
	for (int i = 0; i < l; i++)
		cost_image[i] = 0;

	std::vector<Geometry::ProjectionMatrix> Ps = g_Ps;
	EpipolarConsistency::Metric ecc(g_Ps, g_dtrs);
	ecc.dkappa = GetSet<double>("Data/Value Kappa").getValue();
	double cost = ecc.evaluate(cost_image);

	//write inital cost image to upper right corner
	for (int i = 0; i < cost_image.size(0); i++)
		for (int j = i; j < cost_image.size(1); j++)
			cost_image.pixel(j, i) = cost_image(i, j);

	// Retreive optimization parameters
	std::vector<double> lower_bounds;
	std::vector<double> upper_bounds;
	std::vector<double> params_settings;

	double pixel_spacing = GetSet<double>("Trajectory/Pixel Spacing [mm per pix]");
	Geometry::ModelFDCTCalibrationCorrection HT(pixel_spacing, g_Ps);


	getGuiOptimizerSettings(HT, lower_bounds, upper_bounds, params_settings);
	if (!params_settings.empty()) HT.expand(params_settings);
	
	HT.transform(Ps);

	ecc.setProjectionMatrices(Ps);
	ecc.evaluate(cost_image);

	std::string mypath = GetSetGui::Directory("Data/Saving path").getString();
	mypath += "/ConsistencyImage.nrrd";
	cost_image.save(mypath);
	std::cout << "Consistency image saved as: " << mypath << std::endl;

	return cost;
}


void plot_cost(){

	std::cout << "Plot Cost Function for active Parameters" << std::endl;

	// Retreive optimization parameters
	std::vector<double> lower_bounds;
	std::vector<double> upper_bounds;
	std::vector<double> params_settings;

	double pixel_spacing = GetSet<double>("Trajectory/Pixel Spacing [mm per pix]");
	Geometry::ModelFDCTCalibrationCorrection HT(pixel_spacing, g_Ps);


	getGuiOptimizerSettings(HT, lower_bounds, upper_bounds, params_settings);
	HT.expand(params_settings);

	// Parameter vector with number of active parameters
	std::vector<int> active = HT.getActiveParamIndices();
	int n_active = (int)active.size();
	auto active_parameters = HT.getActiveParamNames();
	if (active_parameters.size() == 0)
	{
		error("No Parameters active!", "Please select active parameters first!");
		return;
	}

	int	n_samples = GetSet<int>("Visualize/Plot Samples").getValue();
	if (n_samples % 2 == 0) //fix for establish right size of plots
	{
		n_samples += 1;
		GetSet<int>("Visualize/Plot Samples") = n_samples;
		std::cout << "Visualize/Plot Samples was set from " << (n_samples - 1) << " to " << n_samples << std::endl;
	}
	int	k = n_samples / 2;

	// for costfunction
	std::vector<Geometry::ProjectionMatrix> Ps = g_Ps;
	EpipolarConsistency::Metric ecc(g_Ps, g_dtrs);
	ecc.dkappa = GetSet<double>("Data/Value Kappa").getValue();

	//need mean of SID and SDD for absolute values
	std::vector<double> sids, sdds;
	ProjTable::ctCircularTrajectoryToParameters(Ps, GetSet<double>("Data/Pixel Spacing"), 0x0, 0x0, 0x0, &sids, &sdds);
	// calculate mean
	double sumSID = std::accumulate(sids.begin(), sids.end(), 0.0);
	double sid = sumSID / sids.size();
	double sumSDD = std::accumulate(sdds.begin(), sdds.end(), 0.0);
	double sdd = sumSDD / sdds.size();

	// progress bar setup
	int progress_max = n_active * n_samples;
	QProgressBar p_plot;
	p_plot.setWindowTitle("Calculating consistency plot ...");
	p_plot.setMaximum(progress_max);
	p_plot.setFixedWidth(600);
	p_plot.show();

	int progress_counter = 0;

	// save cost values
	std::vector<double> param;
	std::vector<double> cost;

	//std::string name = "Cost Function"; // all in one plot
	std::vector<double> current_estimate = HT.restricted(); 
	for (int i = 0; i < n_active; i++)
	{
		std::string name = "Cost Function " + active_parameters[i];
		QtUtils::plotDelete(name);

		double d_abs = upper_bounds[i] - lower_bounds[i];
		double d_rel = upper_bounds[i] + lower_bounds[i];

		current_estimate = params_settings; //reset to primary estimation

		for (int j = -k; j <= k; j++)
		{
			p_plot.setValue(progress_counter);
			QApplication::processEvents();
			// Compute current value within bound constraints
			double x_val = ((double)j / k)*d_abs/2 + d_rel/2;
			// Transform projection matrices with updated set of parameters
			current_estimate[i] = x_val;
			Geometry::ModelFDCTCalibrationCorrection HT_prime = HT;
			HT_prime.expand(current_estimate);
			Ps = g_Ps;
			HT_prime.transform(Ps);
			ecc.setProjectionMatrices(Ps);
			// Compute ECC for current sample
			cost.push_back(ecc.evaluate());

			if (active[i] == 5) param.push_back(x_val + sid); //absolute value for SID
			else if (active[i] == 6)  param.push_back(x_val + sdd); // absolute value for SDD
			else param.push_back(x_val);
			
			progress_counter++;
		}
		QtUtils::plot(name, 0, (int)cost.size(), &param[0], &cost[0]);
		QtUtils::plotGraphSetName(name, 0, active_parameters[i]);
		QCustomPlot &plot = QtUtils::plotByName(name);

		// show current estimate
		plot.addGraph();
		plot.graph(1)->setPen(QColor(255, 100, 0));
		plot.graph(1)->setLineStyle(QCPGraph::lsNone);
		plot.graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 6));
		plot.graph(1)->setName("Current estimate");

		//prepare calculation of curent estimate
		QVector<double> x(1), y(1);
		x[0] = params_settings[i];
		if (active[i] == 5) x[0] += sid; //absolute value for SID
		else if (active[i] == 6)  x[0] += sdd; // absolute value for SDD

		// Transform projection matrices with updated set of parameters
		Geometry::ModelFDCTCalibrationCorrection HT_prime = HT;
		HT_prime.expand(params_settings);
		Ps = g_Ps;
		HT_prime.transform(Ps);
		ecc.setProjectionMatrices(Ps);
		// Compute ECC for current sample
		y[0] = (double)ecc.evaluate();
		//plot current estimate
		plot.graph(1)->setData(x, y);

		// appearance of plot
		std::string xlabel = "Correction [";
		if (active[i] == 0 || active[i] == 1) xlabel.append("pix]"); //mm oder pix ? 
		else if (active[i] == 2 || active[i] == 3 || active[i] == 4) xlabel.append("rad]");
		else if (active[i] == 5 || active[i] == 6) xlabel = "Distance [mm]";
		plot.xAxis->setLabel(QString::fromStdString(xlabel));
		plot.yAxis->setLabel("Inconsistency [a.u.]");
		plot.rescaleAxes();
		plot.replot();
		
		// saving the plot
		QString mypath = QString::fromStdString(GetSetGui::Directory("Data/Saving path").getString());
		QString png = mypath;
		png.append(QString::fromStdString("/" + name + ".png"));
		plot.savePng(png);
		QString pdf = mypath;
		pdf.append(QString::fromStdString("/" + name + ".pdf"));
		plot.savePdf(pdf, true);

		param.clear();
		cost.clear();
	}
	p_plot.close();
}

void plot2D_cost()
{
	std::cout << "Plot 2D Cost Function for active Parameters" << std::endl;

	// Parameter vector with number of selected parameters
	std::vector<int> selected;
	selected.push_back(GetSet<int>("Visualize/Costfunction 2D/Parameter 1").getValue());
	selected.push_back(GetSet<int>("Visualize/Costfunction 2D/Parameter 2").getValue());

	if (selected[0] == selected[1])
	{
		error("Parameters 1 and 2 are the same.", "Please select two different Paramters for the 2D Cost Plot");
		return;
	}

	// Retreive optimization parameters
	std::vector<double> lower_bounds;
	std::vector<double> upper_bounds;
	std::vector<double> params_settings;

	double pixel_spacing = GetSet<double>("Trajectory/Pixel Spacing [mm per pix]");
	Geometry::ModelFDCTCalibrationCorrection HT(pixel_spacing, g_Ps);


	getGuiOptimizerSettings(HT, lower_bounds, upper_bounds, params_settings);
	HT.expand(params_settings);

	std::vector<int> index = { -1, -1 };
	std::vector<int> indices = HT.getActiveParamIndices();
	for (int i = 0; i < indices.size(); i++){
		if (indices[i] == selected[0]) index[0] = i;
		if (indices[i] == selected[1]) index[1] = i;
	}

	//test if selected parameters are active. otherweise set them temporary active!
	if (index[0] == -1){
		HT.activate(selected[0]); //TODO test for uv vs angle
		
		//get where to insert the bounds 
		std::vector<int> indices = HT.getActiveParamIndices();
		for (int i = 0; i < indices.size(); i++){
			if (indices[i] == selected[0]) 
			{
				index[0] = i;
				break;
			}
		}

		//insert bound values into vecors
		auto it = upper_bounds.begin();
		it = upper_bounds.insert(it+index[0] , GetSet<double>(std::string("Parametrization/") + toString(selected[0], 2) + " " + GetSet<>("Visualize/Costfunction 2D/Parameter 1").getString() + "/Upper Bound").getValue());
		it = lower_bounds.begin();
		it = lower_bounds.insert(it+index[0] , GetSet<double>(std::string("Parametrization/") + toString(selected[0], 2) + " " + GetSet<>("Visualize/Costfunction 2D/Parameter 1").getString() + "/Lower Bound").getValue());
		
		it = params_settings.begin();
		it = params_settings.insert(it+index[0] , 0);

	}
	if (index[1] == -1){
		HT.activate(selected[1]); //TODO test for uv vs angle

		//get where to insert the bounds 
		std::vector<int> indices = HT.getActiveParamIndices();
		for (int i = 0; i < indices.size(); i++){
			if (indices[i] == selected[1])
			{
				index[1] = i;
				break;
			}
		}

		//insert bound values into vecors
		auto it = upper_bounds.begin();
		it = upper_bounds.insert(it + index[1], GetSet<double>(std::string("Parametrization/") + toString(selected[1], 2) + " " + GetSet<>("Visualize/Costfunction 2D/Parameter 2").getString() + "/Upper Bound").getValue());
		it = lower_bounds.begin();
		it = lower_bounds.insert(it + index[1], GetSet<double>(std::string("Parametrization/") + toString(selected[1], 2) + " " + GetSet<>("Visualize/Costfunction 2D/Parameter 2").getString() + "/Lower Bound").getValue());

		it = params_settings.begin();
		it = params_settings.insert(it + index[1], 0);

	}

	
	// Create plot
	int	n_samples = GetSet<int>("Visualize/Plot Samples").getValue();
	if (n_samples % 2 == 0) //fix for establish right size of plots
	{
		n_samples += 1;
		GetSet<int>("Visualize/Plot Samples") = n_samples;
		std::cout << "Visualize/Plot Samples was set from " << (n_samples - 1) << " to " << n_samples << std::endl;
	}
	int	k = n_samples / 2;
	std::vector<double> d_abs = { upper_bounds[index[0]] - lower_bounds[index[0]], upper_bounds[index[1]] - lower_bounds[index[1]] };
	std::vector<double> d_rel = { upper_bounds[index[0]] + lower_bounds[index[0]], upper_bounds[index[1]] + lower_bounds[index[1]] };

	// for costfunction
	std::vector<Geometry::ProjectionMatrix> Ps = g_Ps;
	EpipolarConsistency::Metric ecc(g_Ps, g_dtrs);
	ecc.dkappa = GetSet<double>("Data/Value Kappa").getValue();

	NRRD::Image<float> data(3, n_samples, n_samples); //new image each time
	data.setElementKindColor();

	QProgressBar p;
	p.setMinimumWidth(350);
	p.setWindowTitle("Computing 2D plot...");
	p.setMinimum(-k);
	p.setMaximum(k);
	p.setValue(-k);
	p.show();

	for (int j = -k; j <= k; j++)
	{
		for (int i = -k; i <= k; i++)
		{
			// Build parameterization of homography
			double params[] = { ((double)i / k)*d_abs[0] / 2 + d_rel[0] / 2, ((double)j / k)*d_abs[1] / 2 + d_rel[1] / 2 };
			params_settings[index[0]] = params[0];
			params_settings[index[1]] = params[1];
			// Create an object to compute geometric consistency using epipolar redundancies
			data.pixel(0, k + i, k + j) = params[0];
			data.pixel(1, k + i, k + j) = params[1];
			
			Geometry::ModelFDCTCalibrationCorrection HT_prime = HT;
			HT_prime.expand(params_settings); //TODO wo sind werte?/ ändereung der werte?
			Ps = g_Ps;
			HT_prime.transform(Ps);
			ecc.setProjectionMatrices(Ps);
			// Compute ECC for current sample
			data.pixel(2, k + i, k + j) = (double)ecc.evaluate();
		}
		p.setValue(j);
		QCoreApplication::processEvents();
	}
	p.close();
	PlotView *plot = new PlotView(n_samples, data);
	plot->setAttribute(Qt::WA_DeleteOnClose);
	plot->setLabel(GetSet<>("Visualize/Costfunction 2D/Parameter 1").getString(), GetSet<>("Visualize/Costfunction 2D/Parameter 2").getString());
	plot->show();
}

/////////////////////////////////////////////////////////////////////////////////////////////
// LOAD & SAVE DATA

void loadProjectionMatrices()
{
	//// Projection Matrices
	std::string file = GetSet<>("Data/Projection Matrices");
	std::string contents = fileReadString(file);
	g_Ps = stringToVector<Geometry::ProjectionMatrix>(contents, '\n');

	if (GetSet<bool>("Data/Load Half of the Data"))
	{
		std::vector<Geometry::ProjectionMatrix> tmp;
		for (int i = 0; i < g_Ps.size(); i += 2)
		{
			tmp.push_back(g_Ps[i]);
		}
		g_Ps = tmp;
	}

	std::cout << "Loaded " << g_Ps.size() << " matrices." << std::endl;
}

void loadData()
{
	std::cout << "" << std::endl;

	loadProjectionMatrices();

//// Radon Derivatives
	std::vector<std::string> pathRadon = GetSet<std::vector<std::string>>("Data/Radon Derivatives");
	for (int i = 0; i < (int)g_dtrs.size(); i++)
		delete g_dtrs[i];
	g_dtrs.clear();

	QProgressBar p_radon;
	p_radon.setWindowTitle("Loading Radon Derivative ...");

	if (GetSet<bool>("Data/Load Half of the Data"))
	{
		p_radon.setMaximum((int)pathRadon.size()/2 + 1);
		p_radon.setFixedWidth(600);
		p_radon.show();
		for (int i = 0; i < pathRadon.size(); i += 2){
			p_radon.setValue(i);
			QApplication::processEvents();
			g_dtrs.push_back(new EpipolarConsistency::RadonIntermediate(pathRadon[i]));
			if (!g_dtrs.back()){ // error
				std::cout << "Error: loading radon derivative" << pathRadon[i] << std::endl;
			}
		}
	}
	else{
		p_radon.setMaximum((int)pathRadon.size() - 1);
		p_radon.setFixedWidth(600);
		p_radon.show();
	for (int i = 0; i < pathRadon.size(); i++){
		p_radon.setValue(i);
		QApplication::processEvents();
		g_dtrs.push_back(new EpipolarConsistency::MetricRadonIntermediate(pathRadon[i]));
		if (!g_dtrs[i]){ // error
			std::cout << "Error: loading radon derivative" << pathRadon[i] << std::endl;
		}
	}
	}

	std::cout << "Loaded " << g_dtrs.size() << " derivatives." << std::endl;
	p_radon.close();

	//set "Trajectory/Detectorsize Vertical und Horizonatal [pix]"
	GetSetGui::ReadOnlyText("Trajectory/Detectorsize Horizontal [pix]").setValue(toString(g_dtrs[0]->getOriginalImageSize(0)));
	GetSetGui::ReadOnlyText("Trajectory/Detectorsize Vertical [pix]").setValue(toString(g_dtrs[0]->getOriginalImageSize(1)));

}


//save Projection matrices
void saveProjectionMatrices() 
{
	double pixel_spacing = GetSet<double>("Trajectory/Pixel Spacing [mm per pix]");
	Geometry::ModelFDCTCalibrationCorrection HT(pixel_spacing, g_Ps);

	std::vector<Geometry::ProjectionMatrix>  Ps = g_Ps;
	if (GetSet<bool>("Data/Load Half of the Data"))
	{
		std::string file = GetSet<>("Data/Projection Matrices");
		std::string contents = fileReadString(file);
		Ps = stringToVector<Geometry::ProjectionMatrix>(contents, '\n');
	}

	// Retreive optimization parameters
	std::vector<double> lower_bounds;
	std::vector<double> upper_bounds;
	std::vector<double> params_settings;

	getGuiOptimizerSettings(HT, lower_bounds, upper_bounds, params_settings);
	HT.expand(params_settings);
	HT.transform(Ps, GetSet<Eigen::Matrix3d>("Optimize/Initial Transform"));

	//double result1 = calculateEquivalentOf(Ps, 1, 4);
	//double result2 = calculateEquivalentOf(Ps, 3, result1);

	// Projection Matrices
	//std::string file = GetSet<>("Data/Projection Matrices");
	//std::string path = splitLeft(file, ".");
	std::string mypath = GetSetGui::Directory("Data/Saving path").getString();
	mypath += "/ProjectionMatrices_FDCTCalibrationCorrection.ompl";
	ProjTable::saveProjectionsOneMatrixPerLine(Ps, mypath);
	std::cout << "saved " << Ps.size() << " matrices to " << mypath << std::endl;
}

void saveParams()
{
	std::vector<Geometry::ProjectionMatrix>  Ps = g_Ps;

	double pixel_spacing = GetSet<double>("Trajectory/Pixel Spacing [mm per pix]");
	Geometry::ModelFDCTCalibrationCorrection HT(pixel_spacing, g_Ps);

	// Retreive optimization parameters
	std::vector<double> lower_bounds;
	std::vector<double> upper_bounds;
	std::vector<double> params_settings;

	getGuiOptimizerSettings(HT, lower_bounds, upper_bounds, params_settings);
	HT.expand(params_settings);
	HT.transform(Ps, GetSet<Eigen::Matrix3d>("Optimize/Initial Transform")); // H_ini needed to Calculate SDD correctly

	// Projection Matrices
	//std::string path = GetSet<>("Data/Projection Matrices");
	//std::string mypath = splitLeft(path, ".");
	std::string mypath = GetSetGui::Directory("Data/Saving path").getString();
	mypath += "/Parameters_FDCTCalibrationCorrection.txt";
	//ProjTable::saveProjectionsOneMatrixPerLine(Ps, path);
	std::ofstream file(mypath);
	if (file.is_open())
	{
		file << "Parametrization : \n" ;
		auto param = Geometry::ModelFDCTCalibrationCorrection::ParameterNames();

		for (int i = 0; i < HT.size(); i++)
		{
			file << "\n" + toString(i, 2) + " " + param[i] + "\n";

			if (GetSet<bool>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Active")){
				file << "Active\n";
			}
			else{
				file << "Inactive\n";
			}
			double ub = GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Upper Bound");
			file << "Upper Bound: " + toString(ub) + "\n";

			double lb = GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Lower Bound");
			file << "Lower Bound: " + toString(lb) + "\n";

			double estimate = GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Current Estimate");
			file << "Current Estimate: " + toString(estimate) + "\n";
			
			
				double calc = 0;
				bool verbose = false;
				if (i == 0){
					if ((GetSet<bool>(std::string("Parametrization/") + toString(2, 2) + " " + param[2] + "/Active"))){
						calc = calculateEquivalentOf(Ps, 2, GetSet<double>(std::string("Parametrization/") + toString(2, 2) + " " + param[2] + "/Current Estimate"));
						verbose = true;
					}
				}
				else if (i == 1){
					if ((GetSet<bool>(std::string("Parametrization/") + toString(3, 2) + " " + param[3] + "/Active"))){
						calc = calculateEquivalentOf(Ps, 3, GetSet<double>(std::string("Parametrization/") + toString(3, 2) + " " + param[3] + "/Current Estimate"));
						verbose = true;
					}
				}
				else if (i == 2){
					if ((GetSet<bool>(std::string("Parametrization/") + toString(0, 2) + " " + param[0] + "/Active"))){
						calc = calculateEquivalentOf(Ps, 0, GetSet<double>(std::string("Parametrization/") + toString(0, 2) + " " + param[0] + "/Current Estimate"));
						verbose = true;
					}
				}
				else if (i == 3){
					if ((GetSet<bool>(std::string("Parametrization/") + toString(1, 2) + " " + param[1] + "/Active"))){
						calc = calculateEquivalentOf(Ps, 1, GetSet<double>(std::string("Parametrization/") + toString(1, 2) + " " + param[1] + "/Current Estimate"));
						verbose = true;
					}
				}

				if (verbose) file << "Calcualted Value: " + toString(calc) + "\n";
		}

		file.close();
		std::cout << "saved " << HT.size() << " parameters to " << mypath << std::endl;
	}
	else std::cout << "Unable to open file" << std::endl;
}

void saveGetSet()
{
	//std::string path = GetSet<>("Data/Projection Matrices");
	//std::string mypath = splitLeft(path, ".");

	std::string mypath = GetSetGui::Directory("Data/Saving path").getString();
	mypath += "/GetSet_FDCTCalibrationCorrection.txt";

	GetSetIO::save<GetSetIO::TxtFileKeyValue>(mypath);

	std::cout << "Parameter Settings are saved to " << mypath << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// GUI & MAIN

// Retreive optimizer settings from GUI ///////////////////////////////////////////

/*getGuiOptimizerSettings just returns the parameter settings for the active parameters. */
void getGuiOptimizerSettings(Geometry::ModelFDCTCalibrationCorrection &HT, std::vector<double>& param_min_restricted, std::vector<double>& param_max_restricted, std::vector<double>& params_settings)
{
	auto param = Geometry::ModelFDCTCalibrationCorrection::ParameterNames();
	for (int i = 0; i < HT.size(); i++)
	{
		if (GetSet<bool>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Active"))
		{
			HT.activate(i);
			param_max_restricted.push_back(GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Upper Bound"));
			param_min_restricted.push_back(GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Lower Bound"));
			params_settings.push_back(GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Current Estimate"));

			std::cout << param[i] << ": Current estimate " << params_settings.back()
				<< "; Lower Bound " << param_min_restricted.back()
				<< "; Upper Bound " << param_max_restricted.back() << std::endl;
		}
		else
		{
			HT.deactivate(i);
		}
	}
}


void setGuiOptimizerSettings(Geometry::ModelFDCTCalibrationCorrection HT)
{

	auto param = Geometry::ModelFDCTCalibrationCorrection::ParameterNames();
	for (int i = 0; i < HT.size(); i++)
	{
		if (HT.active[i] == true)
		{
			GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Current Estimate") = HT.param[i];
		}
	}
}


void resetGuiOptimizerSettings(Geometry::ModelFDCTCalibrationCorrection &HT)
{

	auto param = Geometry::ModelFDCTCalibrationCorrection::ParameterNames();
	for (int i = 0; i < HT.size(); i++)
	{
		GetSet<bool>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Active") = false;
		GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Upper Bound") = 10;
		GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Lower Bound") = -10;
		GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Current Estimate") = 0;

		HT.deactivate(i);
	}
}

bool testIfCorrectlyLoaded()
{
	if (g_Ps.size() == 0){
		error("No Projection Matrices Loaded", "Please load Projection Matrices first!");
		return false;
	}
	else if (g_dtrs.size() == 0){
		error("No Radon derivatives Loaded", "Please load Radon derivatives first!");
		return false;
	}
	else if (g_Ps.size() != g_dtrs.size()){
		error("Sizes do not match", "Number of Projection Matrices and Radon Transforms do not match!");
		return false;
	}
	return true;
}

void gui(const std::string& section, const std::string& key)
{

	std::cout << section << " -> " << key << std::endl;


////////// DATA /////////////////////////////////////////////

	if (key == "Load Data")
	{
		loadData();
	}
	
	if (key == "Load Projection Matrices")
	{
		loadProjectionMatrices();
	}

	if (key == "Value Kappa"){
		/*double kappa = GetSet<double>("Data/Value Kappa");
		if (kappa < 0.002) GetSet<double>("Data/Value Kappa") = 0.002;
		if (kappa > 0.5) GetSet<double>("Data/Value Kappa") = 0.5;*/
		std::cout << "kappa value: " << GetSet<double>("Data/Value Kappa") << std::endl;
	}

	if (key == "Save")
	{
		if (g_Ps.size() == 0)
		{
			error("No Projection Matrices Loaded", "Please load Projection matrices first and optimize the Parameters!");
		}
		else
		{
			saveProjectionMatrices();
			saveParams();
			saveGetSet();
		}
	}

	if (key == "Saving path")
	{
		std::string path = GetSetGui::Directory("Data/Saving path").getString();
		std::cout << "saving path is: " << path << std::endl;
	}




///////////// OPTIMIZE //////////////////////////////////////////////////////

	if (key == "Optimization"){
		//if (GetSet<>("Optimize/Optimization").getString() == "Kill")
		//{
		//	// stop optimization
		//	std::cout << "not implemented :-P" << std::endl;
		//}
		//else
		//{
		//	GetSet<>("Optimize/Optimization") = "Kill";
		if (testIfCorrectlyLoaded())
		{
			if (GetSet<bool>("Parametrization/00 Translation u/Active") && (GetSet<bool>("Parametrization/02 Yaw/Active")))
			{
				error("Parametrization changed", "Translation u and Yaw are eqivalent.\nPlease make one of them inactive first!");
			}
			else if (GetSet<bool>("Parametrization/01 Translation v/Active") && (GetSet<bool>("Parametrization/03 Pitch/Active")))
			{
				error("Parametrization changed", "Translation v and Pitch are eqivalent.\nPlease make one of them inactive first!");
			}
			else
			{
					std::cout << "Start optimization" << std::endl;
					optimization();
			}
		}
		//}
		//GetSet<>("Optimize/Optimization")= "Optimize";
	}

	if (key == "Initial Transform")
	{
		Eigen::Matrix3d H = GetSet<Eigen::Matrix3d>("Optimize/Initial Transform");
		std::cout << "H_init " << H << std::endl;

		std::vector<Geometry::ProjectionMatrix>  Ps = g_Ps;

		for (int i = 0; i < Ps.size(); i++)
		{
			Ps[i] = H * Ps[i];
			Geometry::normalizeProjectionMatrix(Ps[i]);
		}

		EpipolarConsistency::Metric ecc(Ps, g_dtrs);
		ecc.dkappa = GetSet<double>("Data/Value Kappa");
		double cost = ecc.evaluate();

		std::cout << "ECC value: " << cost << std::endl;
	}


/////////// PARAMETERS //////////////////////////////////////////////

	if (key == "Active")
	{
		QMessageBox msgBox;
		
		if (GetSet<bool>("Parametrization/00 Translation u/Active"))
		{

			if (GetSet<bool>("Parametrization/02 Yaw/Active")){
				GetSet<bool>("Parametrization/02 Yaw/Active").setValue(false);
				GetSet<bool>("Parametrization/02 Yaw/Current Estimate").setValue(0);
				GetSet<bool>("Parametrization/00 Translation u/Current Estimate").setValue(0);

				msgBox.setText("Parametrization changed. Translation u and Yaw are eqivalent.\nYaw was set inactive. \nBoth values were set to 0!");
				msgBox.exec();
			}
		}

		if (GetSet<bool>("Parametrization/01 Translation v/Active"))
		{
			if (GetSet<bool>("Parametrization/03 Pitch/Active")){
				GetSet<bool>("Parametrization/03 Pitch/Active").setValue(false);
				GetSet<bool>("Parametrization/03 Pitch/Current Estimate").setValue(0);
				GetSet<bool>("Parametrization/01 Translation v/Current Estimate").setValue(0);

				msgBox.setText("Parametrization changed. Translation v and Pitch are eqivalent.\nPitch was set inactive. \nBoth values were set to 0!");
				msgBox.exec();
			}
		}
		auto param = Geometry::ModelFDCTCalibrationCorrection::ParameterNames();
		std::string active = "";
		for (int i = 0; i < param.size(); i++)
		{
			if (GetSet<bool>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Active"))
			{
				active.append(param[i] + "; ");
			}
		}
		GetSetGui::ReadOnlyText("Parametrization/Active Parameters") = active;
	}

	if (key == "Reset Param. Settings"){

		Geometry::ModelFDCTCalibrationCorrection HT(0,0,0,0,0);
		resetGuiOptimizerSettings(HT);
		GetSetGui::ReadOnlyText("Parametrization/Active Parameters") = "";
	}


////////// Trajectory /////////////////////////////////
	if (key == "CircularTrajectory")
	{
		if (g_dtrs.size() == 0)
		{
			error("No Radon Derivatives Loaded", "Please load Radon Derivatives first!");
		}
		else
		{
			circularTrajectory();
		}
		
	}


/////////// VISUALIZE //////////////////////////////////////////////////////////

	if (key == "Consistency Image"){ //creates consistency image
		if (testIfCorrectlyLoaded())
		{
			NRRD::Image<float> img((int)g_dtrs.size(), (int)g_dtrs.size(), 1);
			double cost = plot_constistency(img);

			int scale = 1;
			int n = img.size(0);
			while (img.size(0)*scale < 500) scale++;
			QtUtils::Figure("Consistency image", img, 0, 0.5 / cost, scale)
				.drawLine(0, 0, n, n, 1, 0, 0);
		}
	}

	if (key == "Costfunction"){
		if (testIfCorrectlyLoaded())
		{
			plot_cost();
		}
	}

	if (key == "Plot"){ // Costfunction 2D/
		if (testIfCorrectlyLoaded())
		{
			plot2D_cost();
		}
	}


/////// IO ///////////////////////////////////////////////////////
	//GetSetIO::save<GetSetIO::IniFile>("FDCTCalibrationCorrection.ini");
	g_app.saveSettings();
}


int main(int argc, char ** argv)
{
//	QApplication app(argc, argv);

//// Data //////////////////////////////
	GetSetGui::File("Data/Projection Matrices")
		.setExtensions("One Matrix Per Line (*.ompl);;All Files (*)");

	GetSetGui::File("Data/Radon Derivatives")
		.setExtensions("NRRD file (*.nrrd);;All Files (*)")
		.setMultiple(true);

	GetSetGui::Button("Data/Load Data") = "Load";
	GetSetGui::Button("Data/Load Projection Matrices") = "Load Matrices";

	GetSet<double>("Data/Value Kappa") = 0.002;
	GetSet<double>("Data/Pixel Spacing") = 0.308;

	GetSetGui::Button("Data/Save") = "Save data";
	GetSetGui::Directory("Data/Saving path");

	GetSet<bool>("Data/Load Half of the Data");

//// Optimize /////////////////////////////////
	GetSetGui::Button("Optimize/Optimization") = "Optimize";

	GetSetGui::Enum("Optimize/Method").setChoices("GN_DIRECT_L;LN_SBPLX;LN_BOBYQA;LD_LBFGS;LD_MMA");

	GetSet<double>("Optimize/Tolerance xtol_abs") = 0.001;
	GetSet<double>("Optimize/Tolerance ftol_abs") = 1e-5;

	GetSet<Eigen::Matrix3d>("Optimize/Initial Transform"); // = { 1, 0, 0; 0, 1, 0; 0, 0, 1 };

	GetSetGui::ReadOnlyText("Optimize/ECC Value");
	GetSetGui::ReadOnlyText("Optimize/Total Time");


//// VISUALIZE ////////////////////////////////////////
	GetSetGui::Button("Visualize/Consistency Image") = "Show";

	GetSet<int>("Visualize/Plot Samples").setDescription("Number of subdivisions of regular grid.") = 51;
	GetSetGui::StaticText("Visualize/Plot Samples_") = "Note: Be aware that \"Parameter Range or Plot Samples\" is used as";
	GetSetGui::StaticText("Visualize/Plot Samples__") = "step size for the forward diffence (gradient) in the optization.";

	GetSetGui::Button("Visualize/Costfunction") = "Plot active Parameters";
	GetSetGui::Button("Visualize/Costfunction 2D/Plot") = "Plot 2D";

	auto param = Geometry::ModelFDCTCalibrationCorrection::ParameterNames();
	GetSetGui::Enum("Visualize/Costfunction 2D/Parameter 1").setChoices(param);
	GetSetGui::Enum("Visualize/Costfunction 2D/Parameter 2").setChoices(param);

	GetSetGui::StaticText("Visualize/00 ") = "All plots are using the current estimates of the active parameters.";


//// Parametrization ///////////////////////////////////
	// auto param = Geometry::ModelSimilarity2D::ParameterNames();
	for (int i = 0; i < param.size(); i++)
	{
		GetSetGui::StaticText(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/123") = param[i];
		GetSet<bool>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Active");
		GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Upper Bound") = 10;
		GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Lower Bound") = -10;
		GetSet<double>(std::string("Parametrization/") + toString(i, 2) + " " + param[i] + "/Current Estimate") = 0;
	}

	GetSetGui::Button("Parametrization/Reset Param. Settings") = "Reset";
	GetSetGui::ReadOnlyText("Parametrization/Active Parameters");

//// Trajectory /////////////////////////////////
	
	GetSet<double>("Trajectory/Source Isocenter Distance [mm]") = 800;
	GetSet<double>("Trajectory/Source Detector Distance [mm]") = 1200;
	GetSetGui::ReadOnlyText("Trajectory/Detectorsize Horizontal [pix]");
	GetSetGui::ReadOnlyText("Trajectory/Detectorsize Vertical [pix]");

	GetSet<double>("Trajectory/Max Angle [deg]") = 200;
	GetSet<double>("Trajectory/Pixel Spacing [mm per pix]") = 0.308;

	GetSetGui::Button("Trajectory/CircularTrajectory") = "fit";
	GetSetGui::StaticText("Trajectory/CircularTrajectory_") = "calculating the circular trajectory,";
	GetSetGui::StaticText("Trajectory/CircularTrajectory__") = "saving the projection matrices and (re)load them.";


//// Ini-File Loading ///////////////////
//	GetSetIO::load<GetSetIO::IniFile>("FDCTCalibrationCorrection.ini");

//// GUI ///////////////////
//	GetSetSettingsWindow *settings = new GetSetSettingsWindow();
//	settings->setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint | Qt::WindowSystemMenuHint);
//	settings->show();

//	GetSetHandler callback(gui);
	g_app.init(argc, argv, gui);
	g_app.ignoreNotifications(false);

//// Simple scripting ///////////////////
	std::string script = GetSet<>("Script");
	if (!script.empty())
	{
		std::cout << script << std::endl;
		GetSet<>("Script") = "";
		std::vector<std::string> key_value_pair = stringToVector<std::string>(script, ';');
		for (auto it = key_value_pair.begin(); it != key_value_pair.end(); ++it)
		{
			// section=value
			std::vector<std::string> key_value = stringToVector<std::string>(*it, '#');
			if (key_value.size() == 2)
			{
				std::cout << key_value[0] << " set to " << key_value[1] << std::endl;
				GetSet<>(std::string(key_value[0])) = key_value[1];
			}
			if (key_value.size() == 1)
			{
				std::cout << "Triggering " <<  key_value[0] << std::endl;
				GetSetGui::Button(std::string(key_value[0])).trigger();
			}
		}
		exit(0);
	}

	g_app.window().addDefaultFileMenu();
	g_app.window().addMenuItem("Optimization", "Run", "Ctrl+D");

	//g_app.parseScript("file run FDCTCalibrationCorrection.getset");
	//g_app.parseScript("call function set_all_inactive");

	return g_app.exec();
}
