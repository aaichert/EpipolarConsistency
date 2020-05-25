// Created by A. Aichert on Tue July 7th 2018
// Raw data domain registration of CT scans

// Managing saving/loading parametzers and automatic GUI
#include <GetSetGui/GetSetGui.h>
#include <GetSetGui/GetSetTabWidget.h>
GetSetGui::Application g_app("RegistrationIntensityBased");

// Plotting and Visualization
#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>
using UtilsQt::Figure;
using UtilsQt::Plot;

#include <NRRD/nrrd_lowpass.hxx>

// Timing cost function evaluations when plotting
#include <Utils/TimerWin32.hxx>

// Parametrization of Transformation and Optimization
#include <LibProjectiveGeometry/Models/ModelSimilarity3D.hxx>
#include <LibOpterix/ParameterGui.hxx>
#include <LibOpterix/WrapNLOpt.hxx>

// Volume Rendering
#include <LibRayCastBackproject/VolumeRendering.h>
VolumeRendering::VoxelData *g_voxel_data_target=0x0;
VolumeRendering::VoxelData *g_voxel_data_source=0x0;
VolumeRendering::Raycaster *g_raycaster_target =0x0;
VolumeRendering::Raycaster *g_raycaster_source =0x0;

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>
#include <LibUtilsCuda/culaut/culaut.hxx>

extern void cuda_warp_image_3d1c(culaut::Array<float,16> T_inv, int n_x_in, int n_y_in, int n_z_in, cudaTextureObject_t input_image, int n_x_out, int n_y_out, int n_z_out, float* output_d);
extern void cuda_warp_image_3d1c_difference(culaut::Array<float,16> T_inv, int n_x_in, int n_y_in, int n_z_in, cudaTextureObject_t input_image, cudaTextureObject_t reference_image, int n_x_out, int n_y_out, int n_z_out, float* output_d);
extern float cuda_sum_reduction(const UtilsCuda::MemoryView<float>&);

bool cuda_warp(Geometry::RP3Homography T_mm, VolumeRendering::VoxelData& volume, NRRD::ImageView<float>& out, const Geometry::RP3Homography& M_out_vx_2_mm)
{
	Geometry::RP3Homography M_in_vx_2_mm=volume.getModelTransform();
	Geometry::RP3Homography T_vx_inv=M_in_vx_2_mm.inverse()*T_mm.inverse()*M_out_vx_2_mm;
	culaut::Array<float,16> T_vx_inv_f((float*)T_vx_inv.cast<float>().eval().data());
	int w=volume.size(0), h=volume.size(1), d=volume.size(2);
	if (w!=out.size(0)||h!=out.size(1)||d!=out.size(2)) return false;
	UtilsCuda::MemoryBlock<float> temporary(out.length());
	cuda_warp_image_3d1c(T_vx_inv_f,w,h,d,volume.getTexture(),w,h,d, temporary );
	temporary.readback(out);
	out.meta_info["Model Transform [vx]->[mm]"]=toString(M_out_vx_2_mm);
	out.meta_info["Transformation [mm]"] =toString(T_mm);
	return true;
}

// Parameters
LibOpterix::ParameterGUI  *parameter_gui=0x0;

GraphicsItems::Group& make_bounding_box(const VolumeRendering::VoxelData& volume, GraphicsItems::Group& group, QColor color)
{
	group.add(GraphicsItems::ConvexMesh::WireCube(Eigen::Vector3d(0,0,0), Eigen::Vector3d(volume.size(0),volume.size(1),volume.size(2)), color));
	group.setTransform(volume.getModelTransform());
	return group;
}


Geometry::ProjectionMatrix lazy_P;
bool update(NRRD::Image<float>& image_target, GraphicsItems::Group& overlay, UtilsQt::ProjectionParametersGui& projection, double magnification)
{
	// Be lazy.
	if (lazy_P==projection.getProjectionMatrix()) return false;
	lazy_P=projection.getProjectionMatrix();

	// Get Transformations from GUI
	Geometry::RP3Homography model_target=GetSet<Geometry::RP3Homography>("Input/Model Matrix/Target");
	Geometry::RP3Homography model_source=GetSet<Geometry::RP3Homography>("Input/Model Matrix/Source");
	Geometry::RP3Homography registration=GetSet<Geometry::RP3Homography>("Input/Registration/Current Estimate");
	// Set transformation matrices
	g_voxel_data_target->setModelTransform(model_target);
	g_voxel_data_source->setModelTransform(registration*model_source);
	// Ground truth
	Geometry::RP3Homography registration_gt=GetSet<Geometry::RP3Homography>("Input/Registration/Ground Truth");
	Geometry::RP3Homography model_source_gt=registration_gt*model_source;
	// Prepare render target
	int w=(int)projection.image_size[0];
	int h=(int)projection.image_size[1];
	if (image_target.size(1)!=4
	||  image_target.size(1)!=w
	||  image_target.size(2)!=h)
		image_target.set(4,w,h);
	if (w<16 || h<16) return false; // no change to image
	// Direct Volume Rendering
	if (g_raycaster_target)
		g_raycaster_target->render(image_target,lazy_P);
	if (g_raycaster_source)
	{
		NRRD::Image<float> image_overlay(4,w,h);
		g_raycaster_source->render(image_overlay,lazy_P);
		for (int v=0;v<h;v++)
			for (int u=0;u<w;u++)
			{
				float *pixel_target =&(image_target .pixel(0,u,v));
				float *pixel_overlay=&(image_overlay.pixel(0,u,v));
				float alpha=pixel_overlay[3];
				if (false)
				{
					// Alpha blending
					pixel_target[0]=(1.0f-alpha)*pixel_target[0]+alpha*pixel_overlay[0];
					pixel_target[1]=(1.0f-alpha)*pixel_target[1]+alpha*pixel_overlay[1];
					pixel_target[2]=(1.0f-alpha)*pixel_target[2]+alpha*pixel_overlay[2];
					pixel_target[3]=(1.0f-alpha)*pixel_target[2]+alpha*pixel_overlay[2];
				}
				else
				{
					// Channel mask
					pixel_target[0]=pixel_overlay[0];
					pixel_target[1]=pixel_target [1]; // target is green!
					pixel_target[2]=pixel_overlay[2];
					pixel_target[3]=pixel_target [3];
				}
			}
	}
	// Make overlay
	overlay.clear();
	QColor red(255,0,0,128), green(0,255,0,128), blue(0,0,255,128);
	make_bounding_box(*g_voxel_data_source,overlay.group("source")      ,red   ).add(GraphicsItems::CoordinateAxes(100*model_source.block<3,1>(0,0).norm(),"Source","","",""));
	make_bounding_box(*g_voxel_data_target,overlay.group("target")      ,green ).add(GraphicsItems::CoordinateAxes(100*model_target.block<3,1>(0,0).norm(),"Target","","",""));
	make_bounding_box(*g_voxel_data_source,overlay.group("ground truth"),blue  ).setTransform(model_source_gt);
	overlay.set("CoSy",GraphicsItems::CoordinateAxes(100,"World"));
	return true; // Let window know that image has changed.
}

/// Optimization problem for intensity-based registration.
class IntensityBased3D3DReg : public LibOpterix::OptimizationProblem<Geometry::RP3Homography> {
		VolumeRendering::VoxelData& source;
		VolumeRendering::VoxelData& target;
		UtilsCuda::MemoryBlock<float> temporary;
		mutable Geometry::RP3Homography tmp_last_eval;
public:

	IntensityBased3D3DReg(
			LibOpterix::ParameterModel<Geometry::RP3Homography>& _parameter_model,
			VolumeRendering::VoxelData& _source,
			VolumeRendering::VoxelData& _target)
		: LibOpterix::OptimizationProblem<Geometry::RP3Homography>(_parameter_model)
		, source(_source)
		, target(_target)
		, temporary(_target.size(0)*_target.size(1)*_target.size(2))
		{}

	Geometry::RP3Homography lastEval() const { return tmp_last_eval; }

	/// Compute cost function value from ParametrizedObject
	virtual double evaluate(const Geometry::RP3Homography& T_mm)
	{
		tmp_last_eval=T_mm;
		Geometry::RP3Homography M_t=GetSet<Geometry::RP3Homography>("Input/Model Matrix/Target");
		Geometry::RP3Homography M_s=GetSet<Geometry::RP3Homography>("Input/Model Matrix/Source");
		auto T_vx=(M_t.inverse()*T_mm*M_s).eval();
		culaut::Array<float,16> T_vx_inv((float*)(T_vx.inverse().cast<float>()).eval().data());
		int w=target.size(0), h=target.size(1), d=target.size(2);
		cuda_warp_image_3d1c_difference(T_vx_inv,w,h,d,source.getTexture(),target.getTexture(),w,h,d, temporary );
		return cuda_sum_reduction(temporary);
	}
};

bool show_current=false;
void debug_hook(double current_cost, LibOpterix::AbstractOptimizationProblem* problem)
{
	IntensityBased3D3DReg* reg=dynamic_cast<IntensityBased3D3DReg*>(problem);
	if (reg && show_current) GetSet<Geometry::RP3Homography>("Input/Registration/Current Estimate")=reg->lastEval();
	std::cout << current_cost << std::endl;
}

void gui(const GetSetInternal::Node& node)
{

	if (hasPrefix(node.super_section,"Intensity Based Registration/Parameters"))
	{
		Geometry::ModelSimilarity3D model;
		model.setValues(parameter_gui->getValues().data());
		GetSet<Geometry::RP3Homography>("Input/Registration/Current Estimate")=model.getInstance();
	}

	if (node.name=="Current Estimate")
	{
		lazy_P.setZero();
		if (g_voxel_data_target && g_voxel_data_source)
			Figure("Ray Casting").update();
	}

	if (node.name=="Load")
	{
		// Discard old data (if any)
		if (g_voxel_data_target) delete g_voxel_data_target;
		if (g_voxel_data_source) delete g_voxel_data_source;
		g_voxel_data_target=g_voxel_data_source=0x0;
		if (g_raycaster_target) delete g_raycaster_target;
		if (g_raycaster_source) delete g_raycaster_source;
		g_raycaster_target=g_raycaster_source=0x0;

		// Gaussian filter
		int        k=GetSet<int>   ("Input/Gaussian/Kernel Width");
		double sigma=GetSet<double>("Input/Gaussian/Sigma");

		// Load image and assert success
		g_app.progressStart("Loading Input Data...", "Reading large files from disk. This may take a while.",5);
		g_app.progressUpdate(0);
		NRRD::Image<float> volume_target(GetSet<>("Input/Files/Target"));
		int l_t=volume_target.length();
		for (int i =0;i<l_t;i++)
			if (volume_target[i]<0.005)
				volume_target[i]=0;
		if (k>0) 
			NRRD::lowpass3D(volume_target,sigma,k);
		g_app.progressUpdate(1);
		NRRD::Image<float> volume_source(GetSet<>("Input/Files/Source"));
		int l_s=volume_source.length();
		for (int i =0;i<l_s;i++)
			if (volume_source[i]<0.005)
				volume_source[i]=0;
		if (k>0)
			NRRD::lowpass3D(volume_source,sigma,k);
		g_app.progressUpdate(2);
		if (!volume_target || !volume_source) g_app.progressEnd();
		if (!volume_target) g_app.warn("Failed to Load Data", GetSet<>("Input/Files/Target"));
		if (!volume_source) g_app.warn("Failed to Load Data", GetSet<>("Input/Files/Source"));
		if (!volume_target || !volume_source) return;

		// Download to GPU and bild ESS geometry (optional)
		bool use_ess=GetSet<bool>("Input/Empty Space Skipping");
		g_voxel_data_target=new VolumeRendering::VoxelData(volume_target, use_ess);
		g_app.progressUpdate(3);
		g_voxel_data_source=new VolumeRendering::VoxelData(volume_source, use_ess);
		g_raycaster_target =new VolumeRendering::Raycaster (*g_voxel_data_target);
		g_raycaster_source =new VolumeRendering::Raycaster (*g_voxel_data_source);
		double value_min=g_raycaster_target->getVolume().getValueMin();
		g_raycaster_target->raycastPass<VolumeRendering::DigitallyReconstructedRadiograph>();
		double value_max=g_raycaster_target->getVolume().getValueMax();
//		g_raycaster_target->raycastPass<VolumeRendering::DigitallyReconstructedRadiograph>().setTransferFunction(value_min,value_max);
//		g_raycaster_source->raycastPass<VolumeRendering::DigitallyReconstructedRadiograph>().setTransferFunction(value_min,value_max);
		g_raycaster_target->setSamplesPerVoxel(1.5);
		g_raycaster_source->setSamplesPerVoxel(1.5);
		g_app.progressUpdate(4);
		
		// Load Model Matrices
		if (volume_target.meta_info.find("Model Matrix")!=volume_target.meta_info.end())
			 GetSet<>("Input/Model Matrix/Target")=volume_target.meta_info["Model Transform"];
		else GetSet<Geometry::RP3Homography>("Input/Model Matrix/Target")=g_voxel_data_target->getModelTransform();
		if (volume_source.meta_info.find("Source Matrix")!=volume_source.meta_info.end())
			 GetSet<>("Input/Model Matrix/Source")=volume_source.meta_info["Model Transform"];
		else GetSet<Geometry::RP3Homography>("Input/Model Matrix/Source")=g_voxel_data_source->getModelTransform();
		g_app.progressEnd();

		Figure("Ray Casting",800,600).setCallback(update);
	}

	if (node.name=="Run Registration")
	{
		Geometry::ModelSimilarity3D model(parameter_gui->getActiveSet());
		IntensityBased3D3DReg reg(model,*g_voxel_data_source,*g_voxel_data_target);
		g_voxel_data_target->setModelTransform(GetSet<Geometry::RP3Homography>("Input/Model Matrix/Target"));
		g_voxel_data_source->setModelTransform(GetSet<Geometry::RP3Homography>("Input/Model Matrix/Source"));
		GetSetGui::Section optimization("Intensity Based Registration/Optimization");
		UtilsNLOpt::OptimizerSettings opt_settings(UtilsNLOpt::algorithm_names("LN"));
		opt_settings.gui_retreive_section(optimization);
		// Make bounds relative to current value and set bound constraints.
		auto x =parameter_gui->getActiveValues();
		auto lb=parameter_gui->getActiveLowerBounds();
		auto ub=parameter_gui->getActiveUpperBounds();
		for (int i=0;i<(int)x.size();i++) {
			lb[i]+=x[i];
			ub[i]+=x[i];
		}
		UtilsNLOpt::Optimizer opt(reg,opt_settings,lb,ub);
		// Set debug hook for intermediiate information.
		g_raycaster_target->setSamplesPerVoxel(0.25);
		g_raycaster_source->setSamplesPerVoxel(0.25);
		show_current= GetSet<bool> ("Intensity Based Registration/Show Intermediate Steps");
		opt.setCallback(debug_hook);
		// Run optimization and show results in GUI
		std::cout << "before=[" << vectorToString(x) << "]\n";
		opt.optimize(x,g_app);
		std::cout << "after =[" << vectorToString(x) << "]\n";
		g_raycaster_target->setSamplesPerVoxel(1.5);
		g_raycaster_source->setSamplesPerVoxel(1.5);
		parameter_gui->setActiveValues(x);
	}

	if (node.name=="Show Warped Voxel Data")
	{
		if (!g_voxel_data_source)
		{
			g_app.warn("Failed to warp image", "Please load data first.");
			return;
		}
		NRRD::Image<float> warped(g_voxel_data_target->size(0),g_voxel_data_target->size(1),g_voxel_data_target->size(2));
		using Geometry::RP3Homography;
		Geometry::RP3Homography M_t=GetSet<Geometry::RP3Homography>("Input/Model Matrix/Target");
		Geometry::RP3Homography M_s=GetSet<Geometry::RP3Homography>("Input/Model Matrix/Source");
		Geometry::RP3Homography T  =GetSet<Geometry::RP3Homography>("Input/Registration/Current Estimate");
		g_voxel_data_target->setModelTransform(M_t);
		g_voxel_data_source->setModelTransform(M_s);
		cuda_warp(T,*g_voxel_data_source,warped, g_voxel_data_target->getModelTransform());
		Figure("Warped Source Image", warped);
		int l=warped.length();
		NRRD::Image<float> target(g_voxel_data_target->size(0),g_voxel_data_target->size(1),g_voxel_data_target->size(2));
		cuda_warp(Geometry::RP3Homography::Identity(),*g_voxel_data_target,target, g_voxel_data_target->getModelTransform());
		for (int i=0;i<l;i++)
			warped[i]-=target[i];
		Figure("Difference Image", warped,0,0,true);
	}

	// Save settings to ini-file
	g_app.saveSettings();
}

int main(int argc, char ** argv)
{
	// Input
	GetSetGui::File     ("Input/Files/Target").setExtensions("3D NRRD file (*.nrrd);;All Files (*)");
	GetSetGui::File     ("Input/Files/Source").setExtensions("3D NRRD file (*.nrrd);;All Files (*)");
	GetSetGui::Button   ("Input/Files/Load"  )="(Re-)Load";
	GetSet<Geometry::RP3Homography>("Input/Model Matrix/Target"          ).setValue(Geometry::RP3Homography::Identity()).setDescription("Target model transform [vx] -> [mm].");
	GetSet<Geometry::RP3Homography>("Input/Model Matrix/Source"          ).setValue(Geometry::RP3Homography::Identity()).setDescription("Source model transform [vx] -> [mm].");
	GetSet<Geometry::RP3Homography>("Input/Registration/Current Estimate").setValue(Geometry::RP3Homography::Identity()).setDescription("Current estimate source [mm] -> target [mm].");
	GetSet<Geometry::RP3Homography>("Input/Registration/Ground Truth"    ).setValue(Geometry::RP3Homography::Identity()).setDescription("Reference transformation just for visualization.");
	GetSetGui::Section   ("Input/Files").setGrouped();
	GetSetGui::Section   ("Input/Model Matrix").setGrouped();
	GetSetGui::Section   ("Input/Registration").setGrouped();
	GetSet<bool>         ("Input/Empty Space Skipping")=true;
	GetSet<int>          ("Input/Gaussian/Kernel Width")=0;
	GetSet<double>       ("Input/Gaussian/Sigma")=1.86;
	GetSetGui::StaticText("Input/Gaussian/_")="This is just a hack. Please avoid uage (SLOW!).";

	GetSetGui::Section ibr("Intensity Based Registration");
	GetSet<bool>     ("Show Intermediate Steps",ibr)=false;
	GetSetGui::Button("Actions/Run Registration",ibr)="Run...";
	GetSetGui::Button("Actions/Show Warped Voxel Data",ibr)="Show...";
	ibr.subsection("Actions").setGrouped();
	UtilsNLOpt::OptimizerSettings(UtilsNLOpt::algorithm_names("LN")).gui_declare_section( ibr.subsection("Optimization"));
	parameter_gui=new LibOpterix::ParameterGUI(Geometry::ModelSimilarity3D::ParameterNames(), ibr.subsection("Parameters"),false);
	parameter_gui->declareActive();
	parameter_gui->declareBounds();
	parameter_gui->declareValues();
	
	// Menu and about text
	g_app.init(argc,argv,gui);
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

	return g_app.exec();
}
