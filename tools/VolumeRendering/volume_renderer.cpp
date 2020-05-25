// For exporting functions and responsive GUI
#include <QApplication>
#include <QFileDialog>

// Managing saving/loading parameters and automatic GUI
#include <GetSetGui/GetSetGui.h>
#include <GetSetGui/GetSetTabWidget.h>

GetSetGui::Application g_app("VolumeRendering");

// Visualization
#include <LibUtilsQt/Figure.hxx>
using UtilsQt::Figure;
#include <Utils/TimerWin32.hxx>

// Geometry
#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibProjectiveGeometry/EigenToStr.hxx>
#include <Utils/Projtable.hxx>

// Volume Rendering
#include <LibRayCastBackproject/VolumeRendering.h>
VolumeRendering::VoxelData* g_voxel_data=0x0;
VolumeRendering::Raycaster*	g_raycaster=0x0;
bool						g_busy=0; // inidcates if several DRRs are being processed

// Trajectory to display
std::vector<Geometry::ProjectionMatrix>	g_Ps;

void update_gui_ranges(double min, double max, double step)
{
	GetSetGui::RangedDouble("Volume Rendering/Iso Value").setMin(min).setMax(max).setStep(step);
	GetSetGui::RangedDouble("Volume Rendering/Transfer Function Ramp/Min").setMin(min).setMax(max).setStep(step)=min;
	GetSetGui::RangedDouble("Volume Rendering/Transfer Function Ramp/Max").setMin(min).setMax(max).setStep(step)=max;
}

// Prepare window for rendering
bool draw3DVisualization(NRRD::Image<float>&, GraphicsItems::Group&, UtilsQt::ProjectionParametersGui&, double);
void assertWindowVisible()
{
	if (!Figure("Volume Rendering").exists(true))
	{
		Figure figure("Volume Rendering",800,600);
		figure.overlay().add(GraphicsItems::CoordinateAxes());
		figure.setCallback(draw3DVisualization);
	}
}

void loadInputVolume()
{
	Utils::TimerWin32 time;
	g_app.progressStart("Initializing...","Loading Volume Data...",-1);
	std::string volume_file=GetSet<>("Input/Input Volume");
	NRRD::Image<float> volume(volume_file);
	g_app.progressEnd();
	if (!volume || volume.dimension()!=3)
	{
		g_app.warn("Input Error", std::string("Failed to load input file")+volume_file);
		return;
	}
	g_app.progressStart("Initializing...","Converting and optimizing volume data...",-1);

	std::cout << "Conversion Time: " << time.getElapsedTime() << std::endl;
	std::cout << "Dimensions:      " << volume.size(0) << "x" <<  volume.size(1) << "x" << volume.size(2) << " voxels\n";
	std::cout << "Physical Size:   " <<  volume.size(0)*volume.spacing(0) << "x" <<  volume.size(1)*volume.spacing(1) << "x" << volume.size(2)*volume.spacing(2) << " mm\n";
	int l=volume.length();

	// Centered bounding box in world units
	if (g_voxel_data) delete g_voxel_data;
	if (g_raycaster) delete g_raycaster;
	g_voxel_data=new VolumeRendering::VoxelData(volume,0);
	g_raycaster=new VolumeRendering::Raycaster(*g_voxel_data);
	double spv=GetSet<double>("Volume Rendering/Samples Per Voxel");
	g_raycaster->setSamplesPerVoxel(spv);

	g_raycaster->getVolume().centerVolume();
	auto tfn=g_raycaster->getVolume().getModelTransform();
	GetSet<Geometry::RP3Homography>("Volume Rendering/Model Transform")=tfn;

	float max_val=g_raycaster->getVolume().getValueMax();
	float min_val=g_raycaster->getVolume().getValueMin();
	update_gui_ranges(min_val,max_val,(max_val-min_val>10)?(max_val-min_val)/5000:0.05);
	g_app.progressEnd();
	assertWindowVisible();
}

bool draw3DVisualization(NRRD::Image<float>& image, GraphicsItems::Group& overlay, UtilsQt::ProjectionParametersGui&, double)
{
	g_app.ignoreNotifications(true);
	// Get current image size
	Figure figure("Volume Rendering");
	auto image_size=figure.getSize();
	Geometry::ProjectionMatrix P=figure.getProjectionMatrix();
	figure.forget();
	image.set(image_size[0],image_size[1]);
	image.nrrd_header["Projection Matrix"]=toString(P);

	int selected_cam=GetSet<int>("Input/Trajectory Selection");

	if (g_raycaster)
	{
		if (GetSet<bool>("Projection/Clipping/Apply Clipping"))
		{
			std::vector<Geometry::RP3Plane> clip_planes(1);
			clip_planes[0].head(3)=GetSet<Eigen::Vector3d>("Projection/Clipping/Clip Plane Orientation").getValue().normalized();
			GetSet<Eigen::Vector3d>("Projection/Clipping/Clip Plane Orientation")=clip_planes[0].head(3);
			clip_planes[0][3]=GetSet<int>("Projection/Clipping/Clip Plane Distace");
			g_raycaster->setClipPlanes(clip_planes);
		}
		else
			g_raycaster->setClipPlanes(std::vector<Geometry::RP3Plane>());

		Utils::TimerWin32 timer;
		g_raycaster->render(image,P);
		GetSetGui::StaticText("Volume Rendering/_info")=toString((int)(1000*timer.getTotalTime()))+" milliseconds";
	}

	int i=GetSet<int>("Input/Trajectory Selection");
	auto detector_size=GetSet<Eigen::Vector2i>("Input/Detector Size").getValue();
	double detector_spacing=GetSet<double>("Input/Detector Pixel Spacing");
	Eigen::Vector4d image_rect (0,0,detector_size[0],detector_size[1]);
	if (i>=0 && i<g_Ps.size())
		overlay.set("Selected Camera",GraphicsItems::FancyCamera(g_Ps[i],image_rect,detector_spacing));

	g_app.ignoreNotifications(false);

	return true;
}

void gui(const GetSetInternal::Node& node)
{
	assertWindowVisible();

	const std::string& section=node.super_section;
	const std::string& key    =node.name;

	if (key=="Trajectory Selection" )
	{
		int selected_cam=GetSet<int>("Input/Trajectory Selection");
		if (selected_cam<0 || selected_cam >= (int)g_Ps.size())
			return;

		if (g_voxel_data && !g_busy)
		{
			auto   detector_size=GetSet<Eigen::Vector2i>("Input/Detector Size").getValue();
			double binning=1;
			while (detector_size[0]+detector_size[1]>2048*binning)
				binning++;
			detector_size/=binning;
			double detector_spacing=GetSet<double>("Input/Detector Pixel Spacing")*binning;

			Eigen::Matrix3d H=Eigen::Matrix3d::Identity();
			H(1,1)=H(0,0)=1.0/binning;
			Geometry::ProjectionMatrix HP=H*g_Ps[selected_cam];
			std::cout << "H=\n"<<H << std::endl;
			std::cout << "P=\n"<<g_Ps[selected_cam] << std::endl;
			std::cout << "HP=\n"<< HP << std::endl;

			NRRD::Image<float> binned_projection(detector_size[0],detector_size[1]);
			Utils::TimerWin32 timer;
			VolumeRendering::Raycaster vr_preview(*g_voxel_data);
			vr_preview
				.setSamplesPerVoxel(1)
				.raycastPass<VolumeRendering::DigitallyReconstructedRadiograph>();
			vr_preview.render(binned_projection,HP);
			
			UtilsQt::Figure("Preview",binned_projection)
				.setProjectionMatrix(HP)
				.drawText( std::string("Projection ")+toString(selected_cam)+"; "+toString((int)(1000*timer.getTotalTime()))+" milliseconds")
				.overlay().add(GraphicsItems::CoordinateAxes());
		}
	}

	if (key=="Open...")
	{
		std::string input_file=QFileDialog::getOpenFileName(0x0, "Open 3D NRRD Vollume", "", "3D NRRD Volume (*.nrrd);;All Files (*)").toStdString();
		if (!input_file.empty())
			GetSet<>("Input/Input Volume")=input_file;
	}

	if (key=="Input Volume")
		loadInputVolume();

	if (key=="Export 3D Sinogram...")
	{
		if (!g_raycaster) return;
		// Select file to save image stack to
		std::string output_file=QFileDialog::getSaveFileName(0x0, "Export as NRRD stack", "projections.nrrd", "Stack of projections as 3D NRRD image (*.nrrd);;All Files (*)").toStdString();
		if (output_file.empty()) return;

		// Set up memory for projection images
		int num_projections=(int)g_Ps.size();
		auto detector_size=GetSet<Eigen::Vector2i>("Input/Detector Size").getValue();
		double spv=GetSet<double>("Volume Rendering/Samples Per Voxel");
		double spacing=GetSet<double>("Input/Detector Pixel Spacing");
		NRRD::Image<float> projection_stack(detector_size[0],detector_size[1],num_projections);
		projection_stack.setElementKind(2,"index");
		projection_stack.spacing(0)=projection_stack.spacing(1)=spacing;
		projection_stack.meta_info["Detector Pixel Spacing"]=toString(spacing);
		projection_stack.meta_info["Samples Per Voxel"]=toString(spv);
		projection_stack.meta_info["Volume Model Transform"]=toString(g_voxel_data->getModelTransform());
		projection_stack.meta_info["Volume Voxel Size"]=
			 toString(g_voxel_data->size(0))+" "
			+toString(g_voxel_data->size(1))+" "
			+toString(g_voxel_data->size(2));
		projection_stack.meta_info["Projection Matrices"]=vectorToString<Geometry::ProjectionMatrix>(g_Ps," / ");
		projection_stack.meta_info["Transfer Function Ramp/Min"  ]=GetSet<double>("Volume Rendering/Transfer Function Ramp/Min"  ).getString();
		projection_stack.meta_info["Transfer Function Ramp/Max"  ]=GetSet<double>("Volume Rendering/Transfer Function Ramp/Max"  ).getString();
		projection_stack.meta_info["Transfer Function Ramp/Scale"]=GetSet<double>("Volume Rendering/Transfer Function Ramp/Scale").getString();

		// Set up raycaster
		g_app.progressStart("Export 3D Sinogram...","Setting up raycaster...");
		VolumeRendering::Raycaster drr(*g_voxel_data);
		drr.setSamplesPerVoxel(spv);
		drr.raycastPass<VolumeRendering::DigitallyReconstructedRadiograph>();
		if (GetSet<bool>("Projection/Clipping/Apply Clipping"))
		{
			std::vector<Geometry::RP3Plane> clip_planes(1);
			clip_planes[0].head(3)=GetSet<Eigen::Vector3d>("Projection/Clipping/Clip Plane Orientation").getValue().normalized();
			GetSet<Eigen::Vector3d>("Projection/Clipping/Clip Plane Orientation")=clip_planes[0].head(3);
			clip_planes[0][3]=GetSet<int>("Projection/Clipping/Clip Plane Distace");
			drr.setClipPlanes(clip_planes);
			projection_stack.nrrd_header["Clip Plane"]=toString(clip_planes[0]);
		}

		// Compute projections
		bool cancelled=false;
		g_app.progressStart("Export 3D Sinogram...","Computing projections...",num_projections,&cancelled);
		Utils::TimerWin32 time;
		for (int i=0;i<num_projections;i++)
		{
			if (cancelled)
				break;
			// Render and print timing info
			NRRD::ImageView<float> slice(projection_stack,i);
			drr.render(slice,g_Ps[i]);
			double pct_done=(double)(i+1)/num_projections;
			g_app.ignoreNotifications(true);
			GetSet<int>("Input/Trajectory Selection")=i;
			Figure("Volume Rendering").update();
			g_app.progressUpdate(i+1);
			// Copy to output stack
			UtilsQt::Figure("Preview",slice).drawText(
				std::string("Projection ")+toString(i)+"; "+
				toString((int)(time.getElapsedTime()*1000))+"ms"+
				"(est. time remaining: "+toString<int>(time.getTotalTime()/pct_done*(1.0-pct_done))+"sec)");
		}
		if (!cancelled)
		{
			// Save result
			g_app.progressStart("Export 3D Sinogram...","Saving Result...");
			projection_stack.save(output_file);
		}
		g_app.progressEnd();
	}

	if (key=="Model Transform")
	{
		if (!g_raycaster) return;
		GetSet<Geometry::RP3Homography> mt("Volume Rendering/Model Transform");
		if (mt.getValue()==Geometry::RP3Homography::Zero() || mt.getValue()==Geometry::RP3Homography::Identity())
		{
			g_voxel_data->centerVolume();
			mt=g_voxel_data->getModelTransform();
		}
		auto model_transform=mt.getValue();
		g_voxel_data->setModelTransform(model_transform);

		// Make Bounding Box
		auto& voxel3d=Figure("Volume Rendering").overlay().group("Voxel3D");
		double x=g_voxel_data->size(0);
		double y=g_voxel_data->size(1);
		double z=g_voxel_data->size(2);
		auto& cube=voxel3d.set("Bounding Box",GraphicsItems::ConvexMesh::Cube(Eigen::Vector3d(0,0,0), Eigen::Vector3d(x,y,z)));
		cube.q_color.clear();
		cube.l_color=QColor(0,0,255);
		cube.q_color.push_back(QColor(0,0,255,0));
		voxel3d.set("CoSy",GraphicsItems::CoordinateAxes(100,"[vx]"));
		voxel3d.setTransform(mt);

	}

	if (key=="Iso Value")
	{
		if (!g_raycaster) return;
		g_raycaster->raycastPass<VolumeRendering::IsoSurface>().setIsoValue(GetSet<double>("Volume Rendering/Iso Value"));
	}

	if (section=="Volume Rendering/Transfer Function Ramp")
	{
		if (!g_raycaster) return;
		// FIXME emission absorption
	/*	g_raycaster->raycastPass<VolumeRendering::>()
			.setTransferFunction(GetSet<double>("Volume Rendering/Transfer Function Ramp/Min"),
								 GetSet<double>("Volume Rendering/Transfer Function Ramp/Max"),
								 GetSet<double>("Volume Rendering/Transfer Function Ramp/Scale"));*/
	}
	
	if (key=="Samples Per Voxel")
	{
		if (!g_raycaster) return;
		double spv=GetSet<double>("Volume Rendering/Samples Per Voxel");
		g_raycaster->setSamplesPerVoxel(spv);
	}

	if (key=="Use Empty Space Skipping")
	{
		if (!g_raycaster) return;
//		g_raycaster->useEmptySpaceSkipping(GetSet<bool>("Volume Rendering/Use Empty Space Skipping"));
	}

	if (key=="Use MIP")
	{
		if (!g_raycaster) return;
		g_raycaster->raycastPass<VolumeRendering::MaximumIntensityProjection>();
	}

	if (key=="Debug Raycast Environment")
	{
		if (!g_raycaster) return;
		g_raycaster->raycastPass<VolumeRendering::Debug>();
	}

	if (key=="Digitally Reconstructed Radiograph (DRR)")
	{
		if (!g_raycaster) return;
		g_raycaster->raycastPass<VolumeRendering::DigitallyReconstructedRadiograph>();
	}

	if (key=="Projection Matrices")
		g_Ps=ProjTable::loadProjectionsOneMatrixPerLine(GetSet<>("Input/Projection Matrices"));

	if (hasPrefix(key,"Flip Image "))
	{
		auto detector_size=GetSet<Eigen::Vector2i>("Input/Detector Size").getValue();
		Eigen::Matrix3d H=Eigen::Matrix3d::Identity();
		if (key=="Flip Image u-Axes")
		{
			H(0,0)*=-1;
			H(0,2)=detector_size[0];
		}
		else
		{
			H(1,1)*=-1;
			H(1,2)=detector_size[1];
		}
		std::cout << "H=\n" << H << std::endl;
		for (int i=0;i<(int)g_Ps.size();i++)
			g_Ps[i]=H*g_Ps[i];
	}

	if (key=="Flip Orientation (pixel spacing)")
		GetSet<double>("Input/Detector Pixel Spacing")=-GetSet<double>("Input/Detector Pixel Spacing").getValue();

	if (hasPrefix(key,"Flip ") || hasPrefix(key,"Rotate 90 degrees"))
	{
		GetSet<Geometry::RP3Homography>  mt("Volume Rendering/Model Transform");
		     if (key=="Flip X-axis")     mt=Geometry::Scale(-1,1,1)*mt.getValue();
		else if (key=="Flip Y-axis")     mt=Geometry::Scale(1,-1,1)*mt.getValue();
		else if (key=="Flip Z-axis")     mt=Geometry::Scale(1,1,-1)*mt.getValue();
		else if (key=="Rotate 90 degrees about X-axis")     mt=Geometry::RotationX(0.5*Geometry::Pi)*mt.getValue();
		else if (key=="Rotate 90 degrees about Y-axis")     mt=Geometry::RotationY(0.5*Geometry::Pi)*mt.getValue();
		else if (key=="Rotate 90 degrees about Z-axis")     mt=Geometry::RotationZ(0.5*Geometry::Pi)*mt.getValue();
		else if (key=="Reset Transform") {
			g_voxel_data->centerVolume();
			mt=g_voxel_data->getModelTransform();
		}
	}

	// Save settings to file
	g_app.saveSettings();
}

int main(int argc, char ** argv)
{

	// Visualization
	GetSetGui::File           ("Input/Projection Matrices").setMultiple(true).setExtensions("All Files (*);;Siemens Projtable (*.txt);;Siemens Projtable 1.3 (*.xml);;One Projection Matrix Per Line (*.ompl)");
	GetSet    <Eigen::Vector2i>("Input/Detector Size").setString("1240 960");
	GetSet    <double>        ("Input/Detector Pixel Spacing")=.308;
	GetSet    <int>           ("Input/Trajectory Selection")  =0;
	GetSetGui::File           ("Input/Input Volume").setExtensions("3D NRRD file (*.nrrd);;All Files (*)");

	// Volume Rendering
	GetSetGui::StaticText     ("Volume Rendering/_info");
	GetSet<bool>              ("Volume Rendering/Use Empty Space Skipping")        =true;
	GetSetGui::Button         ("Volume Rendering/Use MIP")                         ="Maximum Intensity Projection";
	GetSetGui::RangedDouble   ("Volume Rendering/Iso Value")                       =1;
	GetSetGui::RangedDouble   ("Volume Rendering/Transfer Function Ramp/Min")      =0;
	GetSetGui::RangedDouble   ("Volume Rendering/Transfer Function Ramp/Max")      =1;
	GetSet<double>            ("Volume Rendering/Transfer Function Ramp/Scale")    =1;
	GetSetGui::Section        ("Volume Rendering/Transfer Function Ramp").setGrouped();
	GetSetGui::RangedDouble   ("Volume Rendering/Samples Per Voxel").setMin(0.1).setMax(2.0).setStep(0.25);

	// Clipping
	GetSet<bool>              ("Projection/Clipping/Apply Clipping")       =false;
	GetSet<int>               ("Projection/Clipping/Clip Plane Distace")   =128;
	GetSet<Eigen::Vector3d>   ("Projection/Clipping/Clip Plane Orientation").setString("0 0 -1");
	GetSetGui::Section        ("Projection/Clipping").setGrouped();

	// Menu bar
	g_app.commandLine().index("Input/Input Volume",1,false);
	g_app.init(argc,argv,gui);
	g_app.window().addMenuItem("File","Open...","Ctrl+O");
	g_app.window().addMenuItem("File");
	g_app.window().addMenuItem("File","Export 3D Sinogram...","Ctrl+Shift+S");
	g_app.window().addMenuItem("File");
	g_app.window().addDefaultFileMenu();
	g_app.window().addMenuItem("Utility/Transform/Image","Flip Image u-Axes");
	g_app.window().addMenuItem("Utility/Transform/Image","Flip Image v-Axes");
//	g_app.window().addMenuItem("Utility/Transform/Image");
//	g_app.window().addMenuItem("Utility/Transform/Image", "Rotate Image by 90 degrees");
	g_app.window().addMenuItem("Utility/Transform/Volume","Flip X-axis");
	g_app.window().addMenuItem("Utility/Transform/Volume","Flip Y-axis");
	g_app.window().addMenuItem("Utility/Transform/Volume","Flip Z-axis");
	g_app.window().addMenuItem("Utility/Transform/Volume");
	g_app.window().addMenuItem("Utility/Transform/Volume","Rotate 90 degrees about X-axis");
	g_app.window().addMenuItem("Utility/Transform/Volume","Rotate 90 degrees about Y-axis");
	g_app.window().addMenuItem("Utility/Transform/Volume","Rotate 90 degrees about Z-axis");
	g_app.window().addMenuItem("Utility/Transform/Volume");
	g_app.window().addMenuItem("Utility/Transform/Volume","Reset Transform");
	g_app.window().addMenuItem("Utility/Transform","Flip Orientation (pixel spacing)");
	g_app.window().addMenuItem("Utility","Debug Raycast Environment");
	g_app.window().addMenuItem("Utility","Digitally Reconstructed Radiograph (DRR)");

	loadInputVolume();
	g_Ps=ProjTable::loadProjectionsOneMatrixPerLine(GetSet<>("Input/Projection Matrices"));

	return g_app.exec();
}
