#include <iostream>

#include <GetSetGui/GetSetGui.h>

#include <LibUtilsQt\Figure.hxx>

// NRRD file format
#include <NRRD/nrrd_image.hxx>

#include <Eigen/Geometry>

// Geometry: Projection matrices and epipolar geometry
#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibProjectiveGeometry/CameraOpenGL.hxx>

// Timing Cost function evaluations when plotting
#include <Utils/TimerWin32.hxx>

// LibSimple
#include <LibSimple/SimpleMHD.h>
#include <LibSimple/SimpleVR.h>
#include <LibSimple/SimpleEmptySpaceSkipping.h>
#include <LibSimple/SimpleQTGL.h>


// Main OpenGL Window
SimpleQTGL					*g_mainWindow=0x0;

// A window automatically generated from properties and ini-File to save those.
GetSetGui::Application		g_app("ComputeDRR");

// Textures
SimpleGL::Texture			g_image2D;
SimpleGL::Texture			g_image3D;
SimpleGL::Texture			g_textureTarget;

// Ray-caster
SimpleVR::ProxySubCubes		*g_ess=0x0;
SimpleVR::VolumeRenderer	g_vr;
std::string					g_save_path; // if not empty, render() will save result image here

// Passes
SimpleVR::DigitallyReconstructedRadiograph	*g_drrPass=0x0;
SimpleVR::IsoSurface						*g_isoPass=0x0;


bool						g_reload2D=true;
bool						g_reload3D=true;
bool						g_update3D=true;

// Mouse in GL window
int							g_mouseX=0;
int							g_mouseY=0;
int							g_mouseButton=0;

// pre-declaration of gui(...) callback function
void gui(const std::string&,const std::string&);

// Function to determine file type by extension
std::string getExtension(std::string path)
{
	return splitRight(path,".");
}

// OpenGL drawing function
void draw()
{
	double w=g_mainWindow->width();
	double h=g_mainWindow->height();
	glad_glClearColor(1,1,1,1);
	glad_glClear(GL_COLOR_BUFFER_BIT);

	if (g_reload2D)
	{
		g_reload2D=false;
		std::string path=GetSet<>("Registration/Overlay");
		NRRD::Image<float> tmp(path);
		if (!tmp)
			std::cerr << "No overlay loaded.\n";
		else
		{
			int l=tmp.length();
			float minv=tmp[0],maxv=tmp[1];
			#pragma omp parallel for
				for (int i=0;i<l;i++)
				{
					if (maxv<tmp[i]) maxv=tmp[i];
					if (minv>tmp[i]) minv=tmp[i];
				}
			g_image2D.create(
				MHD::getTypeStr<unsigned char>(),
				tmp.size(0),tmp.size(1),1,1,
				tmp.spacing(0),tmp.spacing(1),1);
			unsigned char *d=(unsigned char *)g_image2D.getData()->raw();
			std::cout << "Converting and downloading...\n";
			float scale=255.0/(maxv-minv);
			int w=tmp.size(0), h=tmp.size(1);
			#pragma omp parallel for
				for (int y=0;y<h;y++)
					for (int x=0;x<w;x++)
						d[y*w+x]=(unsigned char)((tmp[(h-1-y)*w+x]-minv)*scale);
			g_image2D.createTexture();
		}
	}

	// Load CT image
	if (g_reload3D)
	{
		std::string path=GetSet<>("File/Input 3D File");
		std::string extension=getExtension(path);
		if (extension=="mhd" || extension=="MHD")
		{
			// Load HHD. ESS noch supported. FIXME
			g_image3D.loadMHD(path);
			if (!g_image3D.getData()) {
				g_mainWindow->hide();
				return;
			}
			g_image3D.createTexture();
		}
		else 
		{
			// Load NRRD Volume
			std::cout << "Loading volume...\n";
			NRRD::Image<float> tmp(path);
			if (!tmp) {
				g_mainWindow->hide();
				return;
			}
			int l=tmp.length();
			float minv=tmp[0],maxv=tmp[1];
			#pragma omp parallel for
				for (int i=0;i<l;i++)
				{
					if (maxv<tmp[i]) maxv=tmp[i];
					if (minv>tmp[i]) minv=tmp[i];
				}
			std::cout << "Value range: " << minv << " to " << maxv << std::endl;
			g_image3D.create(
				MHD::getTypeStr<unsigned char>(),
				tmp.size(0),tmp.size(1),tmp.size(2),1,
				tmp.spacing(0),tmp.spacing(1),tmp.spacing(2));
			unsigned char *d=(unsigned char *)g_image3D.getData()->raw();
			std::cout << "Converting and downloading...\n";
			float scale=255.0/(maxv-minv);
			#pragma omp parallel for
				for (int i=0;i<l;i++)
					d[i]=(unsigned char)((tmp[i]-minv)*scale);
			g_image3D.createTexture();
			//g_image3D.saveMHD("last_volume");

			// Physical volume size
			std::cout << "Volume size [mm]: "
				<< g_image3D.getData()->physicalSize(0) << " x "
				<< g_image3D.getData()->physicalSize(1) << " x "
				<< g_image3D.getData()->physicalSize(2) << std::endl;

		}
		if (!g_drrPass) g_drrPass=new SimpleVR::DigitallyReconstructedRadiograph();
		if (!g_isoPass) g_isoPass=new SimpleVR::IsoSurface();
		g_isoPass->shaded=true;
		std::cout << "Building Proxy Geometry...\n";
		g_ess=new SimpleVR::ProxySubCubes(g_image3D);
		g_vr.init(&g_image3D,g_ess);
		g_vr.resize(w,h, &g_textureTarget);
		g_reload3D=false;
		g_update3D=true;
		// Apply values from GUI
		gui("Visualization/Transfer Function","");
		gui("","Ray-Cast Pass");
		gui("","Samples Per Voxel");

		return;
	}

	// Set up dummy projection
	if (!g_image3D.getData()) {
		g_mainWindow->hide();
		return;
	}

	// Get current projection
	using namespace Geometry;
	ProjectionMatrix P=GetSet<ProjectionMatrix>("Projection/Projection Matrix");
	Eigen::Matrix3d H=GetSet<Eigen::Matrix3d>("Registration/Transformation Image");
	Eigen::Matrix4d T=GetSet<Eigen::Matrix4d>("Registration/Transformation World");
	P=H*P*T;

	// Image size
	int n_x=GetSet<int>("Projection/Image Size u");
	int n_y=GetSet<int>("Projection/Image Size v");
	// Take binning into account
	int bin=GetSet<int>("Projection/Image Downsample");
	n_x/=bin;
	n_y/=bin;
	P=Eigen::Vector3d(1.0/bin,1.0/bin,1).asDiagonal()*P;

	// Convert projection matrix to OpenGL MVP
	double near=GetSet<double>("Projection/OpenGL/Clip Near");
	double far=GetSet<double>("Projection/OpenGL/Clip Far");
	bool flipv=GetSet<bool>("Projection/OpenGL/Flip v");
	bool flipu=GetSet<bool>("Projection/OpenGL/Flip u");
	Eigen::Matrix4d Pgl,MVgl;
	Geometry::projectionMatrixToOpenGL(P,MVgl,Pgl,flipu?n_x:0,flipu?0:n_x,flipv?n_y:0,flipv?0:n_y,near,far);

	//if (GetSet<bool>("Debug/Load Jian's MVP directly"))
	//{
	//	// V02
	//	GetSet<int>("Projection/OpenGL/Coordinate System")=3;
	//	Pgl << 5.42561, -5.55112e-017, 0.770068, -0.504528, -3.17047, 0, 1.31781, -0.863396, -1.38778e-017, 8.11688, 0, -3.46945e-018, -308.657, 2743.51, -441.915, 703.324;
	//	MVgl << 124.158, 0.216307, -0.225148, 0, -0.216635, 124.158, -0.180761, 0, 0.174772, 0.140818, 96.5137, 0, 0.395462, -95.9355, -338.665, 1;
	//	Pgl.transposeInPlace();
	//	MVgl.transposeInPlace();
	//}

	// Set and load GL matrices
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixd(Pgl.data());
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixd(MVgl.data());

	// Account for center of volume and patient Z-directionu
	int glmode=GetSet<int>("Projection/OpenGL/Coordinate System");
	bool centerVolume=(glmode==1||glmode==3);
	bool scaleVolumeToUnitCube=glmode>1;

	// Update GUI with current matrix state. Note these values are read-only.
	glGetDoublev(GL_PROJECTION_MATRIX,Pgl.data());
	glGetDoublev(GL_MODELVIEW_MATRIX,MVgl.data());
	GetSet<Eigen::Matrix4d>("Projection/OpenGL/GL Projection Matrix")=Pgl;
	GetSet<Eigen::Matrix4d>("Projection/OpenGL/GL ModelView Matrix")=MVgl;

	// Physical volume size
	double wmm=g_image3D.getData()->physicalSize(0);
	double hmm=g_image3D.getData()->physicalSize(1);
	double dmm=g_image3D.getData()->physicalSize(2);

	// Additional transformations to compensate varying coordinate systems
	glPushMatrix();
	if (scaleVolumeToUnitCube)
	{
		if (centerVolume)
			glScalef(2./wmm,2./hmm,2./dmm);
		else
			glScalef(1./wmm,1./hmm,1./dmm);
	}
	if (centerVolume)
		glTranslated(-wmm*0.5,-hmm*0.5,-dmm*0.5);


	// Resize Raycaster Textures and Render Target
	if (GetSet<int>("Visualization/Ray-Cast Pass")==1||!g_save_path.empty())
		g_update3D|=g_vr.resize(w,h,&g_textureTarget,false);
	else
		g_update3D|=g_vr.resize(w/2,h/2,&g_textureTarget);
	// Raycast (check if update is even needed)
	if (g_update3D)
	{
		g_vr.render(&g_textureTarget);
		g_update3D=false;
	}

	// Draw
	g_textureTarget.drawToViewport();

	// Draw Volume Bounding Box
	glColor3f(0,0,0);
	glBegin(GL_LINES);
		glVertex3f(wmm,hmm,dmm);
		glVertex3f(0,hmm,dmm);
		glVertex3f(wmm,hmm,dmm);
		glVertex3f(wmm,0,dmm);
		glVertex3f(wmm,hmm,dmm);
		glVertex3f(wmm,hmm,0);
		glVertex3f(wmm,hmm,0);
		glVertex3f(0,hmm,0);
		glVertex3f(wmm,hmm,0);
		glVertex3f(wmm,0,0);
		glVertex3f(wmm,0,dmm);
		glVertex3f(0,0,dmm);
		glVertex3f(wmm,0,dmm);
		glVertex3f(wmm,0,0);
		glVertex3f(wmm,0,0);
		glVertex3f(0,0,0);
		glVertex3f(0,hmm,dmm);
		glVertex3f(0,0,dmm);
		glVertex3f(0,hmm,dmm);
		glVertex3f(0,hmm,0);
		glVertex3f(0,hmm,0);
		glVertex3f(0,0,0);
		glVertex3f(0,0,dmm);
		glVertex3f(0,0,0);
	glEnd();	
	// Draw XYZ Coord System (1cm axis length) according to MVP
	glBegin(GL_LINES);
		glColor3f(1,0,0);
		glVertex3f(0,0,0);
		glVertex3f(10,0,0);
		glColor3f(0,1,0);
		glVertex3f(0,0,0);
		glVertex3f(0,10,0);
		glColor3f(0,0,1);
		glVertex3f(0,0,0);
		glVertex3f(0,0,10);
	glEnd();
	glColor3f(1,1,1);


	glPopMatrix();

	if (GetSet<bool>("Registration/Show Overlay"))
	{
		glColorMask(0,1,1,1);
		g_image2D.drawToViewport();
		glColorMask(1,1,1,1);
	}

	// Save.
	if (!g_save_path.empty())
	{
		g_textureTarget.bind();
		g_textureTarget.upload();
		g_textureTarget.unbind();
		NRRD::ImageView<float> out(g_textureTarget.getData()->dim(0),g_textureTarget.getData()->dim(1),1,(float*)g_textureTarget.getData()->raw());
		#pragma omp parallel for
		for (int y=0;y<out.size(1)/2;y++)
			for (int x=0;x<out.size(0);x++)
				std::swap(out.pixel(x,y,0),out.pixel(x,h-1-y,0));
		Eigen::Matrix3d H=GetSet<Eigen::Matrix3d>("Registration/Transformation Image");
		Eigen::Matrix4d T=GetSet<Eigen::Matrix4d>("Registration/Transformation World");
		ProjectionMatrix P=GetSet<ProjectionMatrix>("Projection/Projection Matrix");
		P=H*P*T;
		out.meta_info["Projection Matrix"]=toString(P);
		std::cout << out.meta_info["Projection Matrix"] << std::endl;
		if (!out.save(g_save_path))
			std::cerr << "Failed to write image " << g_save_path << std::endl;
		else
			std::cout << "Saved image " << g_save_path << std::endl;
		g_save_path.clear();
		g_update3D=true;
	}
}

void mouse_click(int button, int state, int x, int y)
{
	if (state)
		g_mouseButton=button;
	else
		g_mouseButton=-1;
}

 //std::ofstream mo;
void mouse_move(int dx, int dy)
{
	dx*=2;
	dy*=2;
	using namespace Geometry;
	int input_mode=GetSet<int>("Registration/User Input Mode");
	Eigen::Matrix3d H=GetSet<Eigen::Matrix3d>("Registration/Transformation Image");
	Eigen::Matrix4d T=GetSet<Eigen::Matrix4d>("Registration/Transformation World");
	ProjectionMatrix P=GetSet<ProjectionMatrix>("Projection/Projection Matrix");
	P=H*P*T;
	// mo << toString(P) << std::endl;
	// Get Camera coordinate system (assuming zero skew)
	Eigen::Vector3d U=P.block<1,3>(0,0).transpose().normalized();
	Eigen::Vector3d V=P.block<1,3>(1,0).transpose().normalized();
	Eigen::Vector3d m3=P.block<1,3>(2,0).transpose().normalized();
	// Transformations of image and world
	Eigen::Matrix3d Hp=Eigen::Matrix3d::Identity();
	Eigen::Matrix4d Tp=Eigen::Matrix4d::Identity();

	switch (input_mode)
	{
	case 0: // World Translation
		if (g_mouseButton==0)
		{
			// Shifts parallel to detector
			Tp.block<3,1>(0,3)+=U*dx+V*dy;
		}
		else
		{
			// Translation out of plane
			Tp.block<3,1>(0,3)+=m3*(dx+dy);
		}
		break;
	case 1: // Image Translation and rotation
		if (g_mouseButton==0)
		{
			// Detector Shifts
			Hp(0,2)=-dx;
			Hp(1,2)=dy;
		}
		else
		{
			// Rotation in-plane
			double a=dx*0.005;
			Hp(0,0)=cos(a);
			Hp(1,0)=sin(a);
			Hp(0,1)=-sin(a);
			Hp(1,1)=cos(a);
		}
		break;
	case 2: // World Rotation
		if (g_mouseButton==0)
		{
			// Approximate U and V axes (assuming zero skew)
			Eigen::Matrix3d R;
			R=Eigen::AngleAxisd(dy*0.005,U)*Eigen::AngleAxisd(dx*0.005,V);
			Tp.block<3,3>(0,0)=R;
		}
		else
		{
			// Rotation about principal axis
			Eigen::Matrix3d R;
			R=Eigen::AngleAxisd(dx*0.005,m3);
			Tp.block<3,3>(0,0)=R;
		}
		break;
	}
	// Update Transformations
	H=H*Hp;
	T=T*Tp;
	GetSet<Eigen::Matrix3d>("Registration/Transformation Image")=H;
	GetSet<Eigen::Matrix4d>("Registration/Transformation World")=T;
}

void mouse_wheel(int dx, int dy)
{
	int input_mode=GetSet<int>("Registration/User Input Mode");
	Eigen::Matrix4d T=GetSet<Eigen::Matrix4d>("Registration/Transformation World");
	double s=(1+(dx+dy)*0.01);
	T=T*Eigen::Vector4d(s,s,s,1).asDiagonal();
	GetSet<Eigen::Matrix4d>("Registration/Transformation World")=T;
}

std::vector<Geometry::ProjectionMatrix> loadMatrices()
{
	using Geometry::ProjectionMatrix;
	std::ifstream in_file(GetSet<>("Trajectory/Projection Matrices").getString());
	std::vector<ProjectionMatrix> Ps;
	while  (!in_file.eof() && in_file.good())
	{
		std::string line;
		std::getline(in_file,line,'\n');
		if (!line.empty())
			Ps.push_back(stringTo<ProjectionMatrix>(line));
	}
	return Ps;
}

void gui(const std::string& section, const std::string& key);
void gui_new(const GetSetInternal::Node& node)
{
	gui(node.name, node.super_section);
}

// Call back function for GUI interaction
void gui(const std::string& section, const std::string& key)
{
	static bool block=false;
	if (block) return;
	
	if (key=="GL Projection Matrix"||key=="GL ModelView Matrix") return;

	if (section=="Projection")
	{
		int n_x=GetSet<int>("Projection/Image Size u");
		int n_y=GetSet<int>("Projection/Image Size v");
		int bin=GetSet<int>("Projection/Image Downsample");
		if (bin<1)
			GetSet<int>("Projection/Image Downsample")=1;
		else
			g_mainWindow->setFixedSize(n_x/bin,n_y/bin);
	}

	if (key=="MVP")
	{
		auto MVP=GetSet<Eigen::Matrix4d>("Debug/MVP").getValue();
		int n_x=GetSet<int>("Projection/Image Size u");
		int n_y=GetSet<int>("Projection/Image Size v");
		bool flipv=GetSet<bool>("Projection/OpenGL/Flip v");
		if (GetSet<bool>("Debug/MVP Transpose"))
			MVP.transposeInPlace();
		auto P=Geometry::modelViewProjectionMatrixOpenGL(MVP,n_x,n_y,flipv);

		Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
		T.block<3,1>(0,3)=Eigen::Vector3d(125,125,96);// CoSy fix

		// Set result
		GetSet<Geometry::ProjectionMatrix>("Projection/Projection Matrix")=P*T;
	}

	if (section=="Registration")
		g_update3D=true;

	// Re-Load 3D Image
	if (key=="Input 3D File")
	{
		g_reload3D=true;
		g_mainWindow->show();
	}

	if(key=="Overlay")
		g_reload2D=true;

	
	if (section=="Visualization/Transfer Function")
	{
		if (g_isoPass) g_isoPass->isoValue=GetSet<double>("Visualization/Transfer Function/Min");
		if (g_drrPass) g_drrPass->rampMin=GetSet<double>("Visualization/Transfer Function/Min");
		if (g_drrPass) g_drrPass->rampMax=GetSet<double>("Visualization/Transfer Function/Max");
		if (g_drrPass) g_drrPass->rampHeight=GetSet<double>("Visualization/Transfer Function/Height");
		g_update3D=true;
	}

	if (key=="Ray-Cast Pass")
	{
		int rcp=GetSet<int>("Visualization/Ray-Cast Pass");
		if (rcp==0) g_vr.setRayCastPass(g_drrPass);
		if (rcp==1) g_vr.setRayCastPass(g_isoPass);
		g_update3D=true;
	}

	if (key=="Samples Per Voxel")
	{
		double spv=GetSet<double>("Visualization/Samples Per Voxel");
		if (g_drrPass) g_drrPass->setSamplesPerVoxel(spv);
		if (g_isoPass) g_isoPass->setSamplesPerVoxel(spv);
		g_update3D=true;
	}

	if (key=="Projection Matrix")
		g_update3D=true;

	if (section=="Projection/OpenGL")
		g_update3D=true;

	if (section=="Projection/Look At" && key=="Apply")
	{
		using namespace Geometry;
		// Get Projection Parameters for "Look At"
		int n_x=GetSet<int>("Projection/Image Size u");
		int n_y=GetSet<int>("Projection/Image Size v");
		Eigen::Vector3d center=GetSet<Eigen::Vector3d>("Projection/Look At/Center");
		Eigen::Vector3d eye=GetSet<Eigen::Vector3d>("Projection/Look At/Eye");
		Eigen::Vector3d up=GetSet<Eigen::Vector3d>("Projection/Look At/Up");
		double fov=GetSet<int>("Projection/Look At/Field of View (degrees)");
		// Write Projection Matrix
		Eigen::Matrix3d K=Geometry::makeCalibrationMatrix(fov/(180*Pi),fov/(180*Pi),n_x,n_y);
		ProjectionMatrix P=Geometry::cameraLookAt(K,eye,center,up);
		// Set result
		GetSet<ProjectionMatrix>("Projection/Projection Matrix")=P;
	}
	
	if(key=="Load Matrix")
	{
		auto Ps=loadMatrices();
		int i=GetSet<int>("Trajectory/Load Matrix");
		if (i<0 && !Ps.empty()) GetSet<int>("Trajectory/Load Matrix")=0;
		else if (i>=Ps.size()) GetSet<int>("Trajectory/Load Matrix")=(int)Ps.size()-1;
		else if (i>=0)
			GetSet<Geometry::ProjectionMatrix>("Projection/Projection Matrix")=Ps[i];
	}

	if (key=="Save DRR")
		g_save_path=GetSet<>("File/Output 2D File");

	if (key=="Compute DRRs")
	{
		block=true;
		using Geometry::ProjectionMatrix;
		std::string out_path=GetSet<>("File/Output 2D File");
		
		std::vector<ProjectionMatrix> Ps=loadMatrices();
		g_app.progressStart("Computing DRRs...","Computing DRRs...",(int)Ps.size()-1);
		for (int i=0;i<(int)Ps.size();i++)
		{
			g_app.progressUpdate(i);
			GetSet<ProjectionMatrix>("Projection/Projection Matrix")=Ps[i];
			QCoreApplication::processEvents();
			g_mainWindow->makeCurrent();
			g_save_path=GetSet<>("Trajectory/Output Directory").getString()+"/"+toString(i,3)+".nrrd";
			g_mainWindow->update();
			g_mainWindow->doneCurrent();
			QCoreApplication::processEvents();
		}
		block=false;
		g_app.progressEnd();
	}

	// Load settings from file
	g_app.saveSettings();
	
	// Refresh	
	g_mainWindow->postRedisplay();
}

int main(int argc, char ** argv)
{

	// File Tab
	GetSetGui::File("File/Input 3D File")
		.setExtensions("NRRD File (*.nrrd);;Meta Image (*.mhd);;All Files (*)")
		.setDescription("A CT image.  If you have DICOM or other image format, try ConvertITK tool shipped with this software.");
	GetSetGui::File("File/Output 2D File")
		.setExtensions("NRRD File (*.nrrd);;Meta Image (*.mhd);;All Files (*)")
		.setCreateNew(true)
		.setDescription("Output DRR image.  If you need other image formats, try ConvertITK tool shipped with this software.");
	GetSetGui::Button("File/Save DRR")="Compute...";

	// Trajectory Tab
	GetSetGui::File("Trajectory/Projection Matrices").setExtensions("One Matrix Per Line (*.txt)");
	GetSetGui::Directory("Trajectory/Output Directory");
	GetSetGui::Button("Trajectory/Compute DRRs")="Compute...";
	GetSet<int>("Trajectory/Load Matrix")=0;

	// Visualization/Transfer Function Tab
	GetSetGui::Slider("Visualization/Transfer Function/Min")=0.1;
	GetSetGui::Slider("Visualization/Transfer Function/Max")=1.0;
	GetSet<double>("Visualization/Transfer Function/Height")=1;
	GetSetGui::Enum("Visualization/Ray-Cast Pass").setChoices("Digitally Reconstructed Radiograph;Iso-Surface")=0;
	GetSet<double>("Visualization/Samples Per Voxel")=1.5;

	// Projection Tab
	GetSet<Geometry::ProjectionMatrix>("Projection/Projection Matrix");
	GetSet<int>("Projection/Image Size u")=1024;
	GetSet<int>("Projection/Image Size v")=768;
	GetSet<int>("Projection/Image Downsample")=1;
	GetSet<Eigen::Vector3d>("Projection/Look At/Center")=Eigen::Vector3d(0,0,0);
	GetSet<Eigen::Vector3d>("Projection/Look At/Eye")=Eigen::Vector3d(100,100,100);
	GetSet<Eigen::Vector3d>("Projection/Look At/Up")=Eigen::Vector3d(0,1,0);
	GetSetGui::Button("Projection/Look At/Apply")="Look At";
	GetSet<int>("Projection/Look At/Field of View (degrees)")=40;

	// Projection/OpenGL settings
	GetSet<double>("Projection/OpenGL/Clip Far")=5000;
	GetSet<double>("Projection/OpenGL/Clip Near")=10;
	GetSet<bool>("Projection/OpenGL/Flip u").setDescription("Corrects for left-handed image coordinate systems (i.e. Image u-axis points left)")=false;
	GetSet<bool>("Projection/OpenGL/Flip v").setDescription("Corrects for left-handed image coordinate systems (i.e. Image v-axis points down)")=false;
	// Access (read-only) to OpenGL matrices
	GetSetGui::Enum("Projection/OpenGL/Coordinate System").setChoices("World millimeters, origin at corner;World millimeters, origin in center;Unit Cube;Centered Unit Cube");
	GetSet<Eigen::Matrix4d>("Projection/OpenGL/GL Projection Matrix");
	GetSet<Eigen::Matrix4d>("Projection/OpenGL/GL ModelView Matrix");

	// Registration Tab
	GetSet<Eigen::Matrix3d>("Registration/Transformation Image")=Eigen::Matrix3d::Identity();
	GetSet<Eigen::Matrix4d>("Registration/Transformation World")=Eigen::Matrix4d::Identity();
	GetSetGui::Enum("Registration/User Input Mode").setChoices("World Translation;Image Translation and Rotation;World Rotation")=0;
	GetSetGui::File("Registration/Overlay").setExtensions("NRRD file (*.nrrd);;All Files (*)");
	GetSet<bool>("Registration/Show Overlay")=true;


	GetSet<Eigen::Matrix4d>("Debug/MVP");
	GetSet<bool>("Debug/MVP Transpose")=true;

	g_app.init(argc,argv,gui_new);


	// OpenGL window
	g_mainWindow=new SimpleQTGL();
	g_mainWindow->setDisplayFunc(draw);
	g_mainWindow->setMouseFunc(mouse_click);
	g_mainWindow->setMotionFunc(mouse_move);
	g_mainWindow->setWheelFunc(mouse_wheel);

	// Initialize ray-cast pass
	gui("Projection","");
	gui("","Input 3D File");
	gui("","Ray-Cast Pass");

	return g_app.exec();
}
