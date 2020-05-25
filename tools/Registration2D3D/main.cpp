
#include <iostream>

// Some Qt GUI Components
#include <QApplication>
#include <QProgressBar>

#include <LibUtilsQt/nrrdqt.hxx>

// NRRD file format
#include <NRRD/nrrd_image.hxx>
#include <NRRD/nrrd_local_crorrelation.hxx>

// Geometry: Projection matrices and epipolar geometry
#include <Geometry/ProjectiveGeometry.hxx>
#include <Geometry/ProjectionMatrix.hxx>
#include <Geometry/PinholeCameraDecomposition.hxx>
#include <Geometry/EigenToSrtr.hxx>

// Managing saving/loading parametzers and automatic GUI
#include <GetSet/GetSet.hxx>
#include <GetSet/GetSetIO.h>
#include <GetSetGui/GetSetSettingsWindow.h>

// Timing Cost function evaluations when plotting
#include <Utils/TimerWin32.hxx>

// LibSimple
#include <SimpleMHD.h>
#include <SimpleVR.h>
#include <SimpleQTGL.h>

#include <SimpleEmptySpaceSkipping.h>

// A window automatically generated from properties and ini-File to save those.
GetSetSettingsWindow		*g_mainSettings=0x0;
std::string					g_iniFile="Registration2D3D.ini";

// Images
NRRD::Image<float>			g_image2D;
NRRD::Image<float>			g_image2D_DRR;
SimpleGL::Texture			g_image3D;
SimpleGL::Texture			g_texture2D;
SimpleGL::Texture			g_textureTarget;
bool						g_reload3D=true;
bool						g_reload2D=true;

enum {Nothing, EvaluateOnce}			g_actionWithContext=Nothing;

// Ray-caster
SimpleQTGL					*g_drr=0x0;
SimpleVR::ProxySubCubes		*g_ess=0x0;
SimpleVR::VolumeRenderer	g_vr;
// Passes
SimpleVR::DebugRaycastEnvironment			*g_debugPass=0x0;
SimpleVR::DigitallyReconstructedRadiograph	*g_drrPass=0x0;
SimpleVR::IsoSurface						*g_isoPass=0x0;
SimpleVR::MaximumIntensityProjection		*g_mipPass=0x0;

// Mouse in GL window
int g_mouseX=0;
int g_mouseY=0;
int g_mouseButton=0;

double fixme0=0;
double fixme1=0;
double fixme2=2000;

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
	double w=g_drr->width();
	double h=g_drr->height();
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT);
	// Download 2D image
	if (g_reload2D)
	{
		g_texture2D.createEmptyTexture<float>(g_image2D.size(0),g_image2D.size(1));
		g_texture2D.bind();
		g_texture2D.download((float*)g_image2D);
		g_texture2D.unbind();
		g_reload2D=false;
		g_textureTarget.createEmptyTexture<float>(g_image2D.size(0),g_image2D.size(1));
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
				g_drr->hide();
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
				g_drr->hide();
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
			#pragma omp parallel for
				for (int i=0;i<l;i++)
					d[i]=(unsigned char)(255.0*((tmp[i]-minv)/(maxv-minv)));
			g_image3D.createTexture();
			
		}
		if (!g_debugPass) g_debugPass=new SimpleVR::DebugRaycastEnvironment();
		if (!g_drrPass) g_drrPass=new SimpleVR::DigitallyReconstructedRadiograph();
		if (!g_isoPass) g_isoPass=new SimpleVR::IsoSurface();
		if (!g_mipPass) g_mipPass=new SimpleVR::MaximumIntensityProjection();
		g_isoPass->shaded=true;
		std::cout << "Building Proxy Geometry...\n";
		g_ess=new SimpleVR::ProxySubCubes(g_image3D);
		g_vr.init(&g_image3D,g_ess);
		g_vr.resize(w,h);
		g_reload3D=false;
		// Apply values from GUI
		gui("Visualization/Transfer Function","");
		gui("","Ray-Cast Pass");
		gui("","Samples Per Voxel");
		if (!g_texture2D.getData())
			g_textureTarget.createEmptyTexture<float>(512,512);
		return;
	}

	// Set up dummy projection
	if (!g_image3D.getData()) {
		g_drr->hide();
		return;
	}

	// Physical volume size
	int wmm=g_image3D.getData()->spacing(0)*g_image3D.getData()->dim(0);
	int hmm=g_image3D.getData()->spacing(1)*g_image3D.getData()->dim(1);
	int dmm=g_image3D.getData()->spacing(2)*g_image3D.getData()->dim(2);

	// Dummy Projection
	if (GetSet<>("Projection/OpenGL Projection Matrix").getString().empty())
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		double near=GetSet<double>("Projection/Clip Near");
		double far=GetSet<double>("Projection/Clip Far");
		gluPerspective(45.0,w/h,near,far);
		Eigen::Matrix4d P;
		glGetDoublev(GL_PROJECTION_MATRIX,P.data());
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(wmm/2+1000,hmm/2,dmm/2+1000,wmm/2,hmm/2,dmm/2,0,1,0);
		Eigen::Matrix4d MV;
		glGetDoublev(GL_MODELVIEW_MATRIX,MV.data());
		GetSet<>("Projection/OpenGL ModelView Matrix")=toString(MV);
		GetSet<>("Projection/OpenGL Projection Matrix")=toString(P);
	}
	else
	{
		Eigen::Matrix4d Pgl=stringTo<Eigen::Matrix4d>(GetSet<>("Projection/OpenGL Projection Matrix"));
		Eigen::Matrix4d MVgl=stringTo<Eigen::Matrix4d>(GetSet<>("Projection/OpenGL ModelView Matrix"));
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixd(Pgl.data());
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixd(MVgl.data());
	}

	if (GetSet<bool>("Visualization/Overlay"))
	{
		g_vr.render(&g_textureTarget);
		g_texture2D.drawToViewport();
		glColorMask(GL_TRUE,GL_FALSE,GL_FALSE,GL_FALSE);
		g_textureTarget.drawToViewport();
		glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
	}
	else
		g_vr.render();

	// Now the computations that need an OpenGL context
	if (g_actionWithContext==Nothing || g_texture2D==0 || g_textureTarget==0)
		return;

	if (g_actionWithContext==EvaluateOnce)
	{
		g_textureTarget.bind();
		g_textureTarget.upload((float*)g_image2D_DRR);
		g_textureTarget.unbind();
		NRRD::Image<float> out(g_image2D.size(0),g_image2D.size(1));
		NRRD::Image<unsigned char> mask(g_image2D.size(0),g_image2D.size(1));
		int l=out.length();
		#pragma omp parallel for
			for (int i=0;i<l;i++) mask[i]=255;
		NRRD::computeLocalCorrelation2D<float,unsigned char>(g_image2D,g_image2D_DRR,7,mask,255,out);
		if (!out.save(GetSet<>("File/Output 2D File")))
			std::cout << "Correlatino file not saved.\n";
		QtUtils::showImage(out,1,127.5);
	}

}

// GL window resized
void reshape(int x, int y)
{
	double aspect=(double)g_image2D.size(0)/g_image2D.size(1);
	int nw=aspect*y;
	if (nw!=x)
		g_drr->resize(nw,y);
	else
		g_vr.resize(g_drr->width(),g_drr->height());
}

// Mouse clicked in OpenGL window
void mouse(int button, int state, int x, int y)
{
	std::cout << button << "\n";
	g_mouseButton=button;
	g_mouseX=x;
	g_mouseY=y;
}

// Mouse dragged in GL window
void motion(int x, int y)
{
	double dx=x-g_mouseX;
	double dy=g_mouseY-y;
	g_mouseX=x;
	g_mouseY=y;
	Eigen::Matrix4d MVgl=stringTo<Eigen::Matrix4d>(GetSet<>("Projection/OpenGL ModelView Matrix"));
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixd(MVgl.data());
	Eigen::Vector3d nx=MVgl.block<1,3>(1,0);
	Eigen::Vector3d ny=MVgl.block<1,3>(0,0);
	Eigen::Vector3d nz=nx.cross(ny);
	nx.normalize();
	ny.normalize();
	nz.normalize();
	if (g_mouseButton==0)
	{
		glRotatef(dx,ny(0),ny(1),ny(2));
		glRotatef(dy,nx(0),nx(1),nx(2));
	}
	else if (g_mouseButton==1)
	{
		nz*=dy;
		glTranslatef(nz(0),nz(1),nz(2));
		glRotatef(dx,0,0,1);
	}
	else
	{
		Eigen::Vector3d t=dx*nx+dy*ny;
		glTranslatef(t(0),t(1),t(2));
	}
	glGetDoublev(GL_MODELVIEW_MATRIX,MVgl.data());
	GetSet<>("Projection/OpenGL ModelView Matrix")=toString(MVgl);
	g_drr->postRedisplay();
}

// Retreive optimizer settings from GUI
void getGuiOptimizerSettings(Geometry::PinholeCameraDecomposition &T, std::vector<double>& param_min_restrcted, std::vector<double>& param_max_restrcted)
{
	// Find active parameters and ranges
	auto param = Geometry::PinholeCameraDecomposition::ParameterNames();
	for (int i = 0; i < T.size(); i++)
	{
		std::string    category="00 Intrinsic Parameters/";
		     if (i>21) category="03 World Transformation/";
		else if (i>10) category="02 Image Transformation/";
		else if (i>4)  category="01 Extrinsic Parameters/";
		std::string path=std::string("Optimize/") + category + toString(i, 2) + " " + param[i];
		if (GetSet<bool>(path + "/Active"))
		{
			T.activate(i);
			param_max_restrcted.push_back(GetSet<double>(path + "/Upper Bound"));
			param_min_restrcted.push_back(GetSet<double>(path + "/Lower Bound"));
		}
	}
}

// Call back function for GUI interaction
void gui(const std::string& section, const std::string& key)
{
	// Load both 2D and 3D data
	if (key=="Load Data")
	{
		gui("","Input 2D File");
		gui("","Input 3D File");
	}

	// Re-Load 3D Image
	if (key=="Input 3D File")
	{
		g_reload3D=true;
		g_drr->show();
	}
	
	// Re-Load 2D Image
	if (key=="Input 2D File")
	{
		std::string path=GetSet<>("File/Input 2D File");
		std::string extension=getExtension(path);
		if (extension=="mhd" || extension=="MHD")
		{
			MHD::Image tmp;
			tmp.loadMHD(path);
			tmp.convert(MHD::getTypeStr<float>());
			g_image2D.set(0,0,0,0);
			if (tmp.getData())
				g_image2D.set(
					tmp.getData()->dim(0),
					tmp.getData()->dim(1),
					tmp.getData()->dim(2),
					(float*)tmp.getData()->raw()); // 2do: spacing !!! FIXME
		}
		else 
			g_image2D.load(path);
		// If successful, show image
		if (!g_image2D)
			std::cerr << "Failed to load image.\n";
		else
		{
			float min=g_image2D[0],max=g_image2D[0];
			int l=g_image2D.length();
			#pragma omp parallel for
				for (int i=0;i<l;i++)
				{
					if (g_image2D[i]<min) min=g_image2D[i];
					if (g_image2D[i]>max) max=g_image2D[i];
				}
			auto pxm=QtUtils::nrrdToQPixmap(g_image2D,-min,255/(max-min));
			auto window=new QLabel();
			window->setAttribute(Qt::WA_DeleteOnClose);
			window->setPixmap(pxm.scaledToWidth(800));
			window->show();
		}
		g_reload2D=true;
	}

	if (section=="Visualization/Transfer Function")
	{
		if (g_isoPass) g_isoPass->isoValue=GetSet<double>("Visualization/Transfer Function/Min");
		if (g_drrPass) g_drrPass->rampMin=GetSet<double>("Visualization/Transfer Function/Min");
		if (g_drrPass) g_drrPass->rampMax=GetSet<double>("Visualization/Transfer Function/Max");
		if (g_drrPass) g_drrPass->rampHeight=GetSet<double>("Visualization/Transfer Function/Height");
		g_drr->postRedisplay();
	}

	if (key=="Ray-Cast Pass")
	{
		int rcp=GetSet<int>("Visualization/Ray-Cast Pass");
		if (rcp==0) g_vr.setRayCastPass(g_debugPass);
		if (rcp==1) g_vr.setRayCastPass(g_drrPass);
		if (rcp==2) g_vr.setRayCastPass(g_isoPass);
		if (rcp==3) g_vr.setRayCastPass(g_mipPass);
		g_drr->postRedisplay();
	}

	if (key=="Samples Per Voxel")
	{
		double spv=GetSet<double>("Visualization/Samples Per Voxel");
		if (g_isoPass) g_debugPass->setSamplesPerVoxel(spv);
		if (g_drrPass) g_drrPass->setSamplesPerVoxel(spv);
		if (g_drrPass) g_isoPass->setSamplesPerVoxel(spv);
		if (g_drrPass) g_mipPass->setSamplesPerVoxel(spv);
	}

	if (key=="Evaluate Similarity")
		g_actionWithContext=EvaluateOnce;

	// Update Projection Stuff
	static bool setProjection=false;
	if (!setProjection)
	{
		if (key=="Projection Matrix")
		{
			setProjection=true;
			Eigen::Matrix4d MVgl,Pgl;
			Geometry::ProjectionMatrix P=stringTo<Geometry::ProjectionMatrix>(GetSet<>("Projection/Projection Matrix"));
			int w=g_image2D.size(0);
			int h=g_image2D.size(1);
			double far=GetSet<double>("Projection/Clip Far");
			double near=GetSet<double>("Projection/Clip Near");
			Geometry::projectionMatrixToOpenGL(P,MVgl,Pgl,0,w,0,h,near,far);
			GetSet<>("Projection/OpenGL Projection Matrix")=toString(Pgl);
			GetSet<>("Projection/OpenGL ModelView Matrix")=toString(MVgl);
			setProjection=false;
		}
		if (key=="OpenGL Projection Matrix"||key=="OpenGL ModelView Matrix")
		{
			setProjection=true;
			int w=g_image2D.size(0);
			int h=g_image2D.size(1);
			Eigen::Matrix4d Pgl=stringTo<Eigen::Matrix4d>(GetSet<>("Projection/OpenGL Projection Matrix"));
			Eigen::Matrix4d MVgl=stringTo<Eigen::Matrix4d>(GetSet<>("Projection/OpenGL ModelView Matrix"));
			Geometry::ProjectionMatrix P=Geometry::modelViewProjectionMatrix(Pgl*MVgl,w,h,true);
			GetSet<>("Projection/Projection Matrix")=toString(P);
			setProjection=false;
		}
	}

	// Load settings from file
	GetSetIO::save<GetSetIO::IniFile>(g_iniFile);
}

int main(int argc, char ** argv)
{
	if (argc > 2)
	{
		std::cerr << "Usage:\n   FluoroTracking [file.ini]\n";
		return 1;
	}
	else if (argc == 2)	
		g_iniFile = argv[1];
	QApplication app(argc, argv);

	// File Tab
	GetSetGui::File("File/Input 2D File")
		.setExtensions("NRRD File (*.nrrd);;Meta Image (*.mhd);;All Files (*)")
		.setDescription("An X-Ray image. If you have DICOM or other image format, try ConvertITK tool shipped with this software.");
	GetSetGui::File("File/Input 3D File")
		.setExtensions("NRRD File (*.nrrd);;Meta Image (*.mhd);;All Files (*)")
		.setDescription("A CT image.  If you have DICOM or other image format, try ConvertITK tool shipped with this software.");
	GetSetGui::File("File/Output 2D File")
		.setExtensions("NRRD File (*.nrrd);;Meta Image (*.mhd);;All Files (*)")
		.setCreateNew(true)
		.setDescription("Output DRR image (optional).  If you need other image formats, try ConvertITK tool shipped with this software.");
	GetSetGui::Button("File/Load Data")="Load...";

	// Optimization Tab (List of all parameters provided by model)
	auto param = Geometry::PinholeCameraDecomposition::ParameterNames();
	for (int i = 0; i < param.size(); i++)
	{
		std::string    category="00 Intrinsic Parameters/";
		     if (i>21) category="03 World Transformation/";
		else if (i>10) category="02 Image Transformation/";
		else if (i>4)  category="01 Extrinsic Parameters/";
		std::string path=std::string("Optimize/") + category + toString(i, 2) + " " + param[i];
		GetSetGui::StaticText(path + "/" + toString(i, 2)) = param[i];
		GetSet<bool>(path + "/Active") = false;
		GetSet<double>(path + "/Upper Bound") = +10;
		GetSet<double>(path + "/Lower Bound") = -10;
	}
	
	// Visualization/Transfer Function Tab
	GetSetGui::Slider("Visualization/Transfer Function/Min")=0.1;
	GetSetGui::Slider("Visualization/Transfer Function/Max")=1.0;
	GetSet<double>("Visualization/Transfer Function/Height")=1;
	GetSetGui::Enum("Visualization/Ray-Cast Pass").setChoices("Debug Environment;Digitally Reconstructed Radiograph;Iso-Surface;Maximum Intensity Projection")=0;
	GetSet<double>("Visualization/Samples Per Voxel")=1.5;
	GetSet<bool>("Visualization/Overlay")=true;

	// Projection Tab
	GetSet<>("Projection/OpenGL Projection Matrix");
	GetSet<>("Projection/OpenGL ModelView Matrix");
	GetSet<>("Projection/Projection Matrix");
	GetSet<double>("Projection/Clip Far")=1;
	GetSet<double>("Projection/Clip Near")=10000;

	// registration Tab
	GetSetGui::Button("Registration/Evaluate Similarity") = "Evaluate...";
	GetSetGui::Button("Registration/Run Optimization") = "Run...";

	// Load settings from file
	GetSetIO::load<GetSetIO::IniFile>(g_iniFile);

	// Open settings window
	g_mainSettings = new GetSetSettingsWindow();
	g_mainSettings->setWindowTitle("Settings");
	g_mainSettings->setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint | Qt::WindowSystemMenuHint);
	g_mainSettings->show();

	// OpenGL window
	g_drr=new SimpleQTGL();
	g_drr->setDisplayFunc(draw);
	g_drr->setMouseFunc(mouse);
	g_drr->setMotionFunc(motion);
	g_drr->setReshapeFunc(reshape);

	// GUI handler
	GetSetHandler callback(gui);

	return app.exec();
}