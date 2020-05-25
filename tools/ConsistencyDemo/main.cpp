
// Utilities for Displaying Images and Plots
#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>
using UtilsQt::Figure;
using UtilsQt::Plot;

// A Simple QT Utility to Show a Settings Window
#include <GetSetGui/GetSetGui.h>
GetSetGui::Application g_app("ConsistencyDemo");

/// f(x)=1-x^2+x^4 is zero at +/-1, has zero derivative at +/-1 and a maxiumum at f(0)=1; Values outside [-1,1] are clamped to zero. 
inline double weighting(double x)
{
	if (x<-1.0||x>1.0) return 0;
	double xx=x*x;
	return 1.0-2*xx+xx*xx;
}

/// Intersection length of a ray with a pixel given its central distance.
class PixelIntersection {
	double d1;
	double d2;
	double dmax;
public:
	PixelIntersection(double alpha)
	{
		double cosa(std::cos(alpha));
		double sina(std::sin(alpha));
		double dodd =std::abs(sina+cosa);
		double deven=std::abs(sina-cosa);
		d1=std::max(dodd,deven);
		d2=std::min(dodd,deven);
		dmax=sqrt(2)*std::max(std::abs(sina),std::abs(cosa));
	}

	inline double length(double d)
	{
		if (d<0)d*=-1;
		if (d<d2) return dmax;
		if (d>d1) return 0;
		return dmax*(1.0-(d-d2)/(d1-d2));
	}

};

// draws a line onto an image. Line is relative to image center. l is converted to HNF and moved to image corner.
void drawLine(NRRD::ImageView<float> &img, double l[], double width=1) 
{
	// Image size
	int nx=img.size(0);
	int ny=img.size(1);
	// Convert line to HNF
	double norm=std::sqrt(l[0]*l[0]+l[1]*l[1]);
	l[0]/=norm;
	l[1]/=norm;
	l[2]/=norm;
	PixelIntersection intersection(std::atan2(l[1],-l[0]));

	// Make line relative to image corner.
	double dist=l[0]*0.5*nx+l[1]*0.5*ny;
	l[2]-=dist;

	// Loop over each pixel
	for (int y=0;y<ny;y++)
		for (int x=0;x<nx;x++)
		{
			// Compute current pixel's distance to line.
			double distance=l[0]*x+l[1]*y+l[2];
			// Assign pixel value based on weighted distance
			double weight=weighting(distance/width);
//			double weight=intersection.length(distance);
			img.pixel(x,y)+=weight;
		}
}

/// A call-back function to handle GUI-input
void gui(const GetSetInternal::Node& node)
{

	// When the button has been clicked
	if (node.name=="Update")
	{
		std::string path=GetSet<>("Settings/Sinogram");
		NRRD::Image<float> sinogram(path);
		if (!sinogram) return;
		Figure("Sinogram",sinogram);

		// Access size and number of Radon bins
		auto &meta=sinogram.meta_info;
		double step_a=stringTo<double>(meta["Bin Size/Angle"]);
		double step_t=stringTo<double>(meta["Bin Size/Distance"]);
		int n_a=sinogram.size(0);
		int n_t=sinogram.size(1);

		// Number of image moment to consisder
		int n_n=10;

		// Loop over order of moments
		for (int n=0;n<n_n;n++)
		{
			std::vector<double> as;
			std::vector<double> moments;

			for (int ia=0.5*n_a;ia<=4.5*n_a;ia++)
			{
				double a=(ia-0.5*n_a)*step_a;
				int sa=ia;
				bool flip=false;
				while (sa<0)
					sa+=2*n_a;
				while (sa>=n_a) {
					sa-=n_a;
					flip=!flip;
				}
				double moment=0;
				for (int it=0;it<n_t;it++)
				{
//					double t=(it-0.5*n_t)*step_t;
					double t=(it-0.5*n_t)/(0.5*n_t);
					if (flip) t*=-1;
					double sample=sinogram(sa,it)*0.308;
					moment+=sample*std::pow(t,n);
				}
				// Store data for plot in vectors
				as.push_back(a);
				moments.push_back(moment*step_t);
			}

			std::string graph_name="HLCC order "+toString(n);
			Plot plot(graph_name);
			plot.graph()
				.setName(graph_name)
				.setData(as,moments);
			auto color=QColor(128,0,0);
			plot.drawVerticalLine(Geometry::Pi*1.0,color,1);
			plot.drawVerticalLine(Geometry::Pi*2.0,color,1);
			plot.drawVerticalLine(Geometry::Pi*3.0,color,1);
			plot.setAxisAngularX(false);
			plot.setAxisLabels("Projection Angle","Moment Curve [a.u.]");
			plot.showLegend();
		}

	}
		
	if (node.name=="Algebraic System Matrix")
	{
		int p=GetSet<int> ("Settings/Trajectory/Number of Projections");   // object size in pixels (n*n)
		int n=GetSet<int> ("Settings/Trajectory/Number of Image Pixels");  // number of projectios per half circle
		int m=GetSet<int> ("Settings/Trajectory/Number of Detector Bins"); // number of detector bins

		// Trajectory/System Matrix
		NRRD::Image<float> A(n*n,p*m);		
		double range_t=std::sqrt(2)*n/2;
		// Building the Trajectory/System Matrix
		bool cancel_clicked=false;
		g_app.progressStart("Building System  Matrix","",p,&cancel_clicked);
		for (int a=0;a<p;a++)
		{
			if (cancel_clicked) break;
			g_app.progressUpdate(a);
			double angle=(double)a/p*Geometry::Pi;
			for (int bin=0;bin<m;bin++)
			{
				NRRD::ImageView<float> line_image(n,n,1,(float*)A + (a*m+bin)*n*n);
				for (int i=0;i<line_image.length();i++) line_image[i]=0;
				for (double da=-2*Geometry::Pi/p;da<=2*Geometry::Pi/p;da+=Geometry::Pi/(8*p))
				{
					double l[]={sin(angle+da),cos(angle+da),-(bin-0.5*m)};
					drawLine(line_image,l);
				}
				Figure("Line Image",line_image).update();
			}
			if (a<4)
				Figure("Trajectory/System Matrix",A);
		}
		g_app.progressEnd();
		if (cancel_clicked) return;
		Figure("Trajectory/System Matrix",A).savePNG(std::string("SystemMatrix")+toString(p)+".png");
		A.save(GetSet<>("Settings/Trajectory/System Matrix"));
	}

	if (node.super_section=="Settings/Test")
	{
		// Problem setting
		int p=GetSet<int> ("Settings/Trajectory/Number of Projections");   // object size in pixels (n*n)
		int n=GetSet<int> ("Settings/Trajectory/Number of Image Pixels");  // number of projectios per half circle
		int m=GetSet<int> ("Settings/Trajectory/Number of Detector Bins"); // number of detector bins

		// Load system matrix
		NRRD::Image<float> A_data(GetSet<>("Settings/Trajectory/System Matrix"));
		if (!(A_data.size(0)==n*n && A_data.size(0)==n*n && A_data.size(1)==p*m)) return;
		Eigen::MatrixXf A = Eigen::Map<Eigen::MatrixXf>( A_data, A_data.size(0), A_data.size(1) ).transpose();
		Figure("Trajectory/System Matrix",A_data);

		// Compute projections and show as sinogram

		if (node.name=="Projection")
		{
			NRRD::Image<float> object_data(GetSet<>("Settings/Object"));
			if (!object_data.size(0)==n && !object_data.size(1)==n ) return;
			Eigen::VectorXf x = Eigen::Map<Eigen::VectorXf>( object_data, object_data.size(1)*object_data.size(0) );
			Figure("Object",object_data);
			auto bs=(A*x).eval();
			NRRD::ImageView<float> sinogram(m,p,1,bs.data());
			sinogram.save(GetSet<>("Settings/Sinogram"));
			Figure("Sinogram", sinogram).savePNG(std::string("Sinogram")+toString(p)+".png");
		}
		else
		{
			NRRD::Image<float> sinogram_data(GetSet<>("Settings/Sinogram"));
			if (!sinogram_data.size(0)==m && !sinogram_data.size(1)==p ) return;
			Eigen::VectorXf bs = Eigen::Map<Eigen::VectorXf>( sinogram_data, sinogram_data.size(1)*sinogram_data.size(0) );
			Figure("Sinogram",sinogram_data);
			if(node.name=="Back Projection") {
				auto x=(A.transpose()*bs).eval();
				NRRD::ImageView<float> backprojection(n,n,1,x.data());
				Figure("Backprojection", backprojection).savePNG(std::string("Backprojection")+toString(p)+".png");
			}
			else // if (node.name=="Reconstruction")
			 {
				auto x=(A.colPivHouseholderQr().solve(bs)).eval();
				NRRD::ImageView<float> reconstruction(n,n,1,x.data());
				Figure("Reconstruction", reconstruction).savePNG(std::string("Reconstruction")+toString(p)+".png");
			}
		}

	}

	if (node.name=="Blubb")
	{
		double alpha=GetSet<double>("Settings/Blubb")/180*Geometry::Pi;
		std::vector<double> vp,vd;
		PixelIntersection p(alpha);
		for (double d=-2;d<2;d+=0.01)
		{
			vd.push_back(d);
			vp.push_back(p.length(d));
		}
		UtilsQt::Plot("Projection of Single Pixel",vd,vp);

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
	GetSetGui::File("Settings/Sinogram")="sinogram.nrrd";
	GetSetGui::File("Settings/Object")="object.nrrd";

	GetSetGui::Button("Settings/Update")="Update...";
	GetSetGui::Slider("Settings/Blubb").setMax(180)=90;

	GetSetGui::Button("Settings/Algebraic System Matrix")="Compute...";
	GetSetGui::Button("Settings/Test/Projection")="Compute...";
	GetSetGui::Button("Settings/Test/Back Projection")="Compute...";
	GetSetGui::Button("Settings/Test/Reconstruction")="Compute...";
	

	GetSet<int>    ("Settings/Trajectory/Number of Projections")=92;
	GetSet<int>    ("Settings/Trajectory/Number of Image Pixels")=32;
	GetSet<int>    ("Settings/Trajectory/Number of Detector Bins")=48;
	GetSetGui::File("Settings/Trajectory/System Matrix")="system_matrix.nrrd";

	// Run application
	g_app.init(argc,argv,gui);
	g_app.window().addMenuItem("File","Show Plot Editor...");
	g_app.window().addMenuItem("File"),
	g_app.window().addDefaultFileMenu();
	return g_app.exec();
}
