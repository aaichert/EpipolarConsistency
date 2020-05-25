// NRRD Image File Format
#include <NRRD/nrrd_image.hxx>

// Projective Geometry (including string conversion for GUI)
#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibProjectiveGeometry/EigenToSrtr.hxx>
using namespace Geometry;

// Simple CUDA Wrappers
#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>
typedef CudaUtils::BindlessTexture2D<float> CudaTexture;

// Utilities for Displaying Images and Plots
#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>
using UtilsQt::Figure;
using UtilsQt::Plot;

// A Simple QT Utility to Show a Settings Window
#include <GetSetGui/GetSetGui.h>
GetSetGui::Application g_app("TobiasFBCC");

// Transforming line coordinates
#include "LinePerspectivity.hxx"

/// Actual work done in CUDA
extern void cuda_computeLineIntegrals(
	float* lines_d, short line_stride, short n_lines,  // Lines in Hessian normal form, number of float values to next line and number of lines
	float *fbcc_d, short fbcc_stride,                  // Optional: FBCC_weighting_info
	cudaTextureObject_t I, short n_u, short n_v,       // The image and its size
	float *integrals_out_d);                           // Output memory: the integrals (size is n_lines)

/// Range of angle kappa for two projections, assuming the object is bounded by a sphere with object_radius_mm around origin.
std::pair<double,double> ecc_range_kappa_heuristic(const RP3Line& B, double object_radius_mm)
{
	// Distance of baseline to origin
	double baseline_dist=pluecker_distance_to_origin(B);
	// If the baseline intersects the object, ECC is not well-defined. We return a half circle anyway.
	if (baseline_dist<=object_radius_mm)
		return std::make_pair(-0.5*Pi,0.5*Pi);
	// Else, find angle of plane which touches the sphere
	double kappa_max=std::abs(std::asin(object_radius_mm/baseline_dist));
	return std::make_pair(-kappa_max,kappa_max);
}

/// Compute epipolar lines for all angles given in kappas
std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > computeEpipolarLines(
	const ProjectionMatrix& P0, const ProjectionMatrix& P1,
	const std::vector<double>& kappas)
{
	// Geometry
	auto C0=getCameraCenter(P0);
	auto C1=getCameraCenter(P1);
	auto B =join_pluecker(C0,C1);

	// Projection of planes to lines in the image.
	auto P0invT=pseudoInverse(P0).transpose().eval();
	auto P1invT=pseudoInverse(P1).transpose().eval();
	// Figure out epipolar planes at 0 and 90 degrees w.r.t. the origin.
	RP3Plane  E0=join_pluecker(B,origin3);
	RP3Plane E90=join_pluecker(B,E0);
	// Convert to Hessian normal form
	E0/=E0.head(3).norm();
	E90/=E90.head(3).norm();

	// Loop over all kappas
	int n=(int)kappas.size();
	std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > corresponding_epipolar_lines(n);
#pragma omp parallel for
	for (int i=0;i<n;i++)
	{
		const double& kappa=kappas[i];
		// Epipolar plane at angle k to the origin and epipolar line on reference image.
		RP3Plane  E_kappa=cos(kappa)*E0+sin(kappa)*E90;
		// Projection from plane to line
		RP2Line  l0_kappa=P0invT*E_kappa;
		RP2Line  l1_kappa=P1invT*E_kappa;
		// Hessian normal form
		l0_kappa/=l0_kappa.head(2).norm();
		l1_kappa/=l1_kappa.head(2).norm();
		// Return pairs of corresponding epipolar lines
		corresponding_epipolar_lines[i]=std::make_pair(l0_kappa.cast<float>(),l1_kappa.cast<float>());
	}
	return corresponding_epipolar_lines;
}

/// A call-back function to handle GUI-input
void gui(const GetSetInternal::Node& node)
{
	// When the button has been clicked
	if (node.name=="Update")
	{
		// Load Images (and make sure they are single channel 2D)
		NRRD::Image<float> I0(GetSet<>("FBCC/Images/Image 0"));
		NRRD::Image<float> I1(GetSet<>("FBCC/Images/Image 1"));
		if (I0.dimension()!=2 || I1.dimension()!=2) {
			g_app.warn("Failed to Load Input Images", "Images must be uncompressed single-channel projections given 2D NRRD files.");
			return;
		}

		// Load Projection Matrices
		if (I0.meta_info.find("Projection Matrix")==I0.meta_info.end() || I1.meta_info.find("Projection Matrix")==I1.meta_info.end()) {
			g_app.warn("Failed to Load Input Images", "The \"Projection Matrix\" tag must be set in the NRRD header.");
			return;
		}
		auto P0=stringTo<ProjectionMatrix>(I0.meta_info["Projection Matrix"]);
		auto P1=stringTo<ProjectionMatrix>(I1.meta_info["Projection Matrix"]);

		// Visualize Projection Matrices
		double spacing=GetSet<double>("FBCC/Images/Pixel Spacing");
		Eigen::Vector4d image_rect(0,0,I0.size(0),I0.size(1));
		Figure("Geometry",800,600).overlay()
			.add(GraphicsItems::CoordinateAxes())
			.add(GraphicsItems::ConvexMesh::Cube())
			.add(GraphicsItems::ConvexMesh::Camera(P0,image_rect,spacing,true,QColor(255,0,0,32)))
			.add(GraphicsItems::ConvexMesh::Camera(P1,image_rect,spacing,true,QColor(0,255,0,32)));

		// Visualize Images
		Figure figure0("Image 0",I0);
		Figure figure1("Image 1",I1);
		
		// Figure out which epipolar planes are relevant
		auto C0=getCameraCenter(P0);
		auto C1=getCameraCenter(P1);
		auto B =join_pluecker(C0,C1);
		auto range_kappa=ecc_range_kappa_heuristic(B,GetSet<double>("FBCC/Sampling/Object Radius"));

		// Store epipolar plane angles in vector
		double dkappa=GetSet<double>("FBCC/Sampling/Angle Step (deg)")/180*Pi;
		int n_lines=(range_kappa.second-range_kappa.first)/dkappa;
		std::vector<double> kappas(n_lines);
#pragma omp parallel for
		for (int i=0;i<n_lines;i++)
			kappas[i]=range_kappa.first+dkappa*i;

		// Compute epipolar lines and download to GPU
		auto lines01=computeEpipolarLines(P0,P1,kappas);
		CudaUtils::MemoryBlock<float> l01s_d(n_lines*6,lines01.front().first.data());

		// Download image data to GPU
		CudaTexture I0_tex(I0.size(0),I0.size(1),I0);
		CudaTexture I1_tex(I1.size(0),I1.size(1),I1);

		// Allocate space for results
		CudaUtils::MemoryBlock<float> v0s_d(n_lines);
		CudaUtils::MemoryBlock<float> v1s_d(n_lines);

		// Compute integrals in image 0 and 1
		bool fbcc=GetSet<int>("FBCC/Sampling/Mode")==1;
		if (!fbcc)
		{
			// ECC
			cuda_computeLineIntegrals((float*)l01s_d  , 6, n_lines, 0x0, 0, I0_tex, I0.size(0), I0.size(1), v0s_d);
			cuda_computeLineIntegrals((float*)l01s_d+3, 6, n_lines, 0x0, 0, I1_tex, I1.size(0), I1.size(1), v1s_d);
		}
		else
		{
			// Virtual detector plane
			auto d =pluecker_direction(B);
			auto m =pluecker_moment(B);
			Eigen::Vector3d		U=d.normalized();
			Eigen::Vector3d		V=m.normalized();
			Geometry::RP3Plane	E(0,0,0,0);
			E.head(3)=U.cross(V);

			// Orthogonal proection to E
			Geometry::ProjectionMatrix P_E=Geometry::ProjectionMatrix::Zero();
			P_E.block<1,3>(0,0)=U.transpose();
			P_E.block<1,3>(1,0)=V.transpose();
			P_E(2,3)=spacing;
			// Rectifying homographies
			Geometry::RP2Homography H0=P_E*centralProjectionToPlane(C0,E)*pseudoInverse(P0);
			Geometry::RP2Homography H1=P_E*centralProjectionToPlane(C1,E)*pseudoInverse(P1);

			// Figure out how line coordinates transform under H0/1
			std::vector<FBCC_weighting_info> fbcc01(2*n_lines);
			for (int i=0;i<n_lines;i++) {
				// Corresponding epipolar lines.
				const auto& l0_kappa=lines01[i].first ;
				const auto& l1_kappa=lines01[i].second;
				// Epipoar plane
				RP3Plane E_kappa=P0.transpose()*l0_kappa.cast<double>();

				// Plane orthogonal to baseline through C0/1
				RP3Plane EB0(d(0),d(1),d(2),-d.dot(C0.head(3)));
				RP3Plane EB1(d(0),d(1),d(2),-d.dot(C1.head(3)));
				// Compute intersection of detector, epipolar plane and plane orthogonal to baseline through C0/1
				RP3Point Ak0=Geometry::meet_pluecker(Geometry::meet_pluecker(EB0,E_kappa),E);
				RP3Point Ak1=Geometry::meet_pluecker(Geometry::meet_pluecker(EB1,E_kappa),E);
				Geometry::dehomogenize(Ak0);
				Geometry::dehomogenize(Ak1);
				// Distance of C0/1 to 3D epipolar line in pixels
				float d_l_kappa_C0_px=(Ak0-C0).norm()/spacing;
				float d_l_kappa_C1_px=(Ak1-C1).norm()/spacing;
				// Projection of Ak to image 0/1
				RP2Point ak0=P0*Ak0;
				RP2Point ak1=P1*Ak1;
				Geometry::dehomogenize(ak0);
				Geometry::dehomogenize(ak1);

				// Transformation of line coordinates on l0_kappa and l1_kappa to virtual detector u-axis.
				LinePerspectivity phi0(H0, l0_kappa.cast<double>());
				LinePerspectivity phi1(H1, l1_kappa.cast<double>());

				// Line coordintas of ak0/1 in virtual detector
				float t_prime_ak0=phi0.transform(phi0.project_to_line(ak0,l0_kappa.cast<double>()));
				float t_prime_ak1=phi1.transform(phi1.project_to_line(ak1,l1_kappa.cast<double>()));

				// Store in float memory block for GPU download
				fbcc01[2*i+0].phi            =phi0;
				fbcc01[2*i+1].phi            =phi1;
				fbcc01[2*i+0].t_prime_ak     =t_prime_ak0;
				fbcc01[2*i+1].t_prime_ak     =t_prime_ak1;
				fbcc01[2*i+0].d_l_kappa_C_sq =d_l_kappa_C0_px*d_l_kappa_C0_px;
				fbcc01[2*i+1].d_l_kappa_C_sq =d_l_kappa_C1_px*d_l_kappa_C1_px;

			}

			// Download phis to GPU
			short n_fbcc=sizeof(FBCC_weighting_info)/sizeof(float);
			CudaUtils::MemoryBlock<float> fbcc01_d(2*n_lines*n_fbcc,(float*)fbcc01.data());
			
			// FBCC
			cuda_computeLineIntegrals((float*)l01s_d  , 6, n_lines, (float*)fbcc01_d        , n_fbcc*2, I0_tex, I0.size(0), I0.size(1), v0s_d);
			cuda_computeLineIntegrals((float*)l01s_d+3, 6, n_lines, (float*)fbcc01_d+n_fbcc, n_fbcc*2, I1_tex, I1.size(0), I1.size(1), v1s_d);

		}
		
		// Read back
		std::vector<float> v0s(n_lines*3);
		v0s_d.readback(v0s.data());
		std::vector<float> v1s(n_lines*3);
		v1s_d.readback(v1s.data());

		// Plot
		Plot plot("Epipolar Consistency",true);
		plot.setAxisLabels("Cosnsistency Metric [a.u.]","Epipolar Plane Angle")
			.setAxisAngularX()
			.showLegend();
		plot.graph().setData(n_lines,kappas.data(),v0s.data()).setName("Image 0").setColor(1,0,0);
		plot.graph().setData(n_lines,kappas.data(),v1s.data()).setName("Image 1").setColor(0,1,0);

		// Draw a couple of random epiolar lines
		for (int r=0;r<15;r++)
		{
			int i=std::rand()%n_lines;
			auto color=GraphicsItems::colorByIndex(r);
			figure0.drawLine(lines01[i].first .cast<double>(),color);
			figure1.drawLine(lines01[i].second.cast<double>(),color);
			plot.drawVerticalLine(kappas[i],color,1);
		}

	}
	// Write ini-File
	g_app.saveSettings();
}

/// Main: Show little window with settings.
int main(int argc, char ** argv)
{
	// Define default settings
	GetSetGui::File   ("FBCC/Images/Image 0"           ).setExtensions("2D NRRD image (*.nrrd);All Files (*)");
	GetSetGui::File   ("FBCC/Images/Image 1"           ).setExtensions("2D NRRD image (*.nrrd);All Files (*)");
	GetSet<double>    ("FBCC/Images/Pixel Spacing"     )=.308;
	GetSetGui::Section("FBCC/Images"                   ).setGrouped();
	GetSet<double>    ("FBCC/Sampling/Object Radius"   )=20;
	GetSet<double>    ("FBCC/Sampling/Angle Step (deg)")=0.01;
	GetSetGui::Enum   ("FBCC/Sampling/Mode"            ).setChoices("ECC;FBCC");
	GetSetGui::Section("FBCC/Sampling"                 ).setGrouped();
	GetSetGui::Button ("FBCC/Update"                   )="Update Plots";
	// Run application
	g_app.init(argc, argv, gui);
	return g_app.exec();
}

