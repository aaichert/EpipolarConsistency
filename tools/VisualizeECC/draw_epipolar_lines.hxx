// Utilities for Displaying Images and Plots
#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>
using UtilsQt::Figure;
using UtilsQt::Plot;

#include <LibEpipolarConsistency/EpipolarConsistencyCommon.hxx>

/// Function called when user clicks into projection images. Epipolar lines are drawn in 2 and 3 dimensions.
void updateEpipolarLines(const std::string& figure_name, bool is_blue, std::vector<Eigen::Vector4d>& selections)
{
	// Number of selections
	int n=(int)selections.size();
	if (is_blue || n==0 || n>10) {
		selections.clear();
		return;
	}
	// Do not allow ractangular selections
	if (!selections.back().tail(2).isConstant(1)) {
		selections.pop_back();
		return;
	}
	// Figure out which image has been clicked on with what mouse button
	Figure figure0(figure_name);
	Figure figure1((figure_name=="Image 0")?"Image 1":"Image 0");
	figure1.clearSelection(is_blue);

	// Figure out epipolar geometry
	using namespace Geometry;
	ProjectionMatrix P0=figure0.getProjectionMatrix();
	ProjectionMatrix P1=figure1.getProjectionMatrix();
	// Figure out projection to epipolar line 1 and plane
	auto       F=computeFundamentalMatrix(P0,P1);
	auto  P0invT=pseudoInverse(P0).transpose().eval();
	RP3Line    B=join_pluecker(getCameraCenter(P0),getCameraCenter(P1));
	// Figure out epipolar planes at 0 and 90 degrees w.r.t. the origin.
	RP3Plane  E0=join_pluecker(B,origin3);
	RP3Plane E90=join_pluecker(B,E0);
	// Convert to Hessian normal form
	E0/=E0.head(3).norm();
	E90/=E90.head(3).norm();
	// Figure out image planes
	double spacing=GetSet<double>("Epipolar Consistency/Images/Pixel Spacing");
	auto I0=getCameraImagePlane(P0,spacing);
	auto I1=getCameraImagePlane(P1,spacing);
	// Epipoles in 3D
	auto Ep1=meet_pluecker(B,I0);
	auto Ep0=meet_pluecker(B,I1);
	// Intersection line of image planes
	auto I=meet_pluecker(I0,I1);
	// Remove all overlays from figures, except for 3D geometry
	GraphicsItems::Group geom3d=figure0.overlay().group("3D");
	figure0.overlay().clear().set("3D",geom3d);
	figure1.overlay().clear().set("3D",geom3d);
	// Prepare 2D plot
	Plot plot("Epipolar Consistency");
	plot.clearItems();
	// Prepare info for 3D plot
	auto& line3d=Figure("Geometry").overlay().group("3D Lines");
	line3d.clear();
	line3d.add(GraphicsItems::PlueckerLine3D(B,2,QColor(0,0,0)));
	line3d.add(GraphicsItems::PlueckerLine3D(I,-1 ,QColor(0,0,0,128)));
	double w2=figure0.getImage().size(0)*0.5;
	double h2=figure0.getImage().size(1)*0.5;

	// Should samples in Radon intermediate functions be displayed?
	Figure fig0("Radon Intermediate Function 0");
	Figure fig1("Radon Intermediate Function 1");
	bool show_dtr=fig0.exists(true) & fig0.exists(true);
	double range_t=0;
	if (show_dtr)
	{
		fig0.overlay().group("Highlighted Lines").clear();
		fig1.overlay().group("Highlighted Lines").clear();
		// Compute range of t-value in Radon transform this dtr corresponds to
		double bin_size_distance=stringTo<double>(fig0.getImage().meta_info["Bin Size/Distance"]);
		range_t=bin_size_distance*fig0.getImage().size(1);
	}
	Geometry::RP2Homography line_origin_to_center=
		Geometry::Translation(-0.5*figure0.getImage().size(0),-0.5*figure0.getImage().size(1)) // shift by half image size
		.inverse().transpose(); // inverse transpose because we are transforming lines (covariantly)


	//std::cout << "Current line pairs:\n";
	for (int i=0;i<n;i++)
	{
		RP2Point x0(selections[i][0],selections[i][1],1);
		RP2Line  l1=F*x0;
		RP3Plane  E=P1.transpose()*l1;
		RP2Line  l0=P0invT*E;
		l0/=l0.head(2).norm();
		l1/=l1.head(2).norm();
		//std::cout << toString(i,3,' ') << ": l0 = " << l0[0] << " " << l0[1] << " " << l0[2] << " (alpha=" << std::atan2(-l0[0], -l0[1])+Pi*0.5 << "t=" << -l0[2]-w2*l0[0]-h2*l0[1]<< ")" <<  std::endl;
		//std::cout << toString(i,3,' ') << ": l1 = " << l1[0] << " " << l1[1] << " " << l1[2] << " (alpha=" << std::atan2(-l1[0], -l1[1])+Pi*0.5 << "t=" << -l1[2]-w2*l1[0]-h2*l1[1]<< ")" <<  std::endl;
		E/=E.head(3).norm();
		double kappa=plane_angle_in_pencil(E,E0,E90);
		if (kappa>+Pi/2) kappa-=Pi;
		if (kappa<-Pi/2) kappa+=Pi;
		auto color=GraphicsItems::colorByIndex(i+2);
		plot.drawVerticalLine(kappa,color,1);
		figure0.overlay().add(GraphicsItems::PlueckerLine2D(l0,1,color));
		figure1.overlay().add(GraphicsItems::PlueckerLine2D(l1,1,color));
		auto corner=meet_pluecker(I,E);
		line3d.add(GraphicsItems::Line3D(Ep0,corner,1,color));
		line3d.add(GraphicsItems::Line3D(Ep1,corner,1,color));
		if (show_dtr)
		{
			// Move origin of line to image center
			l0=line_origin_to_center*l0;
			l1=line_origin_to_center*l1;
			// Convert line to angle distance and compute texture coordinates to be sampled.
			lineToSampleDtr(l0,range_t);
			lineToSampleDtr(l1,range_t);
			fig0.overlay().group("Highlighted Lines").add(GraphicsItems::Point2D(
				l0[0]*fig0.getImage().size(0),
				l0[1]*fig0.getImage().size(1),
				5,color,GraphicsItems::Dot));
			fig1.overlay().group("Highlighted Lines").add(GraphicsItems::Point2D(
				l1[0]*fig1.getImage().size(0),
				l1[1]*fig1.getImage().size(1),
				5,color,GraphicsItems::Dot));
		}
	}
}


void showImages(const NRRD::Image<float>& I0, const NRRD::Image<float>& I1, Geometry::ProjectionMatrix P0, Geometry::ProjectionMatrix P1, double spacing)
{
	// Visualize Projection Matrices
	Eigen::Vector4d image_rect(0,0,I0.size(0),I0.size(1));
	GraphicsItems::Group static_3d_geometry;
	auto cube=GraphicsItems::ConvexMesh::Cube();
	static_3d_geometry
		.add(GraphicsItems::CoordinateAxes())
		.add(cube);
	cube.q_color.front()=QColor(0,0,0,64);
	cube.l_color=QColor(0,0,0,255);
	// Project geometry to 3D image planes
	auto C0=Geometry::getCameraCenter(P0);
	auto C1=Geometry::getCameraCenter(P1);
	auto proj_to_plane0=Geometry::centralProjectionToPlane(C0,Geometry::SourceDetectorGeometry(P0,spacing).image_plane);
	auto proj_to_plane1=Geometry::centralProjectionToPlane(C1,Geometry::SourceDetectorGeometry(P1,spacing).image_plane);
	GraphicsItems::Group image_plane_0(proj_to_plane0);
	GraphicsItems::Group image_plane_1(proj_to_plane1);
	image_plane_0.add(cube);
	image_plane_1.add(cube);
	Figure("Geometry",800,600).overlay()
		.add(static_3d_geometry)
		.add(image_plane_0)
		.add(image_plane_1)
		.add(GraphicsItems::ConvexMesh::Camera(P0,image_rect,spacing,true,QColor(255,0,0,32)))
		.add(GraphicsItems::ConvexMesh::Camera(P1,image_rect,spacing,true,QColor(0,255,0,32)));

	// Visualize Images
	Figure figure0("Image 0",I0);
	Figure figure1("Image 1",I1);
	figure0.setCallback(updateEpipolarLines);
	figure1.setCallback(updateEpipolarLines);
	figure0.showTiled(0,800,600,false).setProjectionMatrix(P0).overlay().group("3D").add(static_3d_geometry);
	figure1.showTiled(1,800,600,false).setProjectionMatrix(P1).overlay().group("3D").add(static_3d_geometry);

	// Show some standard lines
	std::vector<Eigen::Vector4d> selections;
	selections.push_back(Eigen::Vector4d(I0.size(0)*0.5,I0.size(1)*0.1,1,1));
	selections.push_back(Eigen::Vector4d(I0.size(0)*0.5,I0.size(1)*0.3,1,1));
	selections.push_back(Eigen::Vector4d(I0.size(0)*0.5,I0.size(1)*0.5,1,1));
	selections.push_back(Eigen::Vector4d(I0.size(0)*0.5,I0.size(1)*0.7,1,1));
	selections.push_back(Eigen::Vector4d(I0.size(0)*0.5,I0.size(1)*0.9,1,1));
	updateEpipolarLines("Image 0", false, selections);

}
