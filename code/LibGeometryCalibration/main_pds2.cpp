
#include <GetSetGui/GetSetGui.h>
#include <GetSetGui/GetSetTabWidget.h>

#include <LibGeometryCalibration/PhantomPDS2.h>
#include <LibGeometryCalibration/BeadDetection.h>
#include <LibGeometryCalibration/EstimateProjectionMatrix.h>

#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>

#include <Utils/Projtable.hxx>

GetSetGui::Application g_app("CalibPDS2");

#include <qtimer.h>

// Input of real images
#include <LibEpipolarConsistency/Gui/PreProccess.h>

namespace Calibration {

	/// GUI fro Phantom generation, matching and projection matrix estimation.
	class PDS2CalibrationGui : public GetSetGui::Configurable, public GetSetGui::Object {
	public:

		PDS2CalibrationGui(const GetSetGui::Section& section, GetSetGui::ProgressInterface *app=0x0) : GetSetGui::Object(section,app) {}

	protected:
		PhantomDescriptorPDS2				descriptor_pds2;
		PhantomDetectionPDS2				detection_pds2 ;
		BeadDetectionTwoSize				bead_detection ;
		EstimateProjectionMatrix			calibration    ;
		EpipolarConsistency::PreProccess	input          ;

	public:

		double calibrate(NRRD::ImageView<float>& img, Geometry::ProjectionMatrix& P)
		{
			using UtilsQt::Figure;
			if (img.dimension()!=2) return -1;
			gui_retreive_section(gui_section());
			
//			Utils::TimerWin32 time;
			auto beads2d  =bead_detection.process(img);
//			err.time_detect=time.getElapsedTime();
			auto matching =detection_pds2.match(beads2d);
//			err.time_match=time.getElapsedTime();
			auto beads3d  =detection_pds2.beads;
			auto P_est    =calibration.estimateProjection(matching,beads2d,beads3d);
//			err.time_calib=time.getElapsedTime();
//			err.time_total=time.getTotalTime();

			// Residual re-projection error
			double reprojection_error=0;
			for (auto it=calibration.inlier_set.begin();it!=calibration.inlier_set.end();++it)
				reprojection_error+=(beads2d[it->first].head(2)-Geometry::euclidian2(P_est*Geometry::RP3Point(beads3d[it->second][0] ,beads3d[it->second][1] ,beads3d[it->second][2] ,1))).norm();
			reprojection_error/=calibration.inlier_set.size();
			// Number of point correspondences used for estimation
			int num_inliers=(int)calibration.inlier_set.size();
			
			// Visualization
			if (true)
			{
				img.meta_info["Projection Matrix"]=toString(P_est);
				Figure figure("Calibration",img);
				figure.setProjectionMatrix(P_est);
				double radius_threshold=BeadPhantomDetection::computeRadiusThresholdPx(beads2d);
				BeadDetection::overlayDetectedBeads(beads2d,figure.overlay().group("Detected Beads"),radius_threshold);
				figure.overlay().set("Initial Matching", GraphicsItems::Text2D(std::string("#inliers=")+toString(num_inliers)+", RPE="+toString(reprojection_error),20,20,QColor(255,255,255)));
				if (P_est.isZero()) figure.overlay().set("3D",GraphicsItems::Text2D("Calibration Failed.",20,40,QColor(255,255,255)));
				else addOverlay(figure.overlay().group("3D"),false);
			}

			P=P_est;
			return reprojection_error;
		}

		void update_beads()
		{
			descriptor_pds2.gui_retreive_section(gui_section().subsection("PDS2"));
			GetSet<std::vector<Eigen::Vector4d> >("PDS2/Location and Radius",gui_section())=detection_pds2.beads=descriptor_pds2.getBeads();
		}

		/// Add bounding box and bead overlay to existing figure. If fancy is set, will also display bounding box and bead indices.
		void addOverlay(GraphicsItems::Group& overlay, bool fancy=false)
		{
			const auto& beads=detection_pds2.beads;
			// Draw beads in 3D
			overlay.set("Cosy", GraphicsItems::CoordinateAxes(100));
			GraphicsItems::Group& bead_locations=overlay.group("Beads");
			GraphicsItems::Group& bead_indices  =overlay.group("Index");
			for (int i=0;i<(int)beads.size();i++) {
				Geometry::RP3Point X(beads[i]);
				double radius=X[3];
				X[3]=1;
				bead_locations.add(GraphicsItems::Point3D(X,radius+1,QColor(255,radius==beads[0][3]?128:0,0,255),fancy?GraphicsItems::MarkerShape::Dot:GraphicsItems::MarkerShape::Plus),"Bead");
			}
		}

		/// Declare types and default values for all properties.
		virtual void gui_declare_section (const GetSetGui::Section& section) {
			descriptor_pds2.gui_declare_section (section.subsection("PDS2"));
			detection_pds2 .gui_declare_section (section.subsection("PDS2"));
			bead_detection .gui_declare_section (section.subsection("Bead Detection"));
			calibration    .gui_declare_section (section.subsection("Calibration"));
			input          .gui_declare_section (section.subsection("Input"));
		}

		/// Retreive current values from GUI
		virtual void gui_retreive_section(const GetSetGui::Section& section) {
			descriptor_pds2.gui_retreive_section(section.subsection("PDS2"));
			detection_pds2 .gui_retreive_section(section.subsection("PDS2"));
			bead_detection .gui_retreive_section(section.subsection("Bead Detection"));
			calibration    .gui_retreive_section(section.subsection("Calibration"));
			input          .gui_retreive_section(section.subsection("Input"));
		}

		virtual void gui_init()
		{
			auto file=gui_section().subsection("Input/File");
			GetSetGui::File("Image Stack", file)
				.setExtensions("NRRD Image Stack (*.nrrd);;All Files (*)");
			GetSet<double>("Pixel Spacing [mm]", file, 0.308);
			GetSetGui::File("Output File", file)
				.setExtensions("One Matrix Per Line (*.ompl);;All Files (*)")
				.setCreateNew();
			file.setGrouped();
			GetSet<int>("Slice Index", file, 0);
			GetSetGui::Button("Calibration/Run")="Run Calibration...";
			GetSetGui::Object::gui_init();
			update_beads();
		}

		virtual void gui_notify(const std::string& relative_section, const GetSetInternal::Node& node)
		{
			using UtilsQt::Figure;

			if (node.name=="Location and Radius") {
				Figure figure("Bead Phantom",800,600);
				figure.z_vertical();
				addOverlay(figure.overlay(), true);
			}

			if (hasPrefix(node.super_section,"PDS2 Descriptor"))
				update_beads();

			if (hasPrefix(node.super_section,"Input"))
			{
				auto file=gui_section().subsection("Input/File");
				std::string file_path=GetSet<>("Image Stack", file);
				int slice_index=GetSet<int>("Slice Index", file);
				NRRD::Image<float> stack(file_path);
				NRRD::ImageView<float> slice(stack,slice_index);
				if (!slice)
					std::cerr << "File Access Eror:\n" << file_path << " slice " << toString(slice_index) << std::endl;
				else
				{
					input.gui_retreive_section(gui_section().subsection("Input"));
					input.process(slice);
					UtilsQt::Figure("Input Image", slice);
				}
			}

			if (node.name=="Run")
			{
				auto file=gui_section().subsection("Input/File");
				std::cout << "Calibrating...\n";
				Geometry::ProjectionMatrix P;
				auto figure=UtilsQt::Figure::Select(true);
				NRRD::ImageView<float> image=figure.getImage(); // FIXME multiple images.
				double residual=calibrate(image,P);
				// Save result
				std::string output_file=GetSet<>("Output File", file);
				if (!output_file.empty())
					image.save(output_file);
				// Visualize result
				Eigen::Vector4d image_rect(0,0,image.size(0),image.size(1));
				double spacing=GetSet<double>("Pixel Spacing [mm]", file);
				Figure("Bead Phantom").overlay().set("Cam",GraphicsItems::FancyCamera(P,image_rect,spacing));
				std::cout << "Projection Matrix P=" << std::endl << P << std::endl;
			}
		}
	};

} // namespace Calibration

void showletter()
{
	std::string letters="CAGEDF";

 	int i=rand()%letters.length();

	std::cout << letters[i] << std::endl;;
		
	QTimer::singleShot(5000, []() { showletter(); } );

}

void gui(const GetSetInternal::Node& node)
{

	if (node.name=="Bla")
	{
showletter();
	}

	g_app.saveSettings();
}

int main(int argc, char ** argv)
{
	GetSetGui::Button("Bead Detection/Bla")="Bla";

	g_app.init(argc,argv,gui);

	Calibration::PDS2CalibrationGui calibration("",&g_app);
	calibration.gui_init();

	Calibration::BeadDetectionGui bead_detection("Bead Detection",&g_app);
	bead_detection.gui_init();

	// Add all properties to command line.
	g_app.window().addDefaultFileMenu();
	g_app.window().addMenuItem("Edit","Close all figures...");
	g_app.window().addMenuItem("Edit","Show plot editor...");
	g_app.window().aboutText() =
		"<h3>Geometric Calibration for Flat-Panel Detector CT</h3>\n\n"
		"Copyright 2014-2017 by <a href=\"mailto:aaichert@gmail.com?Subject=[Calibration]\">Andre Aichert</a> <br>"
		"<h4>Usage:</h4>\n\n"
		"This tool computes a projection matrix from an X-ray image of the PDS2 phantom."
		"<br>"
		"<h4>1) Process input image</h4>"
		"Select 2D or stack of 2D NRRD images under \"Input/File/Input Stack\". If required, apply pre-processing "
		"using \"File/Input/Intensity/Apply Minus Log\". The metal beads should appear bright on dark background."
		"<h4>2) Bead Detection</h4>"
		"Adjust FRST Radii under \"Bead Detection/Detect Large Beads\" and \"Detect Small Beads\" respectively. "
		"You can plot line profiles by mouse drag in the \"Input Image\" window to get an idea of the size. "
		"Adjust the \"FRST Threshold\" based on the preview of the score shown in the \"FRST\" window."
		"<h4>3) Calibrate</h4>"
		"If enough beads are correctly deteted (please do test the Bead Detection) then just hit \"Calibration/Run\" button."
		"<br>"
		"<br>"
		"<br>"
		"See also: "
		"<br>"
		"<a href=\"https://www5.cs.fau.de/en/our-team/aichert-andre/projects/\">Pattern Recognition Lab at Friedrich-Alexander University of Erlangen-Nuremberg</a> "
		"<br>"
		"<h4>Licensed under the Apache License, Version 2.0 (the \"License\")</h4>\n\n"
		"You may not use this file except in compliance with the License. You may obtain a copy of the License at "
		"<a href=\"http://www.apache.org/licenses/LICENSE-2.0\">http://www.apache.org/licenses/LICENSE-2.0</a><br>"
		;

	// Attempt to load first image.
	GetSet<int>("Input/File/Slice Index")=0;

	return g_app.exec();
}

