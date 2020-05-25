#ifndef __bead_phantom_gui_h
#define __bead_phantom_gui_h
// The OpenSource version had everything pertaining to other phantoms, especiall cross ratio phantom removed.


#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>

// small is defined by Windows?!
#ifdef small
#undef small
#endif 

#include <LibGeometryCalibration/PhantomPDS2.h>
#include <LibGeometryCalibration/BeadDetection.h>
#include <LibGeometryCalibration/EstimateProjectionMatrix.h>

// Input of real images
#include <LibEpipolarConsistency/Gui/PreProccess.h>
using EpipolarConsistency::PreProccess;


namespace Calibration {

	/// Compute mean absolute target registration error
	inline double computeTargetRegistrationError(const std::vector<Geometry::RP3Point> beads, const Geometry::ProjectionMatrix& P0,const Geometry::ProjectionMatrix& P1){
		double tre=0;
		for (auto it=beads.begin();it!=beads.end();++it) {
			Geometry::RP2Point X0=P0**it;
			Geometry::RP2Point X1=P1**it;
			tre+=(Geometry::euclidian2(X0)-Geometry::euclidian2(X1)).norm();
		}
		tre/=beads.size();
		return tre;
	}

	/// GUI fro Phantom generation, matching and projection matrix estimation.
	class CalibrationTestGui : public GetSetGui::Configurable, public GetSetGui::Object {
	public:

		CalibrationTestGui(const GetSetGui::Section& section, GetSetGui::ProgressInterface *app=0x0) : GetSetGui::Object(section,app), active_phantom(PDS2) {}

	protected:
		enum ActivePhantom {
			PDS2
		} active_phantom;

		PhantomDescriptorPDS2    phantom_descriptor_pds2;
		PhantomDetectionPDS2     phantom_detection_pds2 ;
		BeadPhantomSimulator     phantom_simulator      ;
		BeadDetectionTwoSize     bead_detection         ;
		EstimateProjectionMatrix calibration            ;
	public:

		void reset() {
		phantom_simulator. load_volume(NRRD::ImageView<float>());
		}

		BeadPhantomDescriptor& getDescriptor() { gui_retreive_section(gui_section()); return (BeadPhantomDescriptor&)phantom_descriptor_pds2) ;}
		BeadPhantomDetection&  getDetector()   { gui_retreive_section(gui_section()); return (BeadPhantomDetection&) phantom_detection_pds2 ) ;}

		struct Stats {
			double target_projection_error  =0;
			double reprojection_error       =0;
			double source_pos_error         =0;
			double num_inliers              =0;
//			double time_detect              =0;
//			double time_match               =0;
//			double time_calib               =0;
//			double time_total               =0;
			Geometry::ProjectionMatrix		P;

			void print() {
				std::cout << "target_projection_error : " << target_projection_error << std::endl;
				std::cout << "reprojection_error      : " << reprojection_error      << std::endl;
				std::cout << "source_pos_error        : " << source_pos_error        << std::endl;
				std::cout << "num_inliers             : " << num_inliers             << std::endl;
//				std::cout << "time_detect             : " << time_detect             << std::endl;
//				std::cout << "time_match              : " << time_match              << std::endl;
//				std::cout << "time_calib              : " << time_calib              << std::endl;
//				std::cout << "time_total              : " << time_total              << std::endl;
				std::cout << "P                       :\n"<< P                       << std::endl;
			}
		};

		// Performs simulation and calibration for a projection matrix
		Stats validate(const Geometry::ProjectionMatrix& P_gt, NRRD::ImageView<float>& img, bool simulate_image=true)
		{
			Stats err;
			using UtilsQt::Figure;
			if (img.dimension()!=2) return err;
			gui_retreive_section(gui_section());

			Eigen::Vector2i image_size(img.size(0),img.size(1));
			NRRD::Image<float> drr(image_size[0],image_size[1]);
			if (simulate_image)
			{
				// Create voxel representation, if needed.
				if (!phantom_simulator.drr.voxel_data) {
					NRRD::Image<float> tmp;
					phantom_simulator.voxelize(tmp,getBeads(),Figure("Bead Phantom").progress());
					if (!!tmp)
					{
						std::cout << "saved BeadPhantom.nrrd\n";
						tmp.save("BeadPhantom.nrrd");
					}
				}
				// Actual calibration
				phantom_simulator.project(drr,P_gt);
			}
			else
				drr.copyDataFrom((float*)img);
			drr.save("last_simulation.nrrd");
//			Utils::TimerWin32 time;
			auto beads2d  =bead_detection.process(drr);
//			err.time_detect=time.getElapsedTime();
			auto matching =getDetector().match(beads2d);
//			err.time_match=time.getElapsedTime();
			auto beads3d  =getDetector().beads;
			auto P_est    =calibration.estimateProjection(matching,beads2d,beads3d);
//			err.time_calib=time.getElapsedTime();
//			err.time_total=time.getTotalTime();
			// Target projection error
			for (auto bead=beads3d.begin();bead!=beads3d.end();++bead)
				err.target_projection_error+=(Geometry::euclidian2(P_est**bead)-Geometry::euclidian2(P_gt**bead)).norm();
			err.target_projection_error/=beads3d.size();
			// Residual re-projection error
			for (auto it=calibration.inlier_set.begin();it!=calibration.inlier_set.end();++it)
				err.reprojection_error+=(beads2d[it->first].head(2)-Geometry::euclidian2(P_est*Geometry::RP3Point(beads3d[it->second][0] ,beads3d[it->second][1] ,beads3d[it->second][2] ,1))).norm();
			err.reprojection_error/=calibration.inlier_set.size();
			// Error in source position
			err.source_pos_error=(Geometry::euclidian3(Geometry::getCameraCenter(P_est))-Geometry::euclidian3(Geometry::getCameraCenter(P_gt))).norm();
			// Number of point correspondences used for estimation
			err.num_inliers=calibration.inlier_set.size();
			// Actual result
			err.P=P_est;

			// Visualization
			if (true)
			{
				Figure figure("Validation",drr);
				figure.setProjectionMatrix(P_est);
				double radius_threshold=BeadPhantomDetection::computeRadiusThresholdPx(beads2d);
				BeadDetection::overlayDetectedBeads(beads2d,figure.overlay().group("Detected Beads"),radius_threshold);
				figure.overlay().set("Initial Matching", GraphicsItems::Text2D(std::string("#inliers=")+toString(err.num_inliers)+", TPE="+toString(err.target_projection_error),20,20,QColor(255,255,255)));
				if (P_est.isZero()) figure.overlay().set("3D",GraphicsItems::Text2D("Calibration Failed.",20,40,QColor(255,255,255)));
				else            addOverlay(figure.overlay().group("3D"),false);
				
			}

			return err;
		}

		/// Access to beads
		const std::vector<Eigen::Vector4d>& getBeads()
		{
			return 
				phantom_detection_pds2.beads=
				GetSet<std::vector<Eigen::Vector4d> >("Matching/Location and Radius",gui_section());
		}

		/// Allows setting current beads for detection and display
		void setBeads(const std::vector<Eigen::Vector4d>& beads)
		{
			phantom_detection_pds2.beads=beads;
			phantom_simulator.load_volume(NRRD::ImageView<float>());
			GetSet<std::vector<Eigen::Vector4d> >("Matching/Location and Radius",gui_section())=beads;
		}

		/// Add bounding box and bead overlay to existing figure. If fancy is set, will also display bounding box and bead indices.
		void addOverlay(GraphicsItems::Group& overlay, bool fancy=false)
		{
			const auto& beads=getBeads();
			// Draw beads in 3D
			overlay.set("Cosy", GraphicsItems::CoordinateAxes(100));
			GraphicsItems::Group& bead_locations=overlay.group("Beads");
			GraphicsItems::Group& bead_indices  =overlay.group("Index");
			for (int i=0;i<(int)beads.size();i++) {
				Geometry::RP3Point X(beads[i]);
				double radius=X[3];
				X[3]=1;
				bead_locations.add(GraphicsItems::Point3D(X,radius+1,QColor(255,radius==beads[0][3]?128:0,0,255),fancy?GraphicsItems::MarkerShape::Dot:GraphicsItems::MarkerShape::Plus),"Bead");
				//if (fancy)
				//	bead_indices.add(GraphicsItems::Text3D(toString(i),X,QColor(255,radius==beads[0][3]?128:0,0,255)),"Bead");
			}
			// Draw bounding box
			auto model_vx2mm=phantom_simulator.getModel();
			auto& bounding_box=overlay.set("Bounding Box",GraphicsItems::ConvexMesh::Cube(Eigen::Vector3d(0,0,0),phantom_simulator.voxelization.voxel_number.cast<double>()));
			for (int i=0;i<(int)bounding_box.p_pos.size();i++)
				bounding_box.p_pos[i]=model_vx2mm*bounding_box.p_pos[i];
		}

		/// Declare types and default values for all properties.
		virtual void gui_declare_section (const GetSetGui::Section& section) {
			phantom_descriptor_pds2.gui_declare_section (section.subsection("Simulation/Descriptor PDS2"));
			phantom_detection_pds2 .gui_declare_section (section.subsection("Matching"));
			phantom_simulator      .gui_declare_section (section.subsection("Simulation"));
			bead_detection         .gui_declare_section (section.subsection("Bead Detection"));
			calibration            .gui_declare_section (section.subsection("Calibration"));
			GetSetGui::Enum("Matching/Algorithm",section,active_phantom).setChoices("PDS2");
		}

		/// Retreive current values from GUI
		virtual void gui_retreive_section(const GetSetGui::Section& section) {
			phantom_descriptor_pds2.gui_retreive_section(section.subsection("Simulation/Descriptor PDS2"));
			phantom_detection_pds2 .gui_retreive_section(section.subsection("Matching"));
			phantom_simulator      .gui_retreive_section(section.subsection("Simulation"));
			bead_detection         .gui_retreive_section(section.subsection("Bead Detection"));
			calibration            .gui_retreive_section(section.subsection("Calibration"));
			active_phantom=(ActivePhantom)GetSet<int>("Matching/Algorithm",section).getValue();
		}
		
		virtual void gui_init() {
			GetSetGui::Button("Simulation/Create Phantom",gui_section())="Create Phantom...";
			GetSetGui::Button("Matching/Test Matching"   ,gui_section())="Test Matching...";
			GetSetGui::Object::gui_init();
		}

		virtual void gui_notify(const std::string& relative_section, const GetSetInternal::Node& node)
		{
			using UtilsQt::Figure;
			if (node.name=="Create Phantom")
				setBeads(getDescriptor().getBeads());

			if (node.name=="Location and Radius") {
				Figure figure("Bead Phantom",800,600);
				figure.z_vertical();
				addOverlay(figure.overlay(), true);
			}

			if (node.name=="Test Matching")
			{
				// Get input image and parameters from GUI
				Figure figure=Figure::Select(true);
				if (!figure.exists()) return;
				figure.overlay().clear();
				gui_retreive_section(gui_section());

				figure.progress().progressStart(__FUNCTION__,"Bead detection and matching...",4,0x0);
				figure.progress().progressUpdate(1);

				// Bead detection
				auto detected_beads=bead_detection.process(figure.getImage());
				double radius_threshold=BeadPhantomDetection::computeRadiusThresholdPx(detected_beads);
				BeadDetection::overlayDetectedBeads(detected_beads,figure.overlay().group("Detected Beads"),radius_threshold);
				figure.progress().progressUpdate(2);

				// Matching
				auto matching=getDetector().match(detected_beads);
				if (matching.empty())
					figure.overlay().set("Initial Matching", GraphicsItems::Text2D("Matching Failed.",20,20,QColor(255,255,255)));
				else
				{
					auto& initial_matching=figure.overlay().group("Initial Matching");
					for (auto it=matching.begin();it!=matching.end();++it)
						initial_matching.add(GraphicsItems::Text2D(toString(it->second),detected_beads[it->first].head(2)+Eigen::Vector2d(10,10),QColor(255,255,255)),"Match");
				}
				figure.progress().progressUpdate(3);

				// Estimation of projection geomety
				auto P=calibration.estimateProjection(matching,detected_beads,getDetector().beads);
				figure.setProjectionMatrix(P);
				std::cout << "P=\n" << P << std::endl;
				if (P.isZero()) figure.overlay().set("3D",GraphicsItems::Text2D("Calibration Failed.",20,40,QColor(255,255,255)));
				else            addOverlay(figure.overlay().group("3D"),false);
				figure.progress().progressUpdate(4);

				// DEBUG /////////////////////
				auto& final_matching=figure.overlay().group("Matching Errors");
					for (auto it=calibration.inlier_set.begin();it!=calibration.inlier_set.end();++it)
					{
						auto initial=matching.find(it->first);
						bool missed=initial==matching.end();
						bool false_match= !missed && (initial->second!=it->second);
						std::string text = std::string("(")+toString(it->second)+")";
						if (missed||false_match && it->first < detected_beads.size() ) // FIXME size problem ?
							final_matching.add(GraphicsItems::Text2D(text,detected_beads[it->first].head(2)+Eigen::Vector2d(10,10),false_match?QColor(0,0,255):QColor(0,255,0)),"Mismatch");
					}

				

				figure.progress().progressEnd();
			}
		}
	};

} // namespace Calibration



#endif // __bead_phantom_gui_h
