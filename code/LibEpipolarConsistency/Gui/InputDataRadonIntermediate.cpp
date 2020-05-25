#include "InputDataRadonIntermediate.h"

#define _HAS_UTILS_QT

#ifdef _HAS_UTILS_QT
#include "DisplayGeometry.hxx"
#include <GetSetGui/GetSetModalDialog.h>
#include <EpipolarConsistencyRadonIntermediate.h>
#endif // _HAS_UTILS_QT

namespace EpipolarConsistency {
	
	void InputDataRadonIntermediate::gui_declare_section (const GetSetGui::Section& section)
	{
		InputDataDirect::gui_declare_section(section);

		GetSet<>                         ("Advanced/Basename/Radon Intermediate Files"      , section, advanced_dtr.basename_radon_intermediate          ).setDescription("If specified, Radon Intermediate Images are stored here.");
		GetSet<std::vector<std::string> >("Advanced/Pre-computed Radon Intermediate Files"  , section, advanced_dtr.precomputed_radon_intermediate_files ).setDescription("Loaded if non-empty and set if basename_radon_intermediate is non-empty.");
		GetSet<bool>                     ("Advanced/Display Radon Intermediate Functions"   , section, advanced_dtr.show_radon_intermediate              ).setDescription("Displays the Radon Intermediate Function after computation");

		radon_intermediate.gui_declare_section(section.subsection("Radon Intermediate").setGrouped().setDescription("Properties related to pre-computation of the Radon intermediate functions."));
		GetSetGui::File                  ("Advanced/Basename/Radon Intermediate Files"      , section).setExtensions("2D NRRD Image (*.nrrd);;All Files (*)").setCreateNew(true);
		GetSetGui::File                  ("Advanced/Pre-computed Radon Intermediate Files"  , section).setMultiple(true).setExtensions("2D NRRD Image (*.nrrd);;All Files (*)");
	}
	
	void InputDataRadonIntermediate::gui_retreive_section(const GetSetGui::Section& section)
	{
		InputDataDirect::gui_retreive_section(section);

		advanced_dtr.basename_radon_intermediate          =GetSet<std::string>              ("Advanced/Basename/Radon Intermediate Files"     , section);
		advanced_dtr.precomputed_radon_intermediate_files =GetSet<std::vector<std::string> >("Advanced/Pre-computed Radon Intermediate Files" , section);
		advanced_dtr.show_radon_intermediate              =GetSet<bool>                     ("Advanced/Display Radon Intermediate Functions"  , section);

		radon_intermediate.gui_retreive_section(section.subsection("Radon Intermediate"));
	}


	bool InputDataRadonIntermediate::loadData(std::vector<Geometry::ProjectionMatrix>& Ps, std::vector<EpipolarConsistency::RadonIntermediate*>& dtrs, GetSetGui::ProgressInterface& progress, bool force_reload)
	{
		// Check if (any) valid data is available. If so, and not force_reload, we just skip out.
		bool reload=force_reload;
		reload|=Ps.empty() || Ps.size()!=dtrs.size();
		if (!reload) return true;
		dtrs.clear();
		Ps.clear();

		// Make sure we have a valid step
		if (advanced.skip_projections<=0) advanced.skip_projections=0;

		// Output file name for image and dtr
		std::string image_output_dir=advanced.basename_pre_processed_images;
		std::string image_output_ext=splitRight(image_output_dir,".");
		std::string image_output_name=splitRight(image_output_dir,"/\\");
		std::string dtr_output_dir=advanced_dtr.basename_radon_intermediate;
		std::string dtr_output_ext=splitRight(dtr_output_dir,".");
		std::string dtr_output_name=splitRight(dtr_output_dir,"/\\");

		// Allow user to back out
		bool cancel_clicked=false;

		/// Load pre-computed dtrs and exit early
		if (!advanced_dtr.precomputed_radon_intermediate_files.empty())
		{
			int n=(int)advanced_dtr.precomputed_radon_intermediate_files.size();
			progress.progressStart(__FUNCTION__, "Loading pre-computed Radon Intermediate Functions from disk.", n, &cancel_clicked);
			for (int i=0;i<n;i++)
			{
				std::map<std::string, std::string> fields,keys;
				if (NRRD::parseHeader(advanced_dtr.precomputed_radon_intermediate_files[i],fields,&keys)<10)
					cancel_clicked=true;
				dtrs.push_back(new RadonIntermediate(advanced_dtr.precomputed_radon_intermediate_files[i]));
				if (!dtrs.back()->data()) {
					cancel_clicked=true; // actually: loading failed.
					progress.progressEnd();
					progress.warn("File Access Error", advanced_dtr.precomputed_radon_intermediate_files[i] + " (dtr file "+toString(i)+")");
					break;
				}
				dtrs.back()->getTexture(); // download to GPU
				if (Ps.size()==i && keys.find("Original Image/Projection Matrix")!=keys.end())
					Ps.push_back(stringTo<Geometry::ProjectionMatrix>(keys["Original Image/Projection Matrix"]));
				progress.progressUpdate(i);
				if (cancel_clicked)  break;
			}
			if (Ps.size()!=dtrs.size())
				Ps.clear();
			progress.progressEnd();
			if (cancel_clicked) {
				for (auto it=dtrs.begin();it!=dtrs.end();++it) delete *it;
				dtrs.clear();
				Ps.clear();
				return false;
			}
			return true;
		}

		// Projection Matrices in Meta-Info will be used unless if these are not provided.
		auto Ps_file=loadProjectionMatrices(projections.projection_matrices);
		int n=prepareForLoadingPreprocessedImage();
		if (n<=0) {
			progress.warn(__FUNCTION__, "File Access Error.\n\nPlease provide multiple 2D images in NRRD format or a single 3D stack of 2D images in NRRD format");
			return false;
		}

		// Start actual processing
		progress.progressStart(__FUNCTION__,"Computing Radon Derivatives on GPU...",n, &cancel_clicked);
		NRRD::Image<float> img;
		
		/// Load and pre-compute projection images and compute Radon transform
		// Compute Derivtaives of Radon transform
		std::vector<std::string> output_files;
		for (int i=0; i<n; i+=advanced.skip_projections+1)
		{
				
			//
			// Pre-Processing
			//

			// Load image and projection matrix
			Geometry::ProjectionMatrix P=Geometry::ProjectionMatrix::Zero();
			if (!loadPreprocessedImage(img,P,i)) {
				progress.warn(__FUNCTION__, std::string("File Access Error. \n\n")
					+ (projections.image_files.size()==1?"Slice "+toString(i):"Image File \n"+projections.image_files[i]+"\n")
					+ " could not be read.");
				cancel_clicked=true;
				break;
			}

			// Check for projection matrix
			if (Ps_file.size()>i) P=Ps_file[i];
			if (P.isZero())
			{
				progress.warn(__FUNCTION__, std::string("File Access Error. \n\n")
					+ "Projection Matrix " + toString(i) + " could not be read.");
				cancel_clicked=true;
				break;
			}
			Ps.push_back(P);

			// Save and display pre-processed image
			std::string image_file_out = image_output_name.empty()?"":image_output_dir + "/" + image_output_name + toString(i,3) + "." + image_output_ext;
			if (!image_file_out.empty()) img.save(image_file_out);
			if (advanced.show_pre_processed_imgs)
				UtilsQt::Figure("Preprocessed Image",img).drawText(toString(i+1)+" / "+toString(n));

			//
			// Radon Intermediate and Filter
			//

			// Compute Radon Intermediate
			EpipolarConsistency::RadonIntermediate *dtr=radon_intermediate.compute(img,&P,&advanced.mm_per_px);
			dtrs.push_back(dtr);

			// Readback and write output file
			std::string dtr_file_out = dtr_output_name.empty()  ?"":dtr_output_dir   + "/" + dtr_output_name   + toString(i,3) + "." + dtr_output_ext;
			if (advanced_dtr.show_radon_intermediate || !dtr_file_out.empty())
			{
				dtr->readback();
				if (!dtr_file_out.empty())
				{
					dtr->data().meta_info.insert(img.meta_info.begin(), img.meta_info.end());
					if (!dtr->data().save(dtr_file_out))
					{
						progress.warn(__FUNCTION__, std::string("File Access Error. \n\n") + dtr_file_out + " could not be written.");
						cancel_clicked=true;
						break;
					}
				}

				// Display DTR
				if (advanced_dtr.show_radon_intermediate)
					UtilsQt::Figure("Radon Intermediate",dtr->data(),0,0,radon_intermediate.filter==0)
						.drawLine(0,dtr->data().size(1)*0.5,dtr->data().size(0),dtr->data().size(1)*0.5,QColor(0,255,0))
						.drawText("alpha",20,dtr->data().size(1)*0.5+20,QColor(0,255,0))
						.drawLine(dtr->data().size(0)*0.5,0,dtr->data().size(0)*0.5,dtr->data().size(1),QColor(255,0,0))
						.drawText("t",dtr->data().size(1)*0.5+20,20,QColor(255,0,0));

				// Store output filenames
				if (!dtr_file_out.empty())
					output_files.push_back(dtr_file_out);
			}

			if (cancel_clicked) break;
			progress.progressUpdate(i);
		}

		// Clean up
		progress.progressEnd();
		if (image_source)
			delete image_source;
		image_source=0x0;
		cancel_clicked |= (dtrs.empty() || (Ps.size()!=dtrs.size()) );

		// Finish up and optionally list output files in ini file.
		if (cancel_clicked)
		{
			for (auto it=dtrs.begin();it!=dtrs.end();++it) delete *it;
			dtrs.clear();
			output_files.clear();
			return false;
		}
		advanced_dtr.precomputed_radon_intermediate_files = output_files;
		return true;
	}
	
	InputDataRadonIntermediateGui::InputDataRadonIntermediateGui(const GetSetGui::Section& section, GetSetGui::ProgressInterface* app)
		: GetSetGui::Object(section,app)
		, InputDataRadonIntermediate()
	{
		gui_declare_section(section);
	}

	void InputDataRadonIntermediateGui::gui_declare_section(const GetSetGui::Section& section)
	{
		gui_ignore_notify(true);
		GetSetGui::Button("(Re-)Load",section)="Load Input Data...";
		GetSetGui::Button("Projection/Pre-Processing/Preview",section)="Preview...";
		#ifdef _HAS_UTILS_QT
			GetSetGui::Button("Advanced/Visualize",section)="Visualize Input Data...";
		#endif // _HAS_UTILS_QT
		InputDataRadonIntermediate::gui_declare_section(section);
		gui_ignore_notify(false);
	}

	void InputDataRadonIntermediateGui::gui_notify(const std::string& section, const GetSetInternal::Node& node)
	{
		// Reload data
		if (node.name=="(Re-)Load")
			loadData(true);
		// Show the first image, pre-processed, and the projections matrices
#ifdef _HAS_UTILS_QT
		using UtilsQt::Figure;
		if (node.name=="Preview")
		{
			gui_retreive_section(gui_section());
			NRRD::Image<float> img;
			auto Ps=loadProjectionMatrices(projections.projection_matrices);
			Geometry::ProjectionMatrix P=(Ps.size()>0?Ps[0]:Geometry::ProjectionMatrix::Zero());
			if (!loadPreprocessedImage(img,P,0))
				return;
			// Visualize Geometry
			Geometry::ProjectionMatrix P_figure=Geometry::cameraLookAt(Geometry::cameraPerspective(20.0/180.0*Geometry::Pi,800,600),Eigen::Vector3d(0,0,1500),Eigen::Vector3d::Zero());
			Figure figure("Projection Geometry",800,600);
			figure.setProjectionMatrix(P_figure);
			figure.draw(GraphicsItems::CoordinateAxes());
			figure.draw(GraphicsItems::ConvexMesh::Cube(Eigen::Vector3d(-50,-50,-50),Eigen::Vector3d(50,50,50)));
			Eigen::Vector4d image_rect(0,0,img.size(0),img.size(1));
			figure.draw(GraphicsItems::ConvexMesh::Camera(P,image_rect,advanced.mm_per_px,true));
			figure.draw(GraphicsItems::Text3D("C0",Geometry::getCameraCenter(P)));

			// Finally, draw image.
			Figure("Preprocessed Image",img).drawText(toString(1));
		}
		// Visualize Geometry and Radon intermediate data
		if (node.name=="Visualize")
		{
			gui_retreive_section(gui_section());
			if (!loadData(false)) return;
			auto& Ps=getProjectionMatrices();
			auto& dtrs=getRadonIntermediateFunctions();
			GetSetGui::GetSetModalDialog diag;
			int n=(int)Ps.size();
			GetSetGui::RangedInt("Epipolar Consistency/Index Reference"   ,diag).setMax(n-1)=0;
			GetSetGui::RangedInt("Epipolar Consistency/Index Input"       ,diag).setMax(n-1)=1;
			GetSet<double>      ("Epipolar Consistency/Plane Angle [deg]" ,diag)=0.05;
			GetSetGui::Section  ("Epipolar Consistency",diag).setGrouped();
			GetSet<bool>        ("Visualization/Show Stack of Radon Intermediates",diag)=0;
			GetSetGui::Enum     ("Visualization/Show Cost Image",diag).setChoices("None;SSD;Correlation");
			GetSetGui::Section  ("Visualization",diag).setGrouped();
			if (diag.exec("Advanced Visualization"))
			{
				EpipolarConsistency::displayRadonSamples(
					GetSet<int>   ("Epipolar Consistency/Index Reference"   ,diag),
					GetSet<int>   ("Epipolar Consistency/Index Input"       ,diag),
					GetSet<double>("Epipolar Consistency/Plane Angle [deg]" ,diag)/180*Geometry::Pi,
					Ps,dtrs,app);
			}
			if (GetSet<bool>  ("Visualization/Show Stack of Radon Intermediates",diag))
				EpipolarConsistency::displayInputDataRadonIntermediate(*this);
	
			int mode=GetSet<int>("Visualization/Show Cost Image",diag);
			if (mode!=0)
			{
				NRRD::Image<float> cost_image(n,n);
				int l=cost_image.length();
				for (int i=0;i<l;i++) cost_image[i]=0;
				EpipolarConsistency::MetricRadonIntermediate ecc(Ps,dtrs);
				if (mode==2) ecc.useCorrelation();
				double inconsistency=ecc
					// .setdKappa(GetSet<double>("Epipolar Consistency/Plane Angle [deg]" ,diag)/180*Geometry::Pi)
					.evaluate(cost_image);
				cost_image.meta_info["Epipolar Inconsistency"]=toString(inconsistency);
				UtilsQt::Figure("Cost Image", cost_image);
			}
		}
#endif // _HAS_UTILS_QT
	}

	bool InputDataRadonIntermediateGui::loadData(bool force_reload) {
		gui_retreive_section(gui_section());
		return InputDataRadonIntermediate::loadData(Ps,dtrs,*app,force_reload);
	}

	const std::vector<Geometry::ProjectionMatrix>&               InputDataRadonIntermediateGui::getProjectionMatrices()         const { return Ps; }
	const std::vector<EpipolarConsistency::RadonIntermediate*>&  InputDataRadonIntermediateGui::getRadonIntermediateFunctions() const { return dtrs; }

} // namespace EpipolarConsistency
