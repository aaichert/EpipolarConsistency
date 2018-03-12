#include "InputDataDirect.h"

#define _HAS_UTILS_QT

#ifdef _HAS_UTILS_QT
#include "DisplayGeometry.hxx"
#include <GetSetGui/GetSetModalDialog.h>
#endif // _HAS_UTILS_QT

#include <LibUtilsCuda/CudaBindlessTexture.h>

namespace EpipolarConsistency {
	
	InputDataDirect::InputDataDirect()
		: image_source(0x0)
	{}

	InputDataDirect::~InputDataDirect()
	{
		if (image_source)
			delete image_source;
		image_source=0x0;
	}


	void InputDataDirect::gui_declare_section (const GetSetGui::Section& section)
	{
		GetSet<>                         ("Projection/Matrices"                           , section, projections.projection_matrices             ).setDescription("Text-file containing projection matrices. Format is 12 column-major values per line.");
		GetSet<std::vector<std::string> >("Projection/Images"                             , section, projections.image_files                     ).setDescription("");
		GetSet<>                         ("Advanced/Basename/Pre-processed Projections"   , section, advanced.basename_pre_processed_images      ).setDescription("");
		GetSet<bool>                     ("Advanced/Display Pre-processed Images"         , section, advanced.show_pre_processed_imgs            ).setDescription("");

		projections.pre_process.gui_declare_section(section.subsection("Projection/Pre-Processing"));

		// Looks
		section.subsection("Advanced/Basename").setGrouped();
		section.subsection("Projection"       ).setGrouped();
		GetSetGui::StaticText("Advanced/Basename/_info", section)="Provide a filename here to store intermedaiete results.\nA three-digit index will be appended.";
//		GetSetGui::StaticText("Projection/_info" , section)="Optionally provide ompl file to override image meta-info.";

		GetSetGui::File                  ("Advanced/Basename/Pre-processed Projections"   , section).setExtensions("2D NRRD Image (*.nrrd);;All Files (*)").setCreateNew(true);
		GetSetGui::File                  ("Projection/Matrices"                           , section).setExtensions("One Matrix Per Line (*.ompl);;Siemens Projtable (*.txt);;All Files (*)");
		GetSetGui::File                  ("Projection/Images"                             , section).setMultiple(true).setExtensions("Multiple 2D projections (*.nrrd);;Stack of projections (*.nrrd);;All Files (*)");
		GetSet<double>                   ("Advanced/Pixel Spacing"                        , section, advanced.mm_per_px);
		GetSet<int>                      ("Advanced/Skip Projections"                     , section, advanced.skip_projections).setDescription("For large data sets, one may reduce memory requirements by skipping some projections.");
	}
	
	void InputDataDirect::gui_retreive_section(const GetSetGui::Section& section)
	{
		projections.projection_matrices             =GetSet<std::string>              ("Projection/Matrices"                          , section);
		projections.image_files                     =GetSet<std::vector<std::string> >("Projection/Images"                            , section);
		advanced.mm_per_px                          =GetSet<double>                   ("Advanced/Pixel Spacing"                       , section);
		advanced.skip_projections                   =GetSet<int>                      ("Advanced/Skip Projections"                    , section);
		advanced.basename_pre_processed_images      =GetSet<std::string>              ("Advanced/Basename/Pre-processed Projections"  , section);
		projections.pre_process.gui_retreive_section(section.subsection("Projection/Pre-Processing"));
	}
			
	int InputDataDirect::prepareForLoadingPreprocessedImage() const
	{
		// Delete old image source
		if (image_source) delete image_source;
		// Create new one with up-to-date information
		image_source=new ImageSource(projections.image_files);
		int n=image_source->numImages();
		// If loading failed, return zero
		if (n<2)
		{
			delete image_source;
			image_source=0x0;
			return 0;
		}
		return n;
	}

	bool InputDataDirect::loadPreprocessedImage(NRRD::Image<float>& img, Geometry::ProjectionMatrix& P, int i)
	{
		// Open data file if required.
		int n=(image_source?n=image_source->numImages():prepareForLoadingPreprocessedImage());
		// Load ith image
		if (n==0 || !image_source->getImage(img,P,i)) return false;
		// Enforce Pixel spacing
		img.spacing(0)=img.spacing(1)=advanced.mm_per_px;
		// Pre-process image data and show preview image.
		projections.pre_process.process(img);
		projections.pre_process.apply_weight_cos_principal_ray(img,P);
		return true;
	}

	InputDataDirect::ImageSource::ImageSource(const std::vector<std::string>& _files)
		: files(_files)
		, stack(0x0)
	{
		if (files.size()==1)
		{
			std::map<std::string, std::string> meta;
			NRRD::parseHeader(files[0], meta);
			// Convert to float once in the beginning.
			if (meta["type"] != "float" || meta["endian"] == "big")
			{
				std::string  tmpfile = "float_image_stack.nrrd";
				NRRD::Image<float>(files[0]).save(tmpfile);
				files[0] = tmpfile;
			}
			// Access z-slices of input stack via NRRD::ImageStack
			stack = new NRRD::ImageStack<float>(files[0]);
			if (!stack->good())
			{
				delete stack;
				stack=0x0;
				files.clear();
				return;
			}

			Ps_stack=stringToVector<Geometry::ProjectionMatrix>(stack->meta_info["Projection Matrices"],'/');
		}
	}

	InputDataDirect::ImageSource::~ImageSource()
	{
		if (stack) delete stack;
		stack=0x0;
	}

	int InputDataDirect::ImageSource::numImages() const
	{
		return (files.size()==1?(stack?stack->size(2):0):(int)files.size());
	}

	bool InputDataDirect::ImageSource::getImage(NRRD::Image<float>& I, Geometry::ProjectionMatrix& P, int i) const
	{
		P.setZero();
		if (i<0 || i>=numImages())
			return false;
		if (stack) {
			if (!stack->readImage(i, I))
				return false;
			if (Ps_stack.size()>i)
				P=Ps_stack[i];
		}
		else
		{
			if (!I.load(files[i])) return false;
			if (I.meta_info.find("Projection Matrix")!=I.meta_info.end())
				P=stringTo<Geometry::ProjectionMatrix>(I.meta_info["Projection Matrix"]);
		}
		return I.dimension() == 2;
	}

	bool InputDataDirect::loadData(std::vector<Geometry::ProjectionMatrix>& Ps, std::vector<UtilsCuda::BindlessTexture2D<float>*>& Is, GetSetGui::ProgressInterface& progress, bool force_reload)
	{
		// Check if (any) valid data is available. If so, and not force_reload, we just skip out.
		bool reload=force_reload;
		reload|=Ps.empty() || Ps.size()!=Is.size();
		if (!reload) return true;
		Is.clear();
		Ps.clear();

		// Make sure we have a valid step
		if (advanced.skip_projections<=0) advanced.skip_projections=0;

		// Output file name for image and dtr
		std::string image_output_dir=advanced.basename_pre_processed_images;
		std::string image_output_ext=splitRight(image_output_dir,".");
		std::string image_output_name=splitRight(image_output_dir,"/\\");

		// Allow user to back out
		bool cancel_clicked=false;

		// Projection Matrices in Meta-Info will be used unless if these are not provided.
		auto Ps_file=loadProjectionMatrices(projections.projection_matrices);
		int n=prepareForLoadingPreprocessedImage();
		if (n<=0) {
			progress.warn(__FUNCTION__, "File Access Error.\n\nPlease provide multiple 2D images in NRRD format or a single 3D stack of 2D images in NRRD format");
			return false;
		}

		// Start actual processing
		progress.progressStart(__FUNCTION__,"Loading and Pre-processing Images...",n, &cancel_clicked);
		NRRD::Image<float> img;
		Geometry::ProjectionMatrix P;

		/// Load and pre-compute projection images and compute Radon transform
		// Compute Derivtaives of Radon transform
		std::vector<std::string> output_files;
		for (int i=0; i<n; i++)
		{
			//
			// Pre-Processing
			//
			if (!loadPreprocessedImage(img,P,i)) {
				progress.warn(__FUNCTION__, std::string("File Access Error. \n\n")
					+ (projections.image_files.size()==1?"Slice "+toString(i):"Image File \n"+projections.image_files[i]+"\n")
					+ " could not be read.");
				cancel_clicked=true;
				break;
			}
			Is.push_back(new UtilsCuda::BindlessTexture2D<float>(img.size(0),img.size(1),img));

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

			if (cancel_clicked) break;
			progress.progressUpdate(i);
		}

		// Clean up
		progress.progressEnd();
		if (image_source)
			delete image_source;
		image_source=0x0;
		cancel_clicked |= (Is.empty() || (Ps.size()!=Is.size()) );
		if (cancel_clicked)
		{
			for (auto it=Is.begin();it!=Is.end();++it) delete *it;
			Is.clear();
			output_files.clear();
			return false;
		}
		return true;
	}

	/// Load projection matrices
	std::vector<Geometry::ProjectionMatrix> InputDataDirect::loadProjectionMatrices(const std::string& projection_matrices)
	{
		std::vector<Geometry::ProjectionMatrix> Ps;
		if (!projection_matrices.empty())
		{
			std::string projections_file=projection_matrices;
			std::string projections_extension=splitRight(projections_file, ".");
			if (projections_extension == "txt")
				Ps = ProjTable::loadProjtable(projection_matrices);
			else
				Ps = ProjTable::loadProjectionsOneMatrixPerLine(projection_matrices);
		}
		return Ps;
	}
		
	double InputDataDirect::getPixelSpacing() const
	{
		return advanced.mm_per_px;
	}

	InputDataDirectGui::InputDataDirectGui(const GetSetGui::Section& section, GetSetGui::ProgressInterface* app)
		: GetSetGui::Object(section,app)
		, InputDataDirect()
		, stack(0x0)
	{
		gui_declare_section(section);
	}

	void InputDataDirectGui::gui_declare_section(const GetSetGui::Section& section)
	{
		gui_ignore_notify(true);
		GetSetGui::Button("(Re-)Load",section)="Load Input Data...";
		#ifdef _HAS_UTILS_QT
			GetSetGui::Button("Projection/Pre-Processing/Preview",section)="Preview...";
			GetSetGui::Button("Visualize",section)="Visualize Input Data...";
		#endif // _HAS_UTILS_QT
		InputDataDirect::gui_declare_section(section);
		gui_ignore_notify(false);
	}


	void InputDataDirectGui::gui_notify(const std::string& section, const GetSetInternal::Node& node)
	{
		// Discard image source, in case something changes.
		if (image_source) delete image_source;
		image_source=0x0;

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
			auto& Is=getProjectionImages();
			if (Ps.empty() || Is.empty()) return;
			double w=Is.front()->size[0];
			double h=Is.front()->size[1];
			double spacing=this->getPixelSpacing();
			displayGeometry(Ps,w,h,spacing);
		}
#endif // _HAS_UTILS_QT
	}

	bool InputDataDirectGui::loadData(bool force_reload) {
		gui_retreive_section(gui_section());
		return InputDataDirect::loadData(Ps,Is,*app,force_reload);
	}

	const std::vector<Geometry::ProjectionMatrix>&               InputDataDirectGui::getProjectionMatrices()         const { return Ps; }
	const std::vector<UtilsCuda::BindlessTexture2D<float>*>&     InputDataDirectGui::getProjectionImages()           const { return Is; }

} // namespace EpipolarConsistency
