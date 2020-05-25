#include "BeadDetection.h"

#include <NRRD/nrrd_frst.hxx>
#include <NRRD/nrrd_connected_components.hxx>

#include <LibUtilsQt/Figure.hxx>

namespace Calibration {

	const std::vector<Geometry::RP2Point>& BeadDetection::process(const NRRD::ImageView<float>& image)
	{

		beads.clear();
		// Compute fast radial symmetry transform
		frst.clone(image);
		NRRD::frst(frst,paramsFRST.beadRadiiPx,paramsFRST.gaussian_sigma,paramsFRST.gaussian_width,1,paramsFRST.threshold_gradient);

		// Subtract threshold and clamp negative values to zero.
		int w=frst.size(0), h=frst.size(1), l=w*h;
#pragma omp parallel for 
		for (int i=0;i<l;i++) frst[i]=frst[i]<paramsFRST.threshold_frst?0:(float)(frst[i]-paramsFRST.threshold_frst);
		// Compute connected components
		NRRD::connected_components_2D<float>(frst, 0, true, components);
		// Find exact bead location (via score of frst)
		std::map<int,Geometry::RP2Point> ccs;
		for (int y=0;y<h;y++)
			for (int x=0;x<w;x++)
			{
				const float& score=frst.pixel(x,y);
				if (score<=0) continue;
				const unsigned short& label=components.pixel(x,y);
				auto it=ccs.find(label);
				if (it!=ccs.end())
					it->second+=Geometry::RP2Point(x*score,y*score,score);
				else
					ccs[label]=Geometry::RP2Point(x*score,y*score,score);
			}
		// Write output
		beads.resize(ccs.size());
		auto it=ccs.begin();
		for (int i=0;it!=ccs.end();i++,++it) {
			beads[i]   =Geometry::dehomogenized(it->second);
			beads[i][2]=paramsFRST.beadRadiiPx.back();

		}
		return beads;
	}

	void BeadDetection::overlayDetectedBeads(const std::vector<Geometry::RP2Point>& result, GraphicsItems::Group& overlay, double last_component_threshold)
	{
		for (auto it=result.begin();it!=result.end();++it) {
			overlay.add(GraphicsItems::Point2D(
				(*it)[0],(*it)[1],(*it)[2],
				((*it)[2]>last_component_threshold? QColor(255,0,0,255) : QColor(255,128,0,255)),
				GraphicsItems::MarkerShape::Circle)
				,"Bead");
		}
	}
	void BeadDetection::gui_declare_section (const GetSetGui::Section &section)
	{
		GetSet<std::vector<double> >("FRST Radii"                 ,section, paramsFRST.beadRadiiPx       ).setDescription("Semicolon seperated list of radii to examine.");
		GetSet<double              >("Gaussian Sigma"             ,section, paramsFRST.gaussian_sigma    ).setDescription("Gaussian convolution of FRST score.");
		GetSet<int                 >("Gaussian Half Kernel Width" ,section, paramsFRST.gaussian_width    ).setDescription("Half kernel width for Gaussian convolution of FRST score.");
		GetSet<double              >("FRST Threshold"             ,section, paramsFRST.threshold_frst    ).setDescription("A lower threshold for the detection of a circle by the FSRT.");
		GetSet<double              >("Min Gradient Magnitude"     ,section, paramsFRST.threshold_gradient).setDescription("A lower threshold for the gradient magnitude for a pixel to be included in the computation. Saves time on homogeneous regions.");
		
	} 

	void BeadDetection::gui_retreive_section(const GetSetGui::Section &section)
	{
		paramsFRST.beadRadiiPx        =GetSet<std::vector<double> >("FRST Radii"                 ,section);
		paramsFRST.gaussian_sigma     =GetSet<double              >("Gaussian Sigma"             ,section);
		paramsFRST.gaussian_width     =GetSet<int                 >("Gaussian Half Kernel Width" ,section);
		paramsFRST.threshold_frst     =GetSet<double              >("FRST Threshold"             ,section);
		paramsFRST.threshold_gradient =GetSet<double              >("Min Gradient Magnitude"     ,section);
	}

	const std::vector<Geometry::RP2Point>& BeadDetectionTwoSize::process(const NRRD::ImageView<float>& image)
	{
		beads.clear();
		large.process(image);
		small.process(image);

		// list of indices for large and small beads which were likely detected twice
		std::set<int> conflicts_large, conflicts_small;
		conflicts_large.clear();
		conflicts_small.clear();
		for (int l=0;l<large.beads.size();l++)
			for (int s=0;s<small.beads.size();s++)
			{
				// Compute distance
				double d=(large.beads[l].head(2)-small.beads[s].head(2)).norm();
				if (d<min_distance_px) {
					conflicts_large.insert(l);
					conflicts_small.insert(s);
				}
			}

		// Result contains all beads which are not in conflict or have their conflict resolved.
		bool prefer_beads=1; // 1: Prefers large beads
		for (int l=0;l<large.beads.size();l++)
			if (prefer_beads!=0 || conflicts_large.find(l)==conflicts_large.end())
				beads.push_back(large.beads[l]);
		for (int s=0;s<small.beads.size();s++)
			if (prefer_beads!=1 || conflicts_small.find(s)==conflicts_small.end())
				beads.push_back(small.beads[s]);

		return beads;
	}
	

	void BeadDetectionTwoSize::gui_declare_section (const GetSetGui::Section &section)
	{
		large.gui_declare_section(GetSetGui::Section("Detect Large Beads",section));
		small.gui_declare_section(GetSetGui::Section("Detect Small Beads",section));
		GetSet<double> ("Minimal Distance between Detections",section,min_distance_px);
	}

	void BeadDetectionTwoSize::gui_retreive_section(const GetSetGui::Section &section)
	{
		large.gui_retreive_section(GetSetGui::Section("Detect Large Beads",section));
		small.gui_retreive_section(GetSetGui::Section("Detect Small Beads",section));
		min_distance_px=GetSet<double>("Minimal Distance between Detections",section);
	}


	BeadDetectionGui::BeadDetectionGui(const GetSetGui::Section& section, GetSetGui::ProgressInterface *app) : GetSetGui::Object(section,app) {}

	void BeadDetectionGui::gui_init() {
		GetSetGui::Button("Test"                    ,gui_section())="Detect Beads...";
		GetSetGui::Button("Detect Large Beads/Test" ,gui_section())="Detect Large Beads...";
		GetSetGui::Button("Detect Small Beads/Test" ,gui_section())="Detect Small Beads...";
		GetSetGui::Object::gui_init();
	}

	void BeadDetectionGui::gui_notify(const std::string& relative_section, const GetSetInternal::Node& node)
	{
		if (node.name=="Test")
		{
			// Get input image and parameters from GUI
			using UtilsQt::Figure;
			auto figure=Figure::Select(true);
			if (!figure.exists()) {
				std::cout << "No image seected.\n";
				return;
			}
			NRRD::ImageView<float> image=figure.getImage();
			gui_retreive_section(gui_section());
			// Process
			std::vector<Geometry::RP2Point> result;
			if (relative_section=="") result=process(image);
			if (relative_section=="Detect Large Beads")
				{ result=large.process(image); Figure("FRST",large.frst); }
			if (relative_section=="Detect Small Beads")
				{ result=small.process(image); Figure("FRST",small.frst); }
			// Visualize
			GraphicsItems::Group bead_markers;
			BeadDetection::overlayDetectedBeads(result,bead_markers,small.paramsFRST.beadRadiiPx.back());
			figure.overlay().set("Detected Beads",bead_markers);
		}
	}


} // namespace Calibration
