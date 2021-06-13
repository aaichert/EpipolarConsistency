// Created by A. Aichert on Tue Nov 21st 2017.
#ifndef __display_geometry
#define __display_geometry

#include <GetSet/GetSetObjects.h>
#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>
#include <LibProjectiveGeometry/EigenToStr.hxx>

#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibEpipolarConsistency/RadonIntermediate.h>

#include <LibEpipolarConsistency/Gui/InputDataRadonIntermediate.h>
#include <LibEpipolarConsistency/Gui/InputDataDirect.h>

namespace EpipolarConsistency
{
	//
	// Displaying Additional Geometry
	//

	struct BoundingBox : public GetSetGui::Configurable {
		struct Style {
			bool            wire_cube=false;
			Eigen::Vector3d color=Eigen::Vector3d(0,0,1);
			int             line_width=2;
		} style;
		struct Corners {
			Eigen::Vector3d min=Eigen::Vector3d(-50,-50,-50);
			Eigen::Vector3d max=Eigen::Vector3d( 50, 50, 50);
		} corners;
		
		/// Declare default values.
		void gui_declare_section (const GetSetGui::Section& section)
		{
			// Line and Face Style
			GetSet<Eigen::Vector3d> ("Style/Color"     , section, style.color      ).setDescription("");
			GetSet<double>          ("Style/Line Width", section, style.line_width ).setDescription("");
			section.subsection("Style").setGrouped();
			// Corners
			GetSet<Eigen::Vector3d> ("Corners/Min", section, corners.min           ).setDescription("");
			GetSet<Eigen::Vector3d> ("Corners/Max", section, corners.max           ).setDescription("");
			section.subsection("Corners").setGrouped();
		}
		
		// Retreive current values from GUI
		void gui_retreive_section(const GetSetGui::Section& section)
		{
			// Line and Face Style
			style.color=GetSet<Eigen::Vector3d>      ("Style/Color"     , section);
			style.line_width=GetSet<double>          ("Style/Line Width", section);
			// Corners
			corners.min=GetSet<Eigen::Vector3d> ("Corners/Min", section);
			corners.max=GetSet<Eigen::Vector3d> ("Corners/Max", section);
		}

		/// Make a graphics item to represent a bounding box
		GraphicsItems::ConvexMesh getGraphicsItem(NRRD::ImageView<float>& img, Geometry::ProjectionMatrix *P=0x0)
		{
			Eigen::Vector3d c(255*style.color);
			if (style.wire_cube)
			{
				GraphicsItems::ConvexMesh ret=GraphicsItems::ConvexMesh::WireCube(corners.min,corners.max,QColor(c[0],c[1],c[2],c[3]));
				ret.l_style=style.line_width;
				return ret;
			}
			else
				return GraphicsItems::ConvexMesh::ColorCube(corners.min,corners.max,QColor(c[0],c[1],c[2],c[3]),style.line_width>=1);
		}

	};

	inline UtilsQt::Figure displayGeometry(const std::vector<Geometry::ProjectionMatrix>& Ps, int w, int h, double spacing, const std::string& figure_name="Geometry")
	{
		using UtilsQt::Figure;
		Figure figure(figure_name +" Source Detector Geometry",600,600);
		figure.overlay().clear();
		figure.overlay().set("CoSy",GraphicsItems::CoordinateAxes());
		GraphicsItems::Group source_detector;
		for (int i=0;i<(int)Ps.size();i++)
		{
			Eigen::Vector4d image_rect(0,0,w,h);
			auto& P(Ps[i]);
			source_detector.add(GraphicsItems::ConvexMesh::Camera(P,image_rect,spacing,false,GraphicsItems::colorByIndex(i,32)),"cam");
			source_detector.add(GraphicsItems::Text3D(toString(i),Geometry::getCameraCenter(P),GraphicsItems::colorByIndex(i)) ,"idx");
		}
		figure.overlay().set("Projections",source_detector);
		return figure;
	}

	inline UtilsQt::Figure displayInputData(const std::vector<Geometry::ProjectionMatrix>& Ps, const std::vector<EpipolarConsistency::RadonIntermediate*>& dtrs, double spacing, const std::string& figure_name="Input Data")
	{
		// Display geometry
		using UtilsQt::Figure;
		int w=dtrs[0]->getOriginalImageSize(0);
		int h=dtrs[0]->getOriginalImageSize(1);
		auto figure=displayGeometry(Ps, w, h, spacing);
		int n_alpha=dtrs.front()->getRadonBinSize(0);
		int n_t    =dtrs.front()->getRadonBinSize(1);
		// Read back all dtrs and stack em up
		NRRD::Image<float> dtr_stack;
		for (int i=0;i<(int)Ps.size();i++)
		{
			dtrs[i]->readback();
			auto& dtr_data=dtrs[i]->data();
			if (!!dtr_data && dtrs.size()<=133) // do not do this for large data sets...
			{
				if (!dtr_stack)
					dtr_stack.set((int)dtr_data.size(0),(int)dtr_data.size(1),(int)dtrs.size());
				if (dtr_stack.size(0)==dtr_data.size(0) && dtr_stack.size(1)==dtr_data.size(1))
					NRRD::ImageView<float>(dtr_stack,i).copyDataFrom<float>(dtr_data);
			}
		}
		if (!!dtr_stack)
			Figure(figure_name + " Radon Intermediate Functions",dtr_stack,0,0,dtrs.front()->isDerivative() )
				.drawLine(0,n_t*0.5,n_alpha,n_t*0.5,QColor(0,255,0))
				.drawText("alpha",20,n_alpha*0.5+20,QColor(0,255,0))
				.drawLine(n_alpha*0.5,0,n_alpha*0.5,n_t,QColor(255,0,0))
				.drawText("t",n_t*0.5+20,20,QColor(255,0,0));
		return figure;
	}

	inline UtilsQt::Figure displayInputDataRadonIntermediate(const EpipolarConsistency::InputDataRadonIntermediateGui& data, const std::string& figure_name="Input Data")
	{
		return displayInputData(data.getProjectionMatrices(),data.getRadonIntermediateFunctions(), data.getPixelSpacing(), figure_name);
	}

	/// Execute Epipolar Consistency algorithm for reference view r and visualize Samples in Radon Space.
	inline double displayRadonSamples(int r, int i, double dkappa, const std::vector<Geometry::ProjectionMatrix>& Ps,const std::vector<EpipolarConsistency::RadonIntermediate*>& dtrs, GetSetGui::ProgressInterface* progress=0x0, const std::string& figure_name="Radon Samples")
	{
		using UtilsQt::Figure;
		using namespace Geometry;
		// Make sure input parameters are valid.
		int n=(int)dtrs.size();
		if (i==r) return 0;
		if (i<0 || r<0) return 0;
		if (i>=n || r>=n) return 0;
		// Color coding indices
		auto color_i=GraphicsItems::colorByIndex(i);
		auto color_r=GraphicsItems::colorByIndex(r);
		// Show Radon intermediate functions (unless we already have visible figures)
		Figure figure_r(figure_name+" "+toString(r));
		Figure figure_i(figure_name+" "+toString(i));
		dtrs[r]->readback();
		dtrs[i]->readback();
		if (!figure_r.exists(true)) figure_r.setImage(dtrs[r]->data(),0,0,true).showTiled(0);
		if (!figure_i.exists(true)) figure_i.setImage(dtrs[i]->data(),0,0,true).showTiled(1);
		// Find reference and input projection matrices, nullspace and inverse transpose
		ProjectionMatrix P0=Ps[r];
		ProjectionMatrix P1=Ps[i];
		auto C0=getCameraCenter(P0);
		auto C1=getCameraCenter(P1);
		auto P0invT=pseudoInverse(P0).transpose().eval();
		auto P1invT=pseudoInverse(P1).transpose().eval();
		// Figure out baseline
		RP3Line   B=join_pluecker(C0,C1);
		double baseline_distance=Geometry::pluecker_distance_to_origin(B);
		// Figure out epipolar planes at 0 and 90 degrees w.r.t. the origin.
		RP3Plane  E0=join_pluecker(B,origin3);
		RP3Plane E90=join_pluecker(B,E0);
		// Convert to Hessian normal form
		E0/=E0.head(3).norm();
		E90/=E90.head(3).norm();
		// Structure to store all information about sampling process
		struct Sample {
			double kappa;
			double alpha0;
			double alpha1;
			double t0;
			double t1;
			double v0;
			double v1;
			double bin_t0;
			double bin_t1;
			double bin_alpha0;
			double bin_alpha1;
			bool   flip0;
			bool   flip1;
		};
		std::map<double,double> radon_bins_t0_by_alpha0;
		std::map<double,double> radon_bins_t1_by_alpha1;
		std::map<double,Sample> radon_samples_by_kappa;
		// Are we dealing with an odd function ?
		bool is_derivative=dtrs[r]->isDerivative();
		// Loop over epipolar planes and visualize data.
		double ecc=0;
		for (double kappa=-0.5*Pi;kappa<0.5*Pi;kappa+=dkappa)
		{
			Sample s_kappa;
			s_kappa.kappa=kappa;
			// Epipolar plane at angle k to the origin and epipolar line on reference image.
			RP3Plane  E_kappa=cos(kappa)*E0+sin(kappa)*E90;
			// Projection from plane to line
			RP2Line  l0_kappa=P0invT*E_kappa;
			RP2Line  l1_kappa=P1invT*E_kappa;
			// Convert to angle/distance
			l0_kappa/=l0_kappa.head(2).norm();
			l1_kappa/=l1_kappa.head(2).norm();
			s_kappa.alpha0=atan2(l0_kappa(1),l0_kappa(0))/Pi;
			s_kappa.alpha1=atan2(l1_kappa(1),l1_kappa(0))/Pi;
			s_kappa.t0=-l0_kappa(2);
			s_kappa.t1=-l1_kappa(2);
			// Account for coordinate origin in center of image for Radon transform.
			s_kappa.t0-=0.5*dtrs[r]->getOriginalImageSize(0)*l0_kappa(0)+0.5*dtrs[r]->getOriginalImageSize(1)*l0_kappa(1);
			s_kappa.t1-=0.5*dtrs[i]->getOriginalImageSize(0)*l1_kappa(0)+0.5*dtrs[i]->getOriginalImageSize(1)*l1_kappa(1);
			// Account for periodicity of the Radon transform (odd with a period of 0.5*Pi)
			if (s_kappa.alpha0<0) s_kappa.alpha0+=2;
			if (s_kappa.alpha1<0) s_kappa.alpha1+=2;
			s_kappa.flip0=s_kappa.alpha0>1;
			s_kappa.flip1=s_kappa.alpha1>1;
			if (s_kappa.flip0) s_kappa.t0*=-1;
			if (s_kappa.flip1) s_kappa.t1*=-1;
			if (s_kappa.flip0) s_kappa.alpha0-=1;
			if (s_kappa.flip1) s_kappa.alpha1-=1;
			// Compute Radon bin indices. Zero is in center of Radon transform.
			s_kappa.bin_alpha0=s_kappa.alpha0*dtrs[r]->getRadonBinNumber(0);
			s_kappa.bin_alpha1=s_kappa.alpha1*dtrs[i]->getRadonBinNumber(0);
			s_kappa.bin_t0    =s_kappa.t0    /dtrs[r]->getRadonBinSize(1)+0.5*dtrs[r]->getRadonBinNumber(1);
			s_kappa.bin_t1    =s_kappa.t1    /dtrs[i]->getRadonBinSize(1)+0.5*dtrs[i]->getRadonBinNumber(1);
			// Ignore Samples outside Radon transform range
			if (s_kappa.bin_t0<0 || s_kappa.bin_t0>=dtrs[r]->getRadonBinNumber(1)
			||  s_kappa.bin_t1<0 || s_kappa.bin_t1>=dtrs[i]->getRadonBinNumber(1))
				continue;
			// Store bin location in map from angle alpha to distance t for plotting
			radon_bins_t0_by_alpha0[s_kappa.bin_alpha0]=s_kappa.bin_t0;
			radon_bins_t1_by_alpha1[s_kappa.bin_alpha1]=s_kappa.bin_t1;
			// Sample Radon transforms
//			s_kappa.v0=dtrs[r]->data()(s_kappa.bin_alpha0,s_kappa.bin_t0);
//			s_kappa.v1=dtrs[i]->data()(s_kappa.bin_alpha1,s_kappa.bin_t1);
			s_kappa.v0=dtrs[r]->data().pixel(s_kappa.bin_alpha0,s_kappa.bin_t0);
			s_kappa.v1=dtrs[i]->data().pixel(s_kappa.bin_alpha1,s_kappa.bin_t1);
			// If we are dealing with an odd function (derivative) take into account perodicity with sign change.
			if (is_derivative&&s_kappa.flip0) s_kappa.v0*=-1;
			if (is_derivative&&s_kappa.flip1) s_kappa.v1*=-1;
			// Store sample by angles alpha and kappa for correct plotting
			radon_samples_by_kappa[kappa]=s_kappa;
			ecc+=(s_kappa.v0-s_kappa.v1)*(s_kappa.v0-s_kappa.v1)*dkappa;
		}
		// Draw samples
		if (radon_bins_t0_by_alpha0.size()>0)
		{
			auto& radon_samples_r=figure_r.overlay().set(toString(i), GraphicsItems::Collection<GraphicsItems::Point2D>());
			radon_samples_r.items.reserve(radon_bins_t0_by_alpha0.size());
			for (auto it=radon_bins_t0_by_alpha0.begin();it!=radon_bins_t0_by_alpha0.end();++it)
				radon_samples_r.items.push_back(GraphicsItems::Point2D(it->first,it->second,1,color_i));
		}
		{
			auto& radon_samples_i=figure_i.overlay().set(toString(r), GraphicsItems::Collection<GraphicsItems::Point2D>());
			radon_samples_i.items.reserve(radon_bins_t1_by_alpha1.size());
			for (auto it=radon_bins_t1_by_alpha1.begin();it!=radon_bins_t1_by_alpha1.end();++it)
				radon_samples_i.items.push_back(GraphicsItems::Point2D(it->first,it->second,1,color_r));
		}
		// Plot t by alpha samples
		using UtilsQt::Plot;
		Plot("0 tau by alpha").showTiled(1).graph().setData(radon_bins_t0_by_alpha0);
		Plot("1 tau by alpha").showTiled(2).graph().setData(radon_bins_t1_by_alpha1);

		// Plot redundant signals over kappa
		int m=(int)radon_samples_by_kappa.size();
		std::vector<double> v0s(m),v1s(m),kappas(m);
		int k=0;
		for (auto it=radon_samples_by_kappa.begin();it!=radon_samples_by_kappa.end();++it)
		{
			kappas[k]=it->first;
			v0s   [k]=it->second.v0;
			v1s   [k]=it->second.v1;
			k++;
		}
		std::string plot_title=std::string("Epipolar Consietency ")+toString(r,3)+"-"+toString(i,3);
		Plot plot(plot_title,true);
		plot.showTiled(0);
		plot.graph().setData(kappas,v0s).setName(std::string("Radon Intermediate ")+toString(r)).setColor(color_r);
		plot.graph().setData(kappas,v1s).setName(std::string("Radon Intermediate ")+toString(i)).setColor(color_i);
		plot.setAxisLabels("Epipolar Plane Angle","Orthogonal Derivative [a.u.]");
		plot.showLegend();
//		UtilsQt::plotByTitle(plot_title).xAxis->setTicker(QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi));
		auto ticker=QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi());
		ticker->setPiSymbol(QString::fromLatin1("\u00B0"));
		ticker->setPiValue(Geometry::Pi/180);
		UtilsQt::plotByTitle(plot_title).xAxis->setTicker(ticker);

		return ecc*baseline_distance;
	}

} // namespace EpipolarConsistency

#endif // __display_geometry
