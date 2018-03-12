// STL
#include <iostream>
#include <random>

// Qt5
#include <QApplication>
#include <QFileDialog>
#include <QPrinter>
#include <QSvgGenerator>

// Managing saving/loading parametzers and automatic GUI
#include <GetSetGui/GetSetGui.h>
#include <GetSetGui/GetSetTabWidget.h>

GetSetGui::Application g_app("TrajectoryView");

// NRRD file format
#include <NRRD/nrrd_image.hxx>

// Geometry
#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibProjectiveGeometry/CameraOpenGL.hxx>
#include <LibProjectiveGeometry/GeometryVisualization.hxx>
#include <LibProjectiveGeometry/EigenToStr.hxx>

#include <LibProjectiveGeometry/Models/ModelCameraSimilarity2D3D.hxx>
#include <LibProjectiveGeometry/Models/ModelTrajectoryIEC61217.hxx>

// GUI for parameter model (header only)
#include <LibOpterix/ParameterGui.hxx>

// Utility
#include <LibUtilsQt/Figure.hxx>
#include <LibUtilsQt/Plot.hxx>
#include <Utils/Projtable.hxx>
#include <LibProjectiveGeometry/Models/ModelTrajectoryIEC61217.hxx>
#include <Utils/UglyXML.hxx>

// Visualization
#include <LibUtilsQt/QCameraView.h>
#include <LibUtilsQt/ProjectionParameters.hxx>

// Drawing
QCameraView				*g_camera_view=0x0;
// Overlay FIXME source positions etc.
GraphicsItems::Group overlay;

// Trajectory to display
using Geometry::ProjectionMatrix;
std::vector<ProjectionMatrix>	g_Ps;

void loadProjectionMatrices()
{
	std::string file=GetSet<>("Trajectory/Projection Matrices");
	if (file.empty()) return;
	g_Ps.clear();
	std::string path=file;
	std::string name=splitRight(path,"/\\");
	std::string extension=splitRight(name,".");
	std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
	if (extension=="xml")
		g_Ps=ProjTable::loadProjtableV13(file);
	else if (extension=="txt")
		g_Ps=ProjTable::loadProjtable(file);
	else
	{
		std::map<std::string,std::string> meta;
		g_Ps=ProjTable::loadProjectionsOneMatrixPerLine(file,&meta);
		double spacing=stringTo<double>(meta["spacing"]);
		Eigen::Vector2i detector_size_px=stringTo<Eigen::Vector2i>(meta["detector size px"]);
		std::string comment=meta["comment"];
		meta.erase("spacing");
		meta.erase("detector size px");
		if (meta.size()>1)              GetSet<>("Visualization/Comment")                              =std::string("> ")+ProjTable::toMetaAttrib(meta);
		else                            GetSet<>("Visualization/Comment")                              =comment;
		if (spacing!=0)                 GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]")=spacing;
		if (!detector_size_px.isZero()) GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels")=detector_size_px;
	}
}

void updateAdditionalGeometry()
{
	std::cout << "Rebuilding scene...\n";
	std::vector<Eigen::Vector4d> pts_red=GetSet<std::vector<Eigen::Vector4d> >("Visualization/Additional Geometry/Points 0");
	std::vector<Eigen::Vector4d> pts_blu=GetSet<std::vector<Eigen::Vector4d> >("Visualization/Additional Geometry/Points 1");
	bool points=GetSet<bool>("Visualization/Additional Geometry/Show Points");
	bool lines=GetSet<bool>("Visualization/Additional Geometry/Show Lines");
	if (points && !pts_red.empty() && !pts_blu.empty())
	{
		// Red Points:
		auto& scene_pts_red=g_camera_view->graphics.set("Red Points",GraphicsItems::ConvexMesh());
		auto& scene_pts_blu=g_camera_view->graphics.set("Blue Points",GraphicsItems::ConvexMesh());
		scene_pts_red.p_pos=pts_red;
		scene_pts_red.p_color=QColor(255,0,0,255);
		scene_pts_red.p_style=2;
		// Blue Points:
		scene_pts_blu.p_pos=pts_blu;
		scene_pts_blu.p_color=QColor(0,255,0,255);
		scene_pts_blu.p_style=2;
	}
	int n=std::min((int)pts_red.size(),(int)pts_blu.size());
	if (lines && n>0)
	{
		auto& scene_lines=g_camera_view->graphics.set("Lines",GraphicsItems::ConvexMesh());
		scene_lines.p_pos.reserve(2*n);
		scene_lines.l_index.reserve(n);
		for (int i=0;i<n;i++)
		{
			scene_lines.p_pos.push_back(pts_red[i]);
			scene_lines.p_pos.push_back(pts_blu[i]);
			scene_lines.l_index.push_back(Eigen::Vector2i(2*i,2*i+1));
		}
		scene_lines.l_style=1;
	}
	std::cout << "Done.\n";
}

/// Convert RTK parameters from GUI to a vector of the corresponding parameter model
std::vector<Geometry::ModelTrajectoryCircularIEC61217> guiGetRTK()
{
	auto param_names=Geometry::ModelTrajectoryCircularIEC61217::ParameterNames();
	std::vector<std::vector<double> > rtk_raw_data(param_names.size());
	GetSetGui::Section parameters_gui("Trajectory/Generate/Parameters");
	int maxlen=0;
	for (int p=0;p<(int)param_names.size();p++)
	{
		rtk_raw_data[p]=GetSet<std::vector<double>>(toString(p, 2)+" "+param_names[p],parameters_gui).getValue();
	
		if (rtk_raw_data[p].empty())
			if (p==0) // "Trajectory/Generate/Parameters/00 Gantry Angle"
			{
				// Generate amgles according to other GUI fields
				int N=GetSet<int>("Trajectory/Generate/Sampling/Number of Projections");
				double start_angle=GetSet<double>("Trajectory/Generate/Sampling/Primary Angle (start)");
				double end_angle=GetSet<double>("Trajectory/Generate/Sampling/Primary Angle (end)");
				std::vector<double> primary_angles(N);
				for (int i=0;i<N;i++)
				{
					double pct=(double)i/(N-1);
					primary_angles[i]=start_angle*(1-pct)+end_angle*pct;
				}
				rtk_raw_data[0]=primary_angles;
			}
			else
				return std::vector<Geometry::ModelTrajectoryCircularIEC61217>();
		if (rtk_raw_data[p].size()>maxlen)
			maxlen=(int)rtk_raw_data[p].size();
	}
	std::set<int> active;
	std::vector<Geometry::ModelTrajectoryCircularIEC61217> ret(maxlen);
	for (int i=0;i<maxlen;i++)
	{
		for (int p=0;p<(int)param_names.size();p++)
		{
			if (rtk_raw_data[p].size()>1)
				active.insert(p);
			ret[i].current_values[p]=rtk_raw_data[p][i%rtk_raw_data[p].size()];
		}
	}
	return ret;
}

/// Refresh 3D visualization of trajectory
void draw3DVisualization(QPainter &p, QCameraView* c)
{
	ProjectionMatrix &P(c->P);

	double cam_spacing=GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]");
	auto cam_size=GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels").getValue();
	auto image_rect=Eigen::Vector4d(0,0,cam_size[0],cam_size[1]);
	int selected_cam=GetSet<int>("Visualization/Trajectory Selection");
	int increment=1+GetSet<int>("Visualization/Trajectory Skip Projections");

	if (increment>0) 
		for (int i=0; i<(int)g_Ps.size(); i+=increment)
			if (i!=selected_cam)
				GraphicsItems::ConvexMesh::Camera(g_Ps[i],image_rect,cam_spacing,false,QColor(0,0,0,15)).draw(p,P,1);

	if (selected_cam>=0 && selected_cam<(int)g_Ps.size())
	{
		bool P_is_selected_cam = (P*Geometry::getCameraCenter(g_Ps[selected_cam])).normalized()[2]<1e-8;
		if (!P_is_selected_cam)
			GraphicsItems::FancyCamera(g_Ps[selected_cam],image_rect,cam_spacing).draw(p,P,1);
	}

	p.setPen(QColor(0,0,0,255));
	p.drawText(30,30,GetSet<>("Visualization/Comment").getValue().c_str());


	overlay.draw(p,P,1);

}

/// Call back function for GUI interaction
void gui(const GetSetInternal::Node& node)
{
	const std::string& section=node.super_section;
	const std::string& key    =node.name;

	if (key=="Projection Matrices")
		loadProjectionMatrices();
	
	if (section=="Transform/Temporal Undersampling" && key=="Apply")
	{
		int skip=GetSet<int>("Temporal Undersampling/Skip");
		int start=GetSet<int>("Transform/Temporal Undersampling/Start");
		int stop=GetSet<int>("Transform/Temporal Undersampling/Stop");
		if (stop<0||stop>(int)g_Ps.size()) stop=(int)g_Ps.size();
		if (start<0) start=0;
		if (skip<1)  skip=1;
		GetSet<int>("Transform/Temporal Undersampling/Skip")=skip;
		GetSet<int>("Transform/Temporal Undersampling/Start")=start;
		GetSet<int>("Transform/Temporal Undersampling/Stop")=stop;
		std::vector<Geometry::ProjectionMatrix> new_Ps;
		for (int i=start;i<stop;i+=skip)
			new_Ps.push_back(g_Ps[i]);
		g_Ps=new_Ps;
	}

	if (key=="Flip Image u-Axes")
	{
		auto cam_size=GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels").getValue();
		Eigen::Matrix3d H=Eigen::Matrix3d::Identity();
		H(0,0)*=-1;
		H(0,2)=cam_size[0];
		std::cout << "H=\n" << H << std::endl;
		for (int i=0;i<(int)g_Ps.size();i++)
			g_Ps[i]=H*g_Ps[i];
	}

	if (key=="Flip Image v-Axes")
	{
		auto cam_size=GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels").getValue();
		Eigen::Matrix3d H=Eigen::Matrix3d::Identity();
		H(1,1)*=-1;
		H(1,2)=cam_size[1];
		std::cout << "H=\n" << H << std::endl;
		for (int i=0;i<(int)g_Ps.size();i++)
			g_Ps[i]=H*g_Ps[i];	
	}

	if (key=="Apply 2x2 binning")
	{
		auto cam_size=GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels").getValue();
		cam_size/=2;
		GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels")=cam_size;
		GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]")=GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]")*2;
		Eigen::Matrix3d H=Eigen::Matrix3d::Identity();
		H(1,1)=H(0,0)=.5;
		std::cout << "H=\n" << H << std::endl;
		for (int i=0;i<(int)g_Ps.size();i++)
			g_Ps[i]=H*g_Ps[i];
	}

	if (section=="Additional Geometry")
		updateAdditionalGeometry();

	if (section=="Trajectory/Generate/RTK Parameters")
	{
		Geometry::RP2Homography model_matrix=GetSet<Geometry::RP2Homography>("Trajectory/Generate/Detector Model Matrix [ps to mm]");
		auto detector_size=GetSet<Eigen::Vector2i>("Trajectory/Generate/Sampling/Detector Number of Pixels").getValue();
		if (key=="Load")
		{
			std::string path=QFileDialog::getOpenFileName(0x0, "Open RTK Circular Trajectory", "", "RTK Circular (*.xml);;All Files (*)").toStdString();
			if (path.empty()) return;
			auto projections=Geometry::loadCircularTrajectoryRTK(path);
			// Copy values to GUI
			std::map<int, std::vector<double> > raw_values;
			for (int i=0;i<(int)projections.size();i++)
			{
				for (int p=0;p<projections[i].numberOfParameters();p++)
				{
					if (projections[i].activeParameters().find(p)!=projections[i].activeParameters().end() || i==0)
						raw_values[p].push_back(projections[i].current_values[p]);
				}
			}
			GetSetGui::Section parameters_gui("Trajectory/Generate/Parameters");
			auto param_names=Geometry::ModelCameraSimilarity2D3D::ParameterNames();
			for (int i=0; i<(int)param_names.size(); i++)
				GetSet<std::vector<double>>(toString(i, 2)+" "+param_names[i],parameters_gui)=raw_values[i];
		}
		if (key=="Save")
		{
			auto rtk_params=guiGetRTK();
			if (rtk_params.empty())
			{
				g_app.warn("RTK Circular Geometry","Please make sure that you specify at least one value for each parameter.");
				return;
			}
			std::string path=QFileDialog::getSaveFileName(0x0, "Save RTK Circular Trajectory", "", "RTK Circular (*.xml);;All Files (*)").toStdString();
			if (path.empty()) return;
			if (!Geometry::saveCircularTrajectoryRTK(path,rtk_params)) g_app.warn("File access error", "Failed to save RTK Circular Trajectory");
		}
		if (key=="Apply")
		{
			auto rtk_params=guiGetRTK();
			if (rtk_params.empty())
			{
				g_app.warn("RTK Circular Geometry","Please make sure that you specify at least one value for each parameter.");
				return;
			}
			g_Ps.resize(rtk_params.size());
			for (int i=0;i<(int)rtk_params.size();i++)
				g_Ps[i]=rtk_params[i].setImageModelMatrixInverse(model_matrix.inverse()).getInstance();

			GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]")=model_matrix.block<2,1>(0,0).norm();
			GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels")=detector_size;

			std::string summary;
			auto param_tags=Geometry::ModelTrajectoryCircularIEC61217::ParameterTagsXML();
			for (int p=0;p<rtk_params.front().numberOfParameters();p++)
				if (rtk_params.front().activeParameters().find(p)==rtk_params.front().activeParameters().end() && rtk_params.front().current_values[p]!=0)
					summary+=param_tags[p]=param_tags[p]+"="+toString(rtk_params.front().current_values[p])+" ";
			GetSet<>("Visualization/Comment")=summary;
		}
	}

	if (section=="Visualization/Additional Geometry/Bounding Box")
	{
		using GraphicsItems::ConvexMesh;
		auto cube_min=GetSet<Eigen::Vector3d>("Visualization/Additional Geometry/Bounding Box/Min").getValue();
		auto cube_max=GetSet<Eigen::Vector3d>("Visualization/Additional Geometry/Bounding Box/Max").getValue();
		g_camera_view->graphics.set("Bounding Box", ConvexMesh::Cube(cube_min,cube_max));
	}

	if (key=="Show Source Positions")
	{
		int n=(int)g_Ps.size();
		std::vector<Eigen::Vector4d> pts_red;
		for (int i=0;i<n;i++)
			pts_red.push_back(Geometry::getCameraCenter(g_Ps[i]));
		GetSet<std::vector<Eigen::Vector4d> >("Visualization/Additional Geometry/Points 0")=pts_red;
		GetSet<bool>("Visualization/Additional Geometry/Show Points")=true;
	}

	if (key=="Show Principal Point Offsets")
	{
		double spacing=GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]");
		auto image_size=GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels").getValue();
		int n=(int)g_Ps.size();
		std::vector<Eigen::Vector4d> pts_red, pts_blu;
		for (int i=0;i<n;i++)
		{
			// C 00 0h wh w0 pp qxes
			std::vector<Geometry::RP3Point> frustum;
			Geometry::cameraFrustum(g_Ps[i],Eigen::Vector4d(0,0,image_size[0],image_size[1]),spacing,frustum);
			Geometry::RP3Point detector_center=0.25*(frustum[1]/frustum[1][3]+frustum[2]/frustum[2][3]+frustum[3]/frustum[3][3]+frustum[4]/frustum[4][3]);
			pts_red.push_back(detector_center);
			Geometry::RP3Point principal_point_3d=frustum[5];
			pts_blu.push_back(principal_point_3d);
		}
		g_app.ignoreNotifications(true);
		GetSet<std::vector<Eigen::Vector4d> >("Visualization/Additional Geometry/Points 0")=pts_red;
		GetSet<std::vector<Eigen::Vector4d> >("Visualization/Additional Geometry/Points 1")=pts_blu;
		GetSet<bool>("Visualization/Additional Geometry/Show Points")=false;
		g_app.ignoreNotifications(false);
		GetSet<bool>("Visualization/Additional Geometry/Show Lines")=true;
	}
	
	if (key=="Edit Plots...")
		UtilsQt::showPlotEditor(true);

	if (key=="Open...")
	{
		std::string path=GetSet<>("Trajectory/Projection Matrices");
		std::string new_path=QFileDialog::getOpenFileName(0x0, "Open Trajectory", path.c_str(), "One Matrix Per Line (*.ompl);;Siemens Projtable (*.txt);;Siemens Projtable V1.3 (*.xml);;All Files (*)").toStdString();
		if (new_path.empty()) return;
		GetSet<>("Trajectory/Projection Matrices")=new_path;
	}

	if (key=="Save...")
	{
		std::string path=GetSet<>("Output File");
		GetSet<>("Output File")="";
		if (path.empty())
			path=QFileDialog::getSaveFileName(0x0, "Save Trajectory", GetSet<>("Trajectory/Projection Matrices").getString().c_str(), "One Matrix Per Line (*.ompl);;All Files (*)").toStdString();
		if (path.empty()) return;
		if (!ProjTable::saveProjectionsOneMatrixPerLine(g_Ps,path,
														GetSet<>("Visualization/Comment"),
														GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]"),
														GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels")))
			g_app.warn("Feil Access Error", std::string("Failed to write OMPL file ")+ path);
	}

	//
	// Import
	//


	if (key=="CONRAD XML...")
	{
		std::string path = QFileDialog::getOpenFileName(0x0, "Import \"PMatrixSerialization\" from edu.stanford.rsl.conrad.geometry.Projection", "conrad.xml", "CONRAD condig (*.xml);;All Files (*)").toStdString();
		if (path.empty()) return;
		auto Ps=ProjTable::loadCONRAD(path,&g_app);
		if (Ps.empty()) g_app.warn("Import Failed","Please see console for details.");
		else g_Ps=Ps;
		GetSet<>("Trajectory/Projection Matrices")="";
	}

	if (key=="RTK geometry file...")
	{
		std::string path = QFileDialog::getOpenFileName(0x0, "Import IEC 61217 Geometry from RTK file", "rtk.xml", "RTK Geometry File (*.xml);;All Files (*)").toStdString();
		if (path.empty()) return;

		// 	Geometry::ModelTrajectoryCircularIEC61217:
		// 2DO !!!
	}


	//
	// Export
	//

	if (key=="Siemens projtable...")
	{
		std::string path=GetSet<>("Output File");
		GetSet<>("Output File")="";
		if (path.empty()) path = QFileDialog::getSaveFileName(0x0, "Export as Siemens projtable", "projtable.txt", "Siemens Projtable (*.txt);;All Files (*)").toStdString();
		if (path.empty()) return;
		ProjTable::saveProjtable(g_Ps,path);
	}

	if (key=="Current View as PDF...")
	{
		std::string export_path=GetSet<>("Export Path");
		GetSet<>("Export Path")="";
		if (export_path.empty())
			export_path = QFileDialog::getSaveFileName(0x0, "Export as PDF", "VirtualPlane.pdf", "Portable Document Format (*.pdf);;All Files (*)").toStdString();
		if (export_path.empty()) return;
		g_camera_view->savePDF(export_path.c_str());
	}

	if (key=="Current View as PNG...")
	{
		std::string export_path=GetSet<>("Export Path");
		GetSet<>("Export Path")="";
		if (export_path.empty())
			export_path = QFileDialog::getSaveFileName(0x0, "Export as PNG", "VirtualPlane.png", "Portable Network Graphics (*.png);;All Files (*)").toStdString();
		if (export_path.empty()) return;
		g_camera_view->grab().save(export_path.c_str());
	}

	if (key=="Current View as SVG...")
	{
		std::string export_path=GetSet<>("Export Path");
		GetSet<>("Export Path")="";
		if (export_path.empty())
			export_path = QFileDialog::getSaveFileName(0x0, "Export as SVG", "TrajectoryView.svg", "Scalable Vector Graphics (*.svg);;All Files (*)").toStdString();
		if (export_path.empty()) return;
		QSvgGenerator generator;
		generator.setFileName(export_path.c_str());
		generator.setSize(g_camera_view->size());
		generator.setViewBox(g_camera_view->rect());
		QPainter painter;
		painter.begin(&generator);
		draw3DVisualization(painter, g_camera_view);		
		g_camera_view->graphics.draw(painter, g_camera_view->P,1);
		painter.end();
	}

	// Make Y-axis point upwards
	if (key=="Y-axis Upwards")
		GetSet<Eigen::Vector4d>("Projection/Rotation")=Eigen::Vector4d(g_camera_view->projection->angle[0],g_camera_view->projection->angle[1],0,0);

	// Make Z-axis point upwards
	if (key=="Z-axis Upwards")
		GetSet<Eigen::Vector4d>("Projection/Rotation")=Eigen::Vector4d(g_camera_view->projection->angle[0],g_camera_view->projection->angle[1],Geometry::Pi*-0.5,0);

	// Make Y-axis point downwards
	if (key=="Y-axis Downwards")
		GetSet<Eigen::Vector4d>("Projection/Rotation")=Eigen::Vector4d(g_camera_view->projection->angle[0],g_camera_view->projection->angle[1],Geometry::Pi,0);

	// Make Z-axis point downwards
	if (key=="Z-axis Downwards")
		GetSet<Eigen::Vector4d>("Projection/Rotation")=Eigen::Vector4d(g_camera_view->projection->angle[0],g_camera_view->projection->angle[1],Geometry::Pi*0.5,0);

	// Plots...
	if (key.substr(0,4)=="Plot")
	{
		std::vector<Geometry::RP3Point> source_pos;
		std::vector<double> alphas,betas,sids,sdds;
		ProjTable::ctCircularTrajectoryToParameters(g_Ps,GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]"),0x0,&alphas,&betas,&sids,&sdds,&source_pos);
		int n=(int)g_Ps.size();
		if (n<1) return;

		if (key=="Plot Primary and Secondary Angles..." || key=="Plot All...")
		{
			double mean_beta,stddev_beta;
			{
				std::vector<double> &v(betas);
				double sum = std::accumulate(v.begin(), v.end(), 0.0);
				mean_beta = sum / v.size();
				double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
				stddev_beta = std::sqrt(sq_sum / v.size() - mean_beta * mean_beta);
			}

			UtilsQt::Plot("Primary Angle")
				.setAxisLabels("Projection Index","Angle [deg]")
				.showLegend()
				.graph(0)
					.setData(alphas)
					.setName(std::string("Primary Angle  min=") + toString(*std::min_element(alphas.begin(), alphas.end()))
									+ " max="  + toString(*std::max_element(alphas.begin(), alphas.end())) );

			for (int i=n-1;i>0;i--)
				alphas[i]-=alphas[i-1];
			alphas[0]=0;
			double mean_diff_alpha,stddev_diff_alpha;
			{
				std::vector<double> &v(alphas);
				double sum = std::accumulate(v.begin(), v.end(), 0.0);
				mean_diff_alpha = sum / (v.size()-1);
				double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
				stddev_diff_alpha = std::sqrt(sq_sum / (v.size()-1) - mean_diff_alpha * mean_diff_alpha);
			}

			UtilsQt::Plot angular_motion("Angular Motion");
			angular_motion.setAxisLabels("Projection Index","Angle [deg]");
			angular_motion.graph(0).setData(alphas).setName(std::string("Primary Angle Difference µ=") + toString(mean_diff_alpha)  + " σ=" + toString(stddev_diff_alpha));
			angular_motion.graph(1).setData(betas ).setName(std::string("Secondary Angle  µ=")         + toString(mean_beta)        + " σ=" + toString(stddev_beta) );
			angular_motion.showLegend();

		}

		if (key=="Plot Source Positions..." || key=="Plot All...")
		{
			std::vector<double> source_x(n), source_y(n), source_z(n);
			for (int i=0;i<n;i++)
			{
				source_x[i]=source_pos[i](0);
				source_y[i]=source_pos[i](1);
				source_z[i]=source_pos[i](2);
			}
			UtilsQt::Plot source_position("Source Position");
			source_position.graph(0).setData(source_x).setName("Source X");
			source_position.graph(1).setData(source_y).setName("Source Y");
			source_position.graph(2).setData(source_z).setName("Source Z");
			source_position
				.setAxisLabels("Projection Index","World Coordinates [mm]")
				.showLegend();
		}
	
		if (key=="Plot Principal Points..." || key=="Plot All...")
		{
			std::vector<double> pp_x(n),pp_y(n);
			double mean_pp_x=0,mean_pp_y=0;
			for (int i=0;i<n;i++)
			{
				Geometry::RP2Point pp=Geometry::getCameraPrincipalPoint(g_Ps[i]);
				pp_x[i]=pp(0);
				pp_y[i]=pp(1);
				mean_pp_x+=pp_x[i];
				mean_pp_y+=pp_y[i];
			}
			mean_pp_x/=(double)n;
			mean_pp_y/=(double)n;
			UtilsQt::Plot principal_point("Prinipal Point");
			principal_point.graph(0).setData(pp_x).setName(std::string("Prinipal Point u µ=")+toString(mean_pp_x));
			principal_point.graph(1).setData(pp_y).setName(std::string("Prinipal Point v µ=")+toString(mean_pp_y));
			principal_point.setAxisLabels("Projection Index","Image Coordinates [px]").showLegend();
		}

		if (key=="Plot Source Distances..." || key=="Plot All...")
		{
			// Compute mean sid and sdd
			double mean_sid=0;
			double mean_sdd=0;
			double max_sdd_sid=0;
			#pragma omp parallel for
			for (int i=0;i<n;i++)
			{
				 mean_sid+=sids[i];
				 mean_sdd+=sdds[i];
				 if (max_sdd_sid<sids[i])
					max_sdd_sid=sids[i];
				 if (max_sdd_sid<sdds[i])
					max_sdd_sid=sdds[i];
			}
			mean_sid/=(double)n;
			mean_sdd/=(double)n;
			max_sdd_sid++;

			UtilsQt::Plot source_distance("Source Distance");
			source_distance.graph(0).setData(sids).setName(std::string("sid µ=")+toString((int)mean_sid));
			source_distance.graph(1).setData(sdds).setName(std::string("sdd µ=")+toString((int)mean_sdd));
			source_distance.setAxisLabels("Projection Index","Distance [mm]").showLegend();
		}
	}
	
	if (hasPrefix(node.super_section,"Transform/Geometry"))
	{
		using namespace Geometry;
		using GetSetGui::Section;
		overlay.clear();
		Section geometry("Transform/Geometry");
		auto p2d=LibOpterix::index_prefix(ModelSimilarity2D::ParameterNames());
		auto p3d=LibOpterix::index_prefix(ModelSimilarity3D::ParameterNames());
		if (node.name=="Reset")
		{
			Section("2D Parameters", geometry).setMultipleKeys<double>(std::vector<double>(p2d.size(),0.0), p2d );
			Section("3D Parameters", geometry).setMultipleKeys<double>(std::vector<double>(p3d.size(),0.0), p3d );
			GetSet<>("Trajectory/Projection Matrices")=GetSet<>("Trajectory/Projection Matrices").getValue();
		}

		std::vector<double> parametrs_2D=Section("2D Parameters", geometry).getMultipleKeys<double>(p2d);
		std::vector<double> parametrs_3D=Section("3D Parameters", geometry).getMultipleKeys<double>(p3d);

		RP2Homography H=ModelSimilarity2D().setCurrentValues(parametrs_2D.data()).getInstance();
		RP3Homography T=ModelSimilarity3D().setCurrentValues(parametrs_3D.data()).getInstance();
		std::cout << "H=\n" << H << "\n" << "T=\n" << T << "\n\n";
		if (H.isIdentity() && T.isIdentity()) return;
		if (node.name=="Apply")
		{
			for (int i=0;i<(int)g_Ps.size();i++)
				g_Ps[i]=H*g_Ps[i]*T;
			GetSetIO::debug_print(geometry);
			overlay.clear();
		}
		else
		{
			// Get stuff related to drawing from GUI
			double cam_spacing=GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]");
			auto cam_size=GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels").getValue();
			auto image_rect=Eigen::Vector4d(0,0,cam_size[0],cam_size[1]);
			int increment=1+GetSet<int>("Visualization/Trajectory Skip Projections");
			// perview...
			overlay.clear();
			if (increment>0) 
				for (int i=0; i<(int)g_Ps.size(); i+=increment)
						overlay.add(GraphicsItems::ConvexMesh::Camera(H*g_Ps[i]*T,image_rect,cam_spacing,false,QColor(0,0,255,10)));
		}
	}

	if (g_camera_view) g_camera_view->update();
	// Save settings to file
	g_app.saveSettings();
}

/// Define GUI and start 3D visualization
int main(int argc, char ** argv)
{
	// Visualization
	GetSet<int>("Visualization/Trajectory Selection")=0;
	GetSet<int>("Visualization/Trajectory Skip Projections") = 0;

	// Visualization (Additional Geometry)
	GetSet<Eigen::Vector3d>("Visualization/Additional Geometry/Bounding Box/Min").setString("-50 -50 -50");
	GetSet<Eigen::Vector3d>("Visualization/Additional Geometry/Bounding Box/Max").setString("+50 +50 +50");
	GetSetGui::Section("Visualization/Additional Geometry/Bounding Box").setGrouped();
	GetSet<std::vector<Eigen::Vector4d> >("Visualization/Additional Geometry/Points 0");
	GetSet<std::vector<Eigen::Vector4d> >("Visualization/Additional Geometry/Points 1");
	GetSet<bool>("Visualization/Additional Geometry/Show Lines")=false;
	GetSet<bool>("Visualization/Additional Geometry/Show Points")=false;
	GetSet<double>("Visualization/Additional Geometry/Coordinate Axes Length")=100;
	GetSet<>("Visualization/Comment")="";

	// Trajectory
	GetSetGui::File("Trajectory/Projection Matrices").setMultiple(true).setExtensions("All Files (*);;Siemens Projtable (*.txt);;Siemens Projtable 1.3 (*.xml);;One Projection Matrix Per Line (*.ompl)");
	GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]")=.308;
	GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels").setString("1240 960");

	// Temporal Undersampling
	{
		GetSetGui::Button("Transform/Temporal Undersampling/Apply") = "Reduce number of views...";
		GetSetGui::Section temporal_undersampling("Transform/Temporal Undersampling");
		GetSet<int>("Skip" ,temporal_undersampling)=2;
		GetSet<int>("Start",temporal_undersampling)=0;
		GetSet<int>("Stop" ,temporal_undersampling)=-1;
		temporal_undersampling.setGrouped();
	}

	// Transformation by 2D and 3D similarity transform
	{
		using namespace Geometry;
		using namespace GetSetGui;
		using namespace LibOpterix;
		Section geometry("Transform/Geometry");
		Button("Actions/Apply" , geometry).setDescription("Applies the transform to the trajectory.") = "Apply...";
		Button("Actions/Reset" , geometry).setDescription("Resets the parameters to identity."      ) = "Reset...";
		Section("2D Parameters", geometry).setGrouped().getMultipleKeys<double>(index_prefix(ModelSimilarity2D::ParameterNames()));
		Section("3D Parameters", geometry).setGrouped().getMultipleKeys<double>(index_prefix(ModelSimilarity3D::ParameterNames()));
		geometry.subsection("Actions").setGrouped();
	}
	
	// Generate from RTK Parameters
	{
		GetSetGui::Button ("Trajectory/Generate/RTK Parameters/Apply")="Generate...";
		GetSetGui::Button ("Trajectory/Generate/RTK Parameters/Load" )="Load XML...";
		GetSetGui::Button ("Trajectory/Generate/RTK Parameters/Save" )="Save XML...";
		GetSetGui::Section("Trajectory/Generate/RTK Parameters").setGrouped();
		GetSetGui::Section parameters_gui("Trajectory/Generate/Parameters");
		auto param_names=Geometry::ModelTrajectoryCircularIEC61217::ParameterNames();
		GetSetGui::StaticText("Trajectory/Generate/Parameters_")=
			"Access is modulated with array length.\n"
			"You can specify each parameter globally by just providing one value\n"
			"or per-projection by providing a vector of values.";
		for (int i=0; i<(int)param_names.size(); i++)
			GetSet<std::vector<double>>(toString(i, 2)+" "+param_names[i],parameters_gui);
		GetSet<Eigen::Vector2i>("Trajectory/Generate/Sampling/Detector Number of Pixels").setString("1240 960");
		GetSet<Geometry::RP2Homography>("Trajectory/Generate/Detector Model Matrix [ps to mm]")=Geometry::Translation(-640,-480) *Geometry::Scale(.308,.308);
		GetSet<int>("Trajectory/Generate/Sampling/Number of Projections").setDescription("Ignored if Gantry Angles are provided explicitly")=360;
		GetSet<double>("Trajectory/Generate/Sampling/Primary Angle (start)").setDescription("Ignored if Gantry Angles are provided explicitly")=0;
		GetSet<double>("Trajectory/Generate/Sampling/Primary Angle (end)").setDescription("Ignored if Gantry Angles are provided explicitly")=Geometry::Pi*2;
		GetSetGui::Section("Trajectory/Generate/Sampling").setGrouped();
	}

	// Projection parameters
	UtilsQt::ProjectionParametersGui* projection_parameters=new UtilsQt::ProjectionParametersGui(GetSetGui::Section("Projection"),&g_app);


	g_app.commandLine().index("Trajectory/Projection Matrices",1,false);
	g_app.init(argc,argv,gui);
	g_app.window().addMenuItem("File","Open...","Ctrl+O");
	g_app.window().addMenuItem("File","Save...","Ctrl+S");
	g_app.window().addMenuItem("File");
	g_app.window().addMenuItem("File");
	g_app.window().addMenuItem("File/Import","CONRAD XML...");
	g_app.window().addMenuItem("File/Import","RTK geometry file...");
	g_app.window().addMenuItem("File/Export","Siemens projtable...","Ctrl+Shift+S");
	g_app.window().addMenuItem("File/Export");
	g_app.window().addMenuItem("File/Export","Current View as PDF...","Ctrl+P");
	g_app.window().addMenuItem("File/Export","Current View as SVG...");
	g_app.window().addMenuItem("File/Export","Current View as PNG...");
	
	g_app.window().addMenuItem("File");
	g_app.window().addDefaultFileMenu();
	g_app.window().addMenuItem("Utility/Visualization","Show Source Positions");
	g_app.window().addMenuItem("Utility/Visualization","Show Principal Point Offsets");
	g_app.window().addMenuItem("Utility/Visualization/Axis orientation","Y-axis Upwards");
	g_app.window().addMenuItem("Utility/Visualization/Axis orientation","Z-axis Upwards");
	g_app.window().addMenuItem("Utility/Visualization/Axis orientation","Y-axis Downwards");
	g_app.window().addMenuItem("Utility/Visualization/Axis orientation","Z-axis Downwards");
	g_app.window().addMenuItem("Utility/Transformation","Flip Image u-Axes");
	g_app.window().addMenuItem("Utility/Transformation","Flip Image v-Axes");
	g_app.window().addMenuItem("Utility/Transformation","Apply 2x2 binning");
	g_app.window().addMenuItem("Utility/Plot","Plot All...");
	g_app.window().addMenuItem("Utility/Plot");
	g_app.window().addMenuItem("Utility/Plot","Plot Source Positions...");
	g_app.window().addMenuItem("Utility/Plot","Plot Primary and Secondary Angles...");
	g_app.window().addMenuItem("Utility/Plot","Plot Source Distances...");
	g_app.window().addMenuItem("Utility/Plot","Plot Principal Points...");
	g_app.window().addMenuItem("Utility/Plot");
	g_app.window().addMenuItem("Utility/Plot","Edit Plots...");
	g_app.window().aboutText()=
		"<h4>Visualization of Flat-Panel Detector CT Trajectories.</h4>\n\n"
		"Copyright 2014-2018 by <a href=\"mailto:aaichert@gmail.com?Subject=[Epipolar Consistency]\">Andre Aichert</a> <br>"
		"<br>"
		"See also: "
		"<br>"
		"<a href=\"https://www5.cs.fau.de/research/software/epipolar-consistency/\">Pattern Recognition Lab at Technical University of Erlangen-Nuremberg</a> "
		"<br>"
		"<h4>Licensed under the Apache License, Version 2.0 (the \"License\")</h4>\n\n"
		"You may not use this file except in compliance with the License. You may obtain a copy of the License at "
		"<a href=\"http://www.apache.org/licenses/LICENSE-2.0\">http://www.apache.org/licenses/LICENSE-2.0</a><br>"
		;
	// Update Visualization
	g_app.ignoreNotifications(true);
	g_camera_view=new QCameraView(0x0,projection_parameters);
	g_app.ignoreNotifications(false);
	auto cube_min=GetSet<Eigen::Vector3d>("Visualization/Additional Geometry/Bounding Box/Min").getValue();
	auto cube_max=GetSet<Eigen::Vector3d>("Visualization/Additional Geometry/Bounding Box/Max").getValue();
	g_camera_view->graphics.set("Bounding Box",GraphicsItems::ConvexMesh::Cube(cube_min,cube_max));
	g_camera_view->graphics.set("CoSy", GraphicsItems::CoordinateAxes(GetSet<double>("Visualization/Additional Geometry/Coordinate Axes Length")));
	auto spacing=GetSet<double>("Trajectory/Detector Pixel Spacing [mm per px]").getValue();
	auto detector_size=GetSet<Eigen::Vector2i>("Trajectory/Detector Number of Pixels").getValue();
	g_camera_view->setDrawFunc(draw3DVisualization);
	g_camera_view->setCloseFunc(QCoreApplication::quit);
	g_camera_view->show();

	// Main window
	QMainWindow *main=new QMainWindow();
	main->setCentralWidget(g_camera_view);
	QDockWidget *dock=new QDockWidget();
	dock->setWidget(&g_app.window());
	main->addDockWidget(Qt::LeftDockWidgetArea, dock);
	dock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar );
	main->resize(1200,600);
	main->show();

	// Set up projection
	loadProjectionMatrices();
	updateAdditionalGeometry();

	return g_app.exec();
}

