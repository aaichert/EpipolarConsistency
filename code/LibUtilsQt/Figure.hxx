#ifndef __qt_nrrd
#define __qt_nrrd
// Created by aaichert on Mon May 5th 2016

#include "FigureWindow.h"

#include "nrrdqt.hxx"

#include <QApplication>
#include <QDesktopWidget>

#include <GetSetGui/GetSetModalDialog.h>

namespace UtilsQt {

	/// Simple class to show a NRRD image and draw to it.
	class Figure {
		std::string name;
	public:

		/// Create a new named Figure by appending a number to a prefix.
		static Figure New(const std::string& prefix="Figrue")
		{
			for (int i=0;;i++)
				if (!Figure(prefix+toString(i,4,' ')).exists(false))
					return Figure(prefix+toString(i,4,' '));
		}

		/// Select any open Figure
		static Figure Select(bool must_have_image=false, bool must_have_projection_matrix=false)
		{
			GetSetGui::GetSetModalDialog select_figure;
			std::ostringstream strstr;
			auto instances=UtilsQt::FigureWindow::instance_names(must_have_image,must_have_projection_matrix);
			if (instances.empty()) return Figure("");
			GetSetGui::Enum("Figure",select_figure).setChoices(setToString(instances,";"));
			if (!select_figure.exec("Please Choose an image source.")) return Figure("");
			else return UtilsQt::Figure(GetSet<>("Figure",select_figure));
		}
		
		/// Close all windows
		static void closeAll(bool must_have_image=false, bool must_have_projection_matrix=false, bool must_be_invisible=false)
		{
			auto instances=UtilsQt::FigureWindow::instance_names(must_have_image,must_have_projection_matrix);
			if (must_be_invisible) {
				for (auto it=instances.begin();it!=instances.end();++it)
					if   (FigureWindow::exists(*it,true)) continue;
					else FigureWindow::instance_delete(*it);
			}
			else
				for (auto it=instances.begin();it!=instances.end();++it)
				 FigureWindow::instance_delete(*it);
		}

		/// Draw to an existing window.
		Figure(const std::string& _name="") : name(_name) {}

		/// Draw GraphicsItems on a plain background
		Figure(const std::string& _name, int w, int h, const QColor& color=QColor(255,255,255,255))
			: name(_name)
		{
			auto& figure=FigureWindow::instance(name);
			figure.overlay.clear();
			figure.background_color=color;
			figure.setVisibleSize(Eigen::Vector2i(w,h));
		}

		/// Draw to a new window. Init pixmap with contents of a NRRD::Image. If _window is null it will be allocated.
		Figure(const std::string& _name, const NRRD::ImageView<float>& img, double bias=0, double scale=0, bool is_signed=false)
			: name(_name)
		{
			if (!FigureWindow::exists(name,false))
				FigureWindow::instance_delete(name);
			auto& figure=FigureWindow::instance(name);
			figure.overlay.clear();
			figure.setImage(img, bias, scale, is_signed);
		}

		/// Check if a figure of said name exists at all (and whether it is also visible)
		bool exists(bool must_be_visible=false) {
			return  FigureWindow::exists(name, must_be_visible);
		}

		/// Set the projection matix used for displaying 3D content.
		Figure& setProjectionMatrix(const Geometry::ProjectionMatrix& P) {
			FigureWindow::instance(name).setProjectionMatrix(P);
			return *this;
		}

		/// Get current projection matrix.
		Geometry::ProjectionMatrix getProjectionMatrix() {
			if (!exists()) return Geometry::ProjectionMatrix::Zero();
			return FigureWindow::instance(name).getProjectionMatrix();
		}
		
		/// Access projection parameters
		ProjectionParameters& projection_parameters()
		{
			return FigureWindow::instance(name).projection_parmeters();
		}

		/// Rotate object to point z-axis downward or upward
		Figure& z_vertical(bool points_up=false)
		{
			auto angle=FigureWindow::instance(name).projection_parmeters().angle;
			if (points_up) angle[2]=-1.570796;
			else           angle[2]=+1.570796;
			FigureWindow::instance(name).projection_parmeters().setAngle(angle);
			return *this;
		}

		/// Utility function to choose an angle of observation
		Figure& rotate(double _long, double _lat)
		{
			auto angle=FigureWindow::instance(name).projection_parmeters().angle;
			angle[0]=_lat;
			angle[1]=_long;
			FigureWindow::instance(name).projection_parmeters().setAngle(angle);
			return *this;
		}

		/// Get size of the figure (which is different from its window size)
		Eigen::Vector2i getSize() {
			if (!exists()) return Eigen::Vector2i(0,0);
			return FigureWindow::instance(name).getVisibleSize();
		}
		
		/// Set size of the figure (which is different from its window size)
		Figure& setSize(const Eigen::Vector2i& size) {
			FigureWindow::instance(name).setVisibleSize(size);
			return *this;
		}

		/// Access the image source of this figure. May be empty for 3D visualizations.
		NRRD::ImageView<float> getImage() {
			if (!exists()) return NRRD::ImageView<float>();
			return  FigureWindow::instance(name).getImage();
		};

		/// Access the image source of this figure. May be empty for 3D visualizations.
		Figure& setImage(const NRRD::ImageView<float>& img, double bias=0, double scale=0, bool is_signed=false) {
			FigureWindow::instance(name).setImage(img, bias, scale, is_signed);
			return *this;
		};

		/// A callback when mouse is clicked in select mode.
		Figure& setCallback(void (*select)(const std::string&, bool , std::vector<Eigen::Vector4d>&)) {
			FigureWindow::instance(name).setCallback(select);
			return *this;
		}

		/// Access selection of user.
		std::vector<Eigen::Vector4d> getSelection(bool blue=false)
		{
			return FigureWindow::instance(name).getSelection(blue);
		}

		/// Access selection of user.
		Figure& setSelection(const std::vector<Eigen::Vector4d>& selection, bool blue)
		{
			FigureWindow::instance(name).setSelection(selection, blue);
			return *this;
		}

		/// Clear user selection
		Figure& clearSelection(bool blue) {
			return setSelection(std::vector<Eigen::Vector4d>(),blue);
		}
		
		/// Save figure to file.
		Figure& savePNG(const std::string& png_file) {
			FigureWindow::instance(name).savePNG(png_file);
			return *this;
		}

		/// Save figure to file.
		Figure& savePDF(const std::string& pdf_file) {
			FigureWindow::instance(name).savePDF( pdf_file);
			return *this;
		}

		/// Close and delete window.
		void close() {
			FigureWindow::instance_delete(name);
			name.clear();
		}

		/// Access to the little progressbar in the FigureWindow
		GetSetGui::ProgressInterface& progress() {
			auto& figure=FigureWindow::instance(name);
			figure.setPlainLook(false);
			return figure;
		}

		/// Access to the overlay of the image. See also: draw(...) drawPoint(...) drawLine(...) drawText(...)
		GraphicsItems::Group& overlay() {
			return FigureWindow::instance(name).overlay;
		}

		/// Add a GraphicsItem to the image overlay
		Figure& draw(const GraphicsItem& item, const std::string& item_name_prefix="Item") {
			FigureWindow::instance(name).overlay.add(item,item_name_prefix);
			return *this;
		}

		/// Draw colored text starting at pixel x,y
		Figure& drawText(const std::string& str, int x=0, int y=0, const QColor& color=QColor(255,255,255)) {
			return draw(GraphicsItems::Text2D(str,x,y,color),"Text2D ");
		}

		/// Draw a colored line
		Figure& drawLine(int x1, int y1, int x2, int y2, const QColor& color=QColor(0,0,255), int thickness=1) {
			return draw(GraphicsItems::Line2D(x1,y1,x2,y2,thickness,color),"Line2D ");
		}

		/// Draw a colored dot
		Figure& drawPoint(int x, int y, const QColor& color=QColor(255,0,0), double radius=1.0f, GraphicsItems::MarkerShape marker=GraphicsItems::MarkerShape::Circle) {
			return draw(GraphicsItems::Point2D(x,y, radius, color, marker ),"Point2D ");
		}

		/// Display temporary information
		Figure& info(const std::string& user_info_text) {
			auto& figure=FigureWindow::instance(name);
			figure.info(user_info_text);
			return *this;
		}

		/// Display a figure as part of a tiled set of Figures
		Figure& showTiled(int index, int w=0, int h=0)
		{
			int space_w=6;
			int space_h=38;
			auto& figure=FigureWindow::instance(name);
			if (w==0) w=figure.window->width();
			if (h==0) h=figure.window->height();
			QRect rect=QApplication::desktop()->availableGeometry(figure.window);
			int num_columns=rect.width()/(w+space_w);
			int index_column=index%num_columns;
			int index_row=index/num_columns;
			int win_x=rect.x()+(w+space_w)*index_column;
			int win_y=rect.y()+(h+space_h)*index_row;
			figure.window->move(win_x,win_y);
			figure.setPlainLook();
			figure.setVisibleSize(Eigen::Vector2i(w,h));
			figure.setAutoResize();
			return *this;
		}
		
		/// Display image size and window. (by default displays image size and intensity bians/scale)
		Figure& info_intensity(double bias, double scale)
		{
			double min_intensity=bias, max_intensity=1.0/scale-bias;
			double mean_intensity=(min_intensity+max_intensity)/2, half_width=(max_intensity-min_intensity)/2;
			auto& figure=FigureWindow::instance(name);
			std::string info_text=toString(figure.pixmap.width())+" x "+toString(figure.pixmap.height())+" px  / ";
			if      (min_intensity==0)  info_text+="Intensity maximum: " + toString (max_intensity);
			else if (mean_intensity==0) info_text+="Intensity range: ± " + toString (half_width);
			else                        info_text+="Intensity window: " +toString(mean_intensity) + " ± " +toString(half_width);
			return info(info_text);
		}

		/// Immediate on-screen update (slow)
		Figure& update()
		{
			if (exists()) FigureWindow::instance(name).update();
			QCoreApplication::processEvents();
			return *this;
		}

		/// Trigger one last update before destruction. Window will live on until it is closed.
		~Figure() {
			if (exists()) FigureWindow::instance(name).update();
			QCoreApplication::processEvents();
		}

	};

}

#endif // __qt_nrrd
