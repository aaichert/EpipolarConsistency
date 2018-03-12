#ifndef __FIGURE_WINDOW_H
#define __FIGURE_WINDOW_H

#include <QMainWindow>
#include <QPixmap>

#include <NRRD/nrrd_image.hxx>
#include <GetSet/GetSet.hxx>

#include "EventFilters.h"

#include "ProjectionParameters.hxx"

#include "GraphicsItems.hxx"
#include "GraphicsItemConvexMesh.hxx"

// Predcl of Qt types
class QLabel;
class QComboBox;
class QScrollArea;
class QProgressBar;
class QPushButton;

namespace GetSetGui {
	class GetSetTabWidget;
} // namespace GetSetGui 

namespace UtilsQt {

	/// A window which displays a pixmap in a label FIXME should derive from QMainWindow
	class FigureWindow : public GetSetGui::ProgressInterface {
		//
		// Qt objects
		//
		// FIXME? allow access to window...

	public:

		QMainWindow                *window;

	protected:
		EventFilterMouse           *handler_mouse;
		EventFilterWindow          *handler_window;

		GetSetGui::GetSetTabWidget *advanced_settings;

		QScrollArea                *scroll_area;
		QLabel                     *label;
		QComboBox                  *tool;
		QProgressBar               *status_progress;
		QPushButton                *status_cancel;
		QLabel                     *status_cursor_pos;
		QLabel                     *status_info;

		//
		// Advanced user input
		//

		/// A dictionary with advanced options. (FIXME: mutable - conceptual problem with informative gui entries)
		mutable GetSetInternal::Dictionary dictionary;
		mutable GetSetHandler              callback;
		mutable ProjectionParametersGui*   projection;

		/// The projection matrix
		Geometry::ProjectionMatrix P;
		
		/// The NRRD::Image (optional) see also: updatePixmap();
		NRRD::Image<float>         image_source;

		/// Handle events sent from advanced options via GetSet
		void notify(const GetSetInternal::Node& node);

		//
		// Event handling
		//

		void file_save_as();
		void file_export();
		void file_close();
		void edit_copy();
		void edit_zoom_in();
		void edit_zoom_out();
		void edit_editplots();

		void show_advanced();
		
		void mouse_move(int x, int y, int dx, int dy);
		void mouse_drag(int x, int y, int dx, int dy);
		void mouse_click(int x, int y, bool left_button, bool release);
		void mouse_double_click(int x, int y, bool left_button);
		void mouse_wheel(int dx, int dy, bool shift, bool ctrl);

		// For selection
		void mouse_drag_select(int x, int y, int dx, int dy);
		void mouse_click_select(int x, int y, bool left_button, bool release);

		// For Line Plots
		void mouse_drag_plot(int x, int y, int dx, int dy);
		void mouse_click_plot(int x, int y, bool left_button, bool release);

		// Auto adjust magnification and, if no image is given, resize pixmap.
		void resize_window(int w, int h);
		
		///  update Meta info in GUI
		void updateMetaInfo();


		/// Callback mechanism when selection changes
		void (*callback_select)(const std::string&, bool , std::vector<Eigen::Vector4d>&);

		//
		/// Access to a database of Figure windows
		//

		// Map of all named instances
		static std::map<std::string, FigureWindow*>& instances();

		/// make not copyable (access through Figure(...), see Figure.hxx )
		FigureWindow(const FigureWindow&);
		

		//
		// ProgressInterface implementation
		//

		bool         *progress_cancel_clicked;
		virtual void progressStart(const std::string& progress, const std::string& info, int maximum, bool *cancel_clicked);
		virtual void progressUpdate(int i);
		virtual void progressEnd();
		virtual void info(const std::string& who, const std::string& what, bool show_dialog);
		virtual void warn(const std::string& who, const std::string& what, bool only_inormative);

	public:
		const                      std::string name; //< Name of this instance (or empty string if not handled)
		QPixmap                    pixmap;			 //< The current pixel representation, not including overlay.
		GraphicsItems::Group       overlay;			 //< The GraphicsItems drawn on top of pixmap
		QColor                     background_color; //< The background color, important if no image is set. See also: 

		FigureWindow(const std::string& name, bool tool3d=false);
		~FigureWindow();

		/// Set a fixed projection matrix for this view.
		void setProjectionMatrix(const Geometry::ProjectionMatrix& P);

		/// Get the current prijection matrix. (either fixed or manual user)
		Geometry::ProjectionMatrix getProjectionMatrix() const;

		/// Returns user projection parameters or zero for a fixed Projection Matrix.
		ProjectionParameters& projection_parmeters();

		/// Get current size of the figure (which is different from the window size)
		Eigen::Vector2i getVisibleSize() const;

		/// Set current size of the figure (which is different from the window size)
		void setVisibleSize(const Eigen::Vector2i& size);

		/// Choose between image and 3D navigation toolsets
		void setToolsetImage(bool tool_image);

		/// Get current selection.
		std::vector<Eigen::Vector4d> getSelection(bool blue);

		/// Set selection
		void setSelection(std::vector<Eigen::Vector4d> selection, bool blue);

		// Callback
		void setCallback(void (*callback_select)(const std::string&, bool , std::vector<Eigen::Vector4d>&));

		/// Set a NRRD::Image
		void setImage(const NRRD::ImageView<float>& img, double bias=0, double scale=0, bool is_signed=0);

		/// Get the NRRD::Image
		NRRD::ImageView<float> getImage() const;

		/// Get current slice (for 3D volumes)
		NRRD::ImageView<float> getCurrentSlice() const;

		/// If image_source is provided, get a contrast enhanced view of that, otherwise, just show background_color
		void updatePixmap();

		/// Save image and overlay as rasterized image
		bool savePNG(const std::string& filename);

		/// Save image and overlay in PDF file
		bool savePDF(const std::string& filename);

		/// Scale image and render overlay to raster image
		QPixmap render(double _magnification=1.0);
		
		/// Display some info in the status bar
		void info(const std::string& info_text);

		/// Force an immediate update of the overlay
		void update();

		/// Automatically fits image to the window size.
		void setAutoResize(bool auto_resize=true);

		/// Show/Hide advanced features if the figure window (notably, menu and status bar).
		void setPlainLook(bool on=true);

		//
		/// Access to a database of Figure windows
		//

		/// Close and delete a plot window
		static void instance_delete(const std::string& name);

		/// Access (or create) an instance with name.
		static FigureWindow& instance(const std::string& name = "");

		/// Access to any existing Figure by letting the user select from a list. Optionally request a certain state.
		static std::set<std::string> instance_names(bool must_have_image=false, bool must_have_projection_matrix=false, bool must_be_visible=false);

		/// Check if a window of that name exists (and is visble)
		static bool exists(const std::string& name, bool must_be_visible=true);
		
		/// Close and delete all FigureWindows
		static void instance_delete_all();
	};



} // namespace UtilsQt

#endif // __FIGURE_WINDOW_H
