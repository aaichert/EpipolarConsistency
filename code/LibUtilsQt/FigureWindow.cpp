#include "FigureWindow.h"

#include "Plot.hxx"

#include "nrrdqt.hxx"

#include <QApplication>
#include <QTransform>
#include <QPicture>
#include <QPrinter>
#include <QScrollArea>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QClipboard>
#include <QStatusBar>
#include <QMouseEvent>
#include <QFileDialog>
#include <QLabel>
#include <QComboBox>
#include <QScrollArea>
#include <QScrollBar>
#include <QPushButton>
#include <QProgressBar>
#include <QMessageBox>

#include <functional>
#include <cmath>

#include <GetSetGui/GetSetTabWidget.h>
#include <GetSetGui/GetSetModalDialog.h>

// Global place for plots...
namespace UtilsQt
{
	/// Access to a database of plot windows
	inline std::map<std::string, QCustomPlot*>& plotsByName()
	{
		static std::map<std::string, QCustomPlot*> plots;
		return plots;
	}
} // namespace UtilsQt

namespace UtilsQt {
	
	//
	// drawing
	//

	QPixmap FigureWindow::render(double _magnification)
	{
		if (pixmap.width()<=0 || pixmap.height()<=0)
		{
			callback.ignoreNotifications(true);
			if (!image_source) {
				auto max_wh=getVisibleSize();
				GetSet<Eigen::Vector2i>("General/Figure Size",dictionary)=max_wh;
				pixmap=QPixmap(max_wh[0],max_wh[1]);
				pixmap.fill(background_color);
				GetSet<double>("General/Magnification/Factor",dictionary)=1.0;
			} else updatePixmap();
			callback.ignoreNotifications(false);
		}
		if (pixmap.width()<=0 || pixmap.height()<=0) return QPixmap();
		bool interpolate=GetSet<bool>("General/Magnification/Interpolation",dictionary);
		QPixmap display=pixmap.scaledToWidth(pixmap.width()*_magnification ,interpolate?Qt::SmoothTransformation:Qt::FastTransformation);
		QPainter p(&display);
		overlay.draw(p,getProjectionMatrix(),_magnification);
		if (!!image_source && GetSet<bool>("Selections/Show",dictionary))
		{
			auto rb=GetSet<std::vector<Eigen::Vector4d> >("Selections/Blue"  ,dictionary).getValue();
			auto ry=GetSet<std::vector<Eigen::Vector4d> >("Selections/Yellow",dictionary).getValue();
			for (auto it=rb.begin();it!=rb.end();++it)
				if ((*it)[2]==1&&(*it)[3]==1) GraphicsItems::Point2D((*it)[0],(*it)[1],5,QColor(128,128,255),GraphicsItems::MarkerShape::Cross).draw(p,P,_magnification);
				else                          GraphicsItems::Rectangle((*it)[0],(*it)[1],(*it)[2],(*it)[3],QColor(128,128,255,96)).draw(p,P,_magnification);
			for (auto it=ry.begin();it!=ry.end();++it)
				if ((*it)[2]==1&&(*it)[3]==1) GraphicsItems::Point2D((*it)[0],(*it)[1],5,QColor(255,255,0),GraphicsItems::MarkerShape::Circle).draw(p,P,_magnification);
				else                          GraphicsItems::Rectangle((*it)[0],(*it)[1],(*it)[2],(*it)[3],QColor(255,255,0,64)).draw(p,P,_magnification);
		}
		p.end();
		return display;
	}
	
	void FigureWindow::updateMetaInfo()
	{
		if (!image_source) { 
			GetSetGui::Section("Meta Data",dictionary).discard();
			return;
		}
		callback.ignoreNotifications(true);
		GetSetGui::Section("Meta Data/Keys",dictionary).discard();
		GetSetGui::Button("Meta Data/Edit/Add Key",dictionary)="Add Key...";
		GetSetGui::Button("Meta Data/Edit/Clear Keys",dictionary)="Erase all keys...";
		GetSetGui::Button("Meta Data/Edit/Erase Key",dictionary)="Erase Key...";		
		GetSetGui::Section("Meta Data/Edit",dictionary).setGrouped();
		GetSetGui::Section keys("Meta Data/Keys",dictionary);
		for (auto it=image_source.meta_info.begin();it!=image_source.meta_info.end();++it)
		{
			GetSet<> key(it->first,keys);
			key=it->second;
			// Group first level of sections.
			auto one_up=key.supersection();
			auto two_up=one_up.supersection();
			if (two_up.path()==keys.path()) one_up.setGrouped();
		}
		callback.ignoreNotifications(false);
	}

	void FigureWindow::setImage(const NRRD::ImageView<float>& img, double bias, double _scale, bool is_signed)
	{
		double scale=(_scale==0?NRRD::nrrdComputeBiasAndScale<float>(img, bias, _scale):_scale);
		callback.ignoreNotifications(true);
		if (is_signed)
		{
			GetSet<double>("Intensity/Scale",dictionary)=scale;
			GetSet<double>("Intensity/Bias",dictionary)=bias+0.5/scale;
		}
		else
		{
			GetSet<double>("Intensity/Scale",dictionary)=scale;
			GetSet<double>("Intensity/Bias",dictionary)=0;
		}
		GetSetGui::Button("Intensity/Auto",dictionary)="Auto-Adjust";
		GetSetGui::Section("Intensity",dictionary).setGrouped();
		setToolsetImage(true);
		callback.ignoreNotifications(false);
		image_source.clone(img);
		updatePixmap();
		GetSet<bool>("General/Magnification/Interpolation",dictionary)=true;
		if (pixmap.width()<=300 && pixmap.height()<=300) {
			GetSet<bool>("General/Magnification/Interpolation",dictionary)=false;
			GetSet<bool>("General/Magnification/Auto",dictionary)=true;
			setVisibleSize(Eigen::Vector2i(512,512.0/pixmap.width()*pixmap.height()+1));
			setPlainLook(false);
		}
		else if (pixmap.width()<=1024 && pixmap.height()<=1024)
		{
			if (image_source.isElementKindColor()
			||  image_source.dimension()==2)
				 setPlainLook(true);
			else setPlainLook(false);
			setVisibleSize(Eigen::Vector2i(pixmap.width(),pixmap.height()));
		}
		else
		{
			GetSet<bool>("General/Magnification/Auto",dictionary)=true;
			double mag=1024.0/std::max(pixmap.width(),pixmap.height());
			setVisibleSize(Eigen::Vector2i(pixmap.width()*mag,pixmap.height()*mag));
			// window->setWindowState(Qt::WindowMaximized);
		}
		if (image_source.dimension()>2 && !image_source.isElementKindColor())
			GetSet<int>("Intensity/Slice Index",dictionary)=image_source.size(2)/2;
		if (image_source.meta_info.find("Projection Matrix")!=image_source.meta_info.end())
			setProjectionMatrix(stringTo<Geometry::ProjectionMatrix>(image_source.meta_info["Projection Matrix"]));
		updateMetaInfo();
		// Update
		update();
	}
	
	NRRD::ImageView<float> FigureWindow::getImage() const { return image_source; }

	NRRD::ImageView<float> FigureWindow::getCurrentSlice() const {
		if (!image_source) return image_source;
		int max_slice=image_source.dimension()<=2?0:image_source.size(2)-1;
		if (max_slice==1) return image_source;
		int slice_index=GetSet<int>("Intensity/Slice Index",dictionary);
		if (slice_index>max_slice) slice_index=max_slice;
		if (slice_index<0)         slice_index=0;
		return NRRD::ImageView<float>(image_source,slice_index);
	}

	void FigureWindow::updatePixmap()
	{
		if (!!image_source)
		{
			double scale=GetSet<double>("Intensity/Scale",dictionary);
			double bias =GetSet<double>("Intensity/Bias" ,dictionary);
			if (image_source.isElementKindColor()) {
				pixmap=NRRD::nrrdToQPixmap(image_source,bias,scale);
				update();
				return;
			}
			int max_slice=image_source.dimension()<=2?0:image_source.size(2)-1;
			GetSet<int> slice_index("Intensity/Slice Index",dictionary);
			if (slice_index>max_slice) { slice_index=max_slice; return; }
			if (slice_index<0)         { slice_index=0;         return; }
			NRRD::ImageView<float> slice(image_source,slice_index);
			if (slice_index>0)
				status_cursor_pos->setText((std::string("z = ")+slice_index.getString()+" / " + toString(max_slice+1)).c_str());
			if (scale==0) {
				callback.ignoreNotifications(true);
				NRRD::nrrdComputeBiasAndScale(slice,bias,scale);
				GetSet<double>("Intensity/Scale",dictionary)=scale<=0?1:scale;
				GetSet<double>("Intensity/Bias" ,dictionary)=bias;
				callback.ignoreNotifications(false);
			}
			double min_intensity=bias, max_intensity=1.0/scale-bias;
			double mean_intensity=(min_intensity+max_intensity)/2, half_width=(max_intensity-min_intensity)/2;
			callback.ignoreNotifications(true);
			GetSetGui::StaticText("Intensity/_info",dictionary)="Visible Intensity window:  " +toString(mean_intensity) + " +/- " +toString(half_width);;
			callback.ignoreNotifications(false);
			pixmap=NRRD::nrrdToQPixmap(slice,bias,scale);
			update();
		}
	}

	//
	// Advanced user input
	//

	void FigureWindow::notify(const GetSetInternal::Node& node)
	{
		if (!window) return;

		if (node.super_section=="Projection")
			update();
		else if (node.super_section=="General") {
			if (node.name=="Projection Matrix") {
				P=GetSet<Geometry::ProjectionMatrix>("General/Projection Matrix",dictionary);
				GetSet<bool>("Projection/Manual Projection",dictionary)=false;
			}
			if (node.name=="Figure Size")
				setVisibleSize(GetSet<Eigen::Vector2i>("General/Figure Size",dictionary));
		}
		else if (node.super_section=="Intensity") {
			if (node.name=="Auto")
				GetSet<double>("Intensity/Scale",dictionary)=0;
			else updatePixmap();
		}
		else if (node.super_section=="General/Magnification")
		{
			if (node.name=="Factor")
				update();
			else if (node.name=="Auto") // just update magnification
				resize_window(window->width(),window->height());
		}
		else if (node.super_section=="Selections")
		{
			if (node.name=="Clear") {
				GetSet<std::vector<Eigen::Vector4d> >("Selections/Blue"  ,dictionary).setString("");
				GetSet<std::vector<Eigen::Vector4d> >("Selections/Yellow",dictionary).setString("");
			}
			update();
		}
		else if (hasPrefix(node.super_section,"Meta Data/Keys"))
		{
			std::string key=
			image_source.meta_info[node.name]=node.getString();
		}
		else if (node.name=="Erase Key")
		{
			std::vector<std::string> all_keys;
			for (auto it=image_source.meta_info.begin();it!=image_source.meta_info.end();++it)
				all_keys.push_back(it->first);
			GetSetGui::GetSetModalDialog which_key;
			GetSetGui::Enum("Key",which_key).setChoices(all_keys);
			if (which_key.exec()) {
				auto key=image_source.meta_info.find(GetSet<>("Key",which_key));
				if (key!=image_source.meta_info.end()) image_source.meta_info.erase(key);
				updateMetaInfo();
			}
		}
		else if (node.name=="Add Key")
		{
			GetSetGui::GetSetModalDialog new_key;
			GetSet<>("Key"  ,new_key);
			GetSet<>("Value",new_key);
			if (new_key.exec() && !GetSet<>("Key"  ,new_key).getString().empty()) {
				image_source.meta_info[GetSet<>("Key" ,new_key)]=GetSet<>("Value",new_key);
				updateMetaInfo();
			}
		}
		else if (node.name=="Clear Keys")
		{
			image_source.meta_info.clear();
			updateMetaInfo();
		}
	}
	
	//
	// Event handling
	//

	void FigureWindow::file_save_as()
	{
		if (!image_source)
		{
			// if we do not have an image, exporting and saving is considered the same thing
			file_export();
			return;
		}
		std::string path=QFileDialog::getSaveFileName(0x0, "Save as NRRD",(name+".nrrd").c_str(), "NRRD Image (*.nrrd);;All Files (*)").toStdString();
		if (!path.empty())
			if (!image_source.save(path))
				QMessageBox::warning(window,"Faled to save NRRD file.",(std::string("File ")+path+"could not  be written!").c_str(),QMessageBox::StandardButton::Ok );
	}

	void FigureWindow::file_export()
	{
		std::string path=QFileDialog::getSaveFileName(0x0, "Export...",(name+".pdf").c_str(), "Portable Document Format (*.pdf);;Portable Network Graphics (*.png);;All Files (*)").toStdString();
		if (path.empty()) return;
		std::string extension=path;
		extension=splitRight(extension,".");
		if (extension=="png")
			savePNG(path);
		else
			savePDF(path);
	}

	void FigureWindow::file_close()     { window->close(); }
	void FigureWindow::edit_copy()      { QApplication::clipboard()->setImage(render().toImage(), QClipboard::Clipboard); }
	void FigureWindow::edit_zoom_in()   { GetSet<double> magnification("General/Magnification/Factor",dictionary); double m=((int)(magnification*10))*0.1+.2; magnification=m>4?4:m; update(); }
	void FigureWindow::edit_zoom_out()  { GetSet<double> magnification("General/Magnification/Factor",dictionary); double m=((int)(magnification*10))*0.1-.2; magnification=m>4?4:m; update(); }
	void FigureWindow::edit_editplots() { UtilsQt::showPlotEditor(); }
	
	void FigureWindow::show_advanced()
	{
		if (!advanced_settings)
		{
			advanced_settings=new GetSetGui::GetSetTabWidget(window,dictionary);
			advanced_settings->setWindowFlags(Qt::Tool);
			advanced_settings->setWindowTitle((name + " - Figure Settings").c_str());
			QObject::connect(handler_window, &EventFilterWindow::closed,  std::bind(&QWidget::close, advanced_settings));
		}
		advanced_settings->show();
		advanced_settings->raise();
		advanced_settings->activateWindow();
	}

	void FigureWindow::mouse_move(int x, int y, int dx, int dy)
	{
		double magnification=GetSet<double>("General/Magnification/Factor",dictionary);
		int xpx=(int)std::round((double)x/magnification);
		int ypx=(int)std::round((double)y/magnification);
		status_cursor_pos->setText((toString(xpx)+" "+toString(ypx)
			+(image_source.dimension()>2?" "+GetSet<int>("Intensity/Slice Index",dictionary).getString():"")).c_str());
		if (!image_source) return;
		if (image_source.isElementKindColor())
		{
			if (xpx>=0 && xpx<image_source.size(1) && ypx>=0 && ypx<image_source.size(2) )
			{
				std::string color;
				for (int c=0;c<image_source.size(0);c++)
					color+=toString(image_source.pixel(c,xpx,ypx))+" ";
				status_info->setText(color.c_str());
			}
		}
		else
		{
			auto slice=getCurrentSlice();
			if (!!image_source && xpx>=0 && xpx<slice.size(0) && ypx>=0 && ypx<slice.size(1) )
				status_info->setText(toString(slice.pixel(xpx,ypx)).c_str());
		}
	}

	void FigureWindow::mouse_drag(int x, int y, int dx, int dy)
	{
		scroll_area->ensureVisible(x,y);
		if      (tool->currentText()=="Navigate")  ;
		else if (tool->currentText()=="Select")    mouse_drag_select  (x,y,dx,dy);
		else if (tool->currentText()=="Plot")      mouse_drag_plot    (x,y,dx,dy);
		else if (tool->currentText()=="Rotate 3D") {
			GetSet<bool> manual("Projection/Manual Projection",dictionary);
			if (!manual) manual=true;
			projection->mouse(dx,dy,handler_mouse->is_left_button());
		}
	}

	void FigureWindow::mouse_click(int x, int y, bool left_button, bool release)
	{
		if (release) info("");
		if (tool->currentText()=="Select")    mouse_click_select(x,y,left_button,release);
		else if (tool->currentText()=="Plot") mouse_click_plot(x,y,left_button,release);
	}

	void FigureWindow::mouse_double_click(int x, int y, bool left_button)
	{
		setPlainLook(window->menuBar()->isVisible());
	}

	void FigureWindow::mouse_wheel(int dx, int dy, bool shift, bool ctrl)
	{
		if      (!ctrl && !shift && image_source.dimension()>2)
		{
			GetSet<int> slice_index("Intensity/Slice Index",dictionary);
			slice_index=slice_index+(dy>0?1:-1);
		}
		else if (ctrl && dy>0) edit_zoom_in();
		else if (ctrl && dy<0) edit_zoom_out();
		else if (tool->currentText()=="Rotate 3D" && !shift && !ctrl) projection->wheel(dy);
	}

	void FigureWindow::mouse_drag_select(int x, int y, int dx, int dy)
	{
		GraphicsItems::Rectangle *rect=overlay.get<GraphicsItems::Rectangle>("Rect Selection");
		if (!rect) return;
		double magnification=GetSet<double>("General/Magnification/Factor",dictionary);
		rect->size[0]=x/magnification -rect->x[0];
		rect->size[1]=y/magnification -rect->x[1];
		info(toString(x)+" "+toString(y) + " ("+toString(rect->size[0])+" x "+toString(rect->size[1])+")");
		update();
	}

	void FigureWindow::mouse_click_select(int x, int y, bool left_button, bool release)
	{
		if (release) {
			GraphicsItems::Rectangle *rect=overlay.get<GraphicsItems::Rectangle>("Rect Selection");
			if (!rect) return;
			Eigen::Vector4d selection(rect->x[0], rect->x[1], rect->size[0], rect->size[1]);
			bool is_blue=(rect->color.blue()!=0);
			auto new_selections=getSelection(is_blue);
			new_selections.push_back(selection);
			setSelection(new_selections,is_blue);
			overlay.erase("Rect Selection");
			mouse_move(x,y,0,0);
		}
		else {
			double magnification=GetSet<double>("General/Magnification/Factor",dictionary);
			overlay.set("Rect Selection",
				GraphicsItems::Rectangle(x/magnification,y/magnification,1,1,left_button?QColor(255,255,0,64):QColor(128,128,255,96)));
		}
		update();
	}

	/// Determine positive muliplier s, so that p+ds is integer
	inline double stride_fraction(const double& p, const double& d)
	{
		const double tolerance=1E-5; // make sure we err on the right side of things
		double frac=p-(int)p; // fractional part of p
		if (d>tolerance) // positive direction?
		{
			if (1.0-frac<tolerance) frac=0;
			return (1.0-frac)/d + tolerance;
		}
		else if (d<-tolerance) // negaive direction?
		{
			if (frac<tolerance) frac=1;
			return -frac/d + tolerance;
		}
		else return 1.0/tolerance; // zero? just return something big...
	}

	void FigureWindow::mouse_drag_plot(int x, int y, int dx, int dy)
	{
		GraphicsItems::Arrow2D *line=overlay.get<GraphicsItems::Arrow2D>("Line Selection");
		if (!line) return;
		double magnification=GetSet<double>("General/Magnification/Factor",dictionary);
		Eigen::Vector2d from (line->o[0], line->o[1]);
		Eigen::Vector2d to   (x/magnification,y/magnification);
		Eigen::Vector2d delta=to-from;
		line->x[0]=to[0];
		line->x[1]=to[1];
		double length=delta.norm();
		delta/=length;
		info(toString(delta)+" ("+toString<int>(length)+" px)");
		if (!!image_source && length>=5)
		{
			if (image_source.isElementKindColor())
			{
				std::vector<double> location;
				std::vector<std::vector<double>> samples(image_source.size(0));
				for (double s=0;s<=length;) {
					Eigen::Vector2d sample=from+s*delta;
					location.push_back(s);
					for (int c=0;c<image_source.size(0);c++)
						samples[c].push_back(image_source((double)c,sample[0],sample[1]));
					double s0=stride_fraction(sample[0],delta[0]) ;
					double s1=stride_fraction(sample[1],delta[1]) ;
					s+=std::min(s0,s1);
				}
				Plot plot(name + " - Intensity Plot");
				for (int c=0;c<image_source.size(0);c++)
					plot.graph(c).setData(location,samples[c]);
				plot.no_update();
			}
			else
			{
				auto slice=getCurrentSlice();
				std::vector<double> samples,location;
				for (double s=0;s<=length;) {
					Eigen::Vector2d sample=from+s*delta;
					location.push_back(s);
					samples.push_back(slice(sample[0],sample[1]));
					double s0=stride_fraction(sample[0],delta[0]) ;
					double s1=stride_fraction(sample[1],delta[1]) ;
					s+=std::min(s0,s1);
				}
				Plot plot(name + " - Intensity Plot");
				plot.graph(0).setData(location,samples);
				plot.no_update();
			}
		}
		update();
	}

	void FigureWindow::mouse_click_plot(int x, int y, bool left_button, bool release)
	{
		if (release) {
			overlay.erase("Line Selection");
			mouse_move(x,y,0,0);
			label->setCursor(Qt::CrossCursor);
		}
		else {
			Plot plot(name + " - Intensity Plot");
			plot.no_show();
			plot.setAxisLabels("Distance [px]","Intensity");
			if (image_source.isElementKindColor())
			{
				for (int c=0;c<image_source.size(0);c++)
						 if (c==0) plot.graph(0).setColor(QColor(255,0,0,255)).setName("red");
					else if (c==1) plot.graph(1).setColor(QColor(0,255,0,255)).setName("green");
					else if (c==2) plot.graph(2).setColor(QColor(0,0,255,255)).setName("blue");
					else if (c==3) plot.graph(3).setColor(QColor(0,0,0,128))  .setName("alpha");
					else           plot.graph(c).setColor(QColor(0,0,0,128))  .setName(std::string("channel ")+toString(c));
				plot.showLegend();
			}
			label->setCursor(Qt::CrossCursor); // should be be blank
			double magnification=GetSet<double>("General/Magnification/Factor",dictionary);
			overlay.set("Line Selection",
				GraphicsItems::Arrow2D(
					Eigen::Vector2d(x/magnification,y/magnification),
					Eigen::Vector2d(x/magnification,y/magnification),
					QColor(255,255,0),
					2/magnification,5/magnification
					));
		}
		update();
	}

	void FigureWindow::resize_window(int w, int h)
	{
		// Invlidate Pixmap
		pixmap=QPixmap();
		// Optionally adjust magnification
		if (!!image_source && GetSet<bool>("General/Magnification/Auto",dictionary))
		{
			auto max_wh=getVisibleSize();
			callback.ignoreNotifications(true);
			if (image_source.isElementKindColor())
				GetSet<double>("General/Magnification/Factor",dictionary)=std::min((double)max_wh[0]/image_source.size(1),(double)max_wh[1]/image_source.size(1));
			else
				GetSet<double>("General/Magnification/Factor",dictionary)=std::min((double)max_wh[0]/image_source.size(0),(double)max_wh[1]/image_source.size(1));
			callback.ignoreNotifications(false);
		}
		update();
	}

	//
	// c-tor
	//

	FigureWindow::FigureWindow(const std::string& _name, bool tool3d)
		: GetSetGui::ProgressInterface()
		, dictionary()
		, window(0x0)
		, advanced_settings(0x0)
		, projection(new ProjectionParametersGui(GetSetGui::Section("Projection", dictionary).setGrouped(),this))
		, callback(std::bind(&FigureWindow::notify, this, std::placeholders::_1),dictionary)
		, name(_name)
		, P(Geometry::ProjectionMatrix::Identity())
	{
		using namespace std::placeholders;

		callback.ignoreNotifications(true);
		GetSet<bool>("Projection/Manual Projection",dictionary)=true;
		GetSet<double>("General/Magnification/Factor",dictionary)=1;
		GetSet<bool>("General/Magnification/Auto",dictionary)=false;
		GetSet<bool>("General/Magnification/Interpolation",dictionary)=true;
		GetSetGui::Section("General/Magnification",dictionary).setGrouped();
		GetSetGui::Section("General",dictionary).setGrouped();

		// Keep track of all instances. FIXME may want to move this to the actual creation method to allow subclassing.
		instance_delete(name);
		instances()[name]=this;

		// Main window (which is parent to all other Qt objects in this class)
		window=new QMainWindow();
		window->setWindowTitle(name.c_str());
		handler_window=new EventFilterWindow(window);
		QObject::connect(handler_window, &EventFilterWindow::resize, std::bind(&FigureWindow::resize_window, this, _1, _2));

		// Actual image
		label=new QLabel(window);
		label->setAlignment(Qt::AlignCenter);
		scroll_area=new QScrollArea(window);
		scroll_area->setWidgetResizable(false);
		scroll_area->setWidget(label);
		window->setCentralWidget(scroll_area);

		// File menu
		QMenu* fileMenu=window->menuBar()->addMenu("&File");
		QAction* saveas = new QAction("&Save As...", window);
		saveas->setShortcuts(QKeySequence::Save);
		QObject::connect(saveas, &QAction::triggered, std::bind(&FigureWindow::file_save_as, this));
		fileMenu->addAction(saveas);			
		QAction* expt = new QAction("E&xport...", window);
		expt->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_X));
		QObject::connect(expt, &QAction::triggered, std::bind(&FigureWindow::file_export, this));
		fileMenu->addAction(expt);			
		fileMenu->addSeparator();
		auto* close = new QAction("&Close", window);
		close->setShortcuts(QKeySequence::Close);
		QObject::connect(close, &QAction::triggered, std::bind(&FigureWindow::file_close, this));
		fileMenu->addAction(close);
		// Edit menu
		QMenu* editMenu=window->menuBar()->addMenu("&Edit");
		QAction* copy = new QAction("&Copy", window);
		copy->setShortcuts(QKeySequence::Copy);
		QObject::connect(copy, &QAction::triggered, std::bind(&FigureWindow::edit_copy, this));
		editMenu->addAction(copy);
		editMenu->addSeparator();
		QAction* zoomin = new QAction("Zoom In", window);
		zoomin->setShortcuts(QKeySequence::ZoomIn);
		QObject::connect(zoomin, &QAction::triggered, std::bind(&FigureWindow::edit_zoom_in, this));
		editMenu->addAction(zoomin);
		QAction* zoomout = new QAction("Zoom Out", window);
		zoomout->setShortcuts(QKeySequence::ZoomOut);
		QObject::connect(zoomout, &QAction::triggered, std::bind(&FigureWindow::edit_zoom_out, this));
		editMenu->addAction(zoomout);
		editMenu->addSeparator();
		QAction* editplots = new QAction("Zoom Out", window);
		QObject::connect(editplots, &QAction::triggered, std::bind(&FigureWindow::edit_editplots, this));
		editMenu->addAction(editplots);

		// "..." button for advanced projection
		auto status_advanced=new QPushButton("...",window);
		status_advanced->setFixedWidth(40);
		window->statusBar()->addWidget(status_advanced);
		QObject::connect(status_advanced, &QPushButton::clicked,  std::bind(&FigureWindow::show_advanced,  this));

		// Tool
		tool=new QComboBox(window);
		setToolsetImage(false);
		tool->setMaximumHeight(20);
		window->statusBar()->addWidget(tool);

		// Progress bar
		status_progress=new QProgressBar(window);
		window->statusBar()->addWidget(status_progress);
		status_progress->setFixedWidth(150);
		status_progress->hide();
		status_cancel=new QPushButton("cancel",window);
		status_cancel->setFixedWidth(50);
		status_cancel->hide();
		window->statusBar()->addWidget(status_cancel);
		QObject::connect(status_cancel, &QPushButton::clicked,  std::bind(&FigureWindow::progressEnd,  this));

		// Mouse Interaction
		status_cursor_pos=new QLabel(window);
		status_cursor_pos->setFixedWidth(80);
		status_info=new QLabel(window);;
		window->statusBar()->addWidget(status_cursor_pos);
		window->statusBar()->addWidget(status_info, 1);
		handler_mouse=new EventFilterMouse(label);

		QObject::connect(handler_mouse, &EventFilterMouse::mouse_move,         std::bind(&FigureWindow::mouse_move,         this, _1, _2, _3, _4));
		QObject::connect(handler_mouse, &EventFilterMouse::mouse_drag,         std::bind(&FigureWindow::mouse_drag,         this, _1, _2, _3, _4));
		QObject::connect(handler_mouse, &EventFilterMouse::mouse_click,        std::bind(&FigureWindow::mouse_click,        this, _1, _2, _3, _4));
		QObject::connect(handler_mouse, &EventFilterMouse::mouse_wheel,        std::bind(&FigureWindow::mouse_wheel,        this, _1, _2, _3, _4));
		QObject::connect(handler_mouse, &EventFilterMouse::mouse_double_click, std::bind(&FigureWindow::mouse_double_click, this, _1, _2, _3));

		window->setMinimumHeight(150);
		window->setMinimumWidth(200);

		window->menuBar()->setVisible  (false);
		window->statusBar()->setVisible(false);

		// Start actual work
		callback.ignoreNotifications(false);
	}

	FigureWindow::~FigureWindow() {
		if (advanced_settings) delete advanced_settings;
		callback.attachTo(0x0);
		delete projection;
		delete window;
	}

	
	//
	// actual functionality
	//

	void FigureWindow::setToolsetImage(bool tool_image)
	{
		tool->clear();
		if (tool_image)
		{
			GetSetGui::Section("Projection", dictionary).setHidden(true);
			// tool->addItem(QIcon("./resources/move.svg"),        "Navigate");
			tool->addItem(QIcon("./resources/arrow_ne.svg"),    "Plot");
			tool->addItem(QIcon("./resources/select_rect.svg"), "Select");
			GetSet<std::vector<Eigen::Vector4d> >("Selections/Blue"  ,dictionary);
			GetSet<std::vector<Eigen::Vector4d> >("Selections/Yellow",dictionary);
			GetSetGui::Button("Selections/Clear",dictionary)="Clear Selections";
			GetSet<bool>("Selections/Show",dictionary)=true;
		}
		else {
			GetSetGui::Section("Selections"    ,dictionary).discard();
			GetSetGui::Section("Projection"    ,dictionary).setHidden(false);
			tool->addItem(QIcon("./resources/rotate.svg"),      "Rotate 3D");
		}
	}

	std::vector<Eigen::Vector4d> FigureWindow::getSelection(bool blue)
	{
		if (blue) return GetSet<std::vector<Eigen::Vector4d> >("Selections/Blue"  ,dictionary);
		else      return GetSet<std::vector<Eigen::Vector4d> >("Selections/Yellow",dictionary);
	}

	void FigureWindow::setSelection(std::vector<Eigen::Vector4d> selection, bool blue)
	{
		if (callback_select) callback_select(window->windowTitle().toStdString(),blue,selection);
		if (blue) GetSet<std::vector<Eigen::Vector4d> >("Selections/Blue"  ,dictionary)=selection;
		else      GetSet<std::vector<Eigen::Vector4d> >("Selections/Yellow",dictionary)=selection;
	}

	void FigureWindow::setCallback(void (*cb)(const std::string&, bool , std::vector<Eigen::Vector4d>&))
	{
		bool is_toolset_image=tool->itemText(0)!="Plot";
		callback_select=cb;
		setToolsetImage(is_toolset_image);
		if (cb)
		{
			if (!is_toolset_image)
			{
				tool->addItem(QIcon("./resources/select_rect.svg"), "Select");
				GetSet<std::vector<Eigen::Vector4d> >("Selections/Blue"  ,dictionary);
				GetSet<std::vector<Eigen::Vector4d> >("Selections/Yellow",dictionary);
				GetSetGui::Button("Selections/Clear",dictionary)="Clear Selections";
				GetSet<bool>("Selections/Show",dictionary)=true;
				tool->setCurrentIndex(tool->count()-1);
			}
			else
				tool->setCurrentIndex(1);
		}
	}

	void FigureWindow::setProjectionMatrix(const Geometry::ProjectionMatrix& _P)
	{
		GetSet<bool>("Projection/Manual Projection",dictionary)=false;
		P=_P;
	}

	Geometry::ProjectionMatrix FigureWindow::getProjectionMatrix() const
	{
		Geometry::ProjectionMatrix ret=P;
		callback.ignoreNotifications(true);
		if (GetSet<bool>("Projection/Manual Projection",dictionary))
			ret=projection->getProjectionMatrix(pixmap.width(),pixmap.height());
		GetSet<Geometry::ProjectionMatrix>("General/Projection Matrix",dictionary)=ret;
		callback.ignoreNotifications(false);
		return ret;
	}

	ProjectionParameters& FigureWindow::projection_parmeters()
	{
		GetSet<bool>("Projection/Manual Projection",dictionary)=true;
		return *projection;
	}

	Eigen::Vector2i FigureWindow::getVisibleSize() const
	{
		// FIXME: window->centralWidget()->sizeHint() ?! This is really shitty code.
		int max_w=window->width()-2;
		int max_h=window->height()-2;
		if (window->menuBar()->isVisible())   max_h-=window->menuBar()->height();
		if (window->statusBar()->isVisible()) max_h-=window->statusBar()->height();
		return Eigen::Vector2i(max_w,max_h);
	}

	void FigureWindow::setVisibleSize(const Eigen::Vector2i& size)
	{
		int win_w=size[0]+2;
		int win_h=size[1]+2;
		if (window->menuBar()->isVisible())   win_h+=window->menuBar()->height();
		if (window->statusBar()->isVisible()) win_h+=window->statusBar()->height();
		window->resize(win_w,win_h);
	}

	bool FigureWindow::savePNG(const std::string& filename)
	{
		return render(1.0).save(filename.c_str());
	}

	bool FigureWindow::savePDF(const std::string& filename)
	{
		update();
		// Paint to QPicture to explicitly ask for vector graphics.
		QPicture pic;
		QPainter p(&pic);
		overlay.draw(p,getProjectionMatrix(),1);
		p.end();

		// Set up a PDF printer
		QPrinter printer;
		printer.setOutputFormat(QPrinter::PdfFormat);
		printer.setOutputFileName(filename.c_str());
		printer.setPageMargins(0,0,0,0,QPrinter::Unit::Millimeter);
		printer.setPaperSize(QSizeF((double)pixmap.width(),(double)pixmap.height()), QPrinter::Unit::DevicePixel);

		// Finally, print to PDF
		QPainter pdf_painter(&printer);
		if (!!image_source)
			pdf_painter.drawPixmap(0,0,pixmap.width(),pixmap.height(),pixmap);
		pdf_painter.drawPicture(0,0,pic);
		pdf_painter.end();
		return true;
	}

	void FigureWindow::info(const std::string& info_text) {
		status_info->setText(info_text.c_str());
	}

	void FigureWindow::update()
	{
		QPixmap display=render(GetSet<double>("General/Magnification/Factor",dictionary));
		if (display.width()>0 && display.height()>0)
		{
			label->setPixmap(display);
			label->setFixedSize(display.size());
		}
		window->show();
	}

	void FigureWindow::setAutoResize(bool auto_resize)
	{
		GetSet<bool>("General/Magnification/Auto",dictionary)=auto_resize;
		update();
	}

	void FigureWindow::setPlainLook(bool on)
	{
		// If menubar is invisible and plainlook is on (or the other way round) we need not act
		if (window->menuBar()->isVisible()==on)
		{
			window->menuBar()->setVisible(!on);
			window->statusBar()->setVisible(!on);
			int sizediff_y=window->menuBar()->height()+window->statusBar()->height();
			int w=window->width(),h=window->height();
			if (on) window->resize(w,h-sizediff_y);
			else    window->resize(w,h+sizediff_y);
		}
	}
	
	//
	// ProgressInterface implementation
	//

	void FigureWindow::progressStart(const std::string& who, const std::string& what, int maximum, bool *cancel_clicked)
	{
		status_progress->setValue(-1);
		status_progress->setMaximum(maximum);
		progress_cancel_clicked=cancel_clicked;
		status_progress->show();
		if (cancel_clicked)
			status_cancel->show();
		info(who + " - " + what);
		window->raise();
		QCoreApplication::processEvents();
	}

	void FigureWindow::progressUpdate(int i)
	{
		status_progress->setValue(i);
		QCoreApplication::processEvents();
	}

	void FigureWindow::progressEnd()
	{
		status_progress->hide();
		status_cancel->hide();
		progress_cancel_clicked=0x0;
		window->raise();
	}

	void FigureWindow::info(const std::string& who, const std::string& what, bool show_dialog)
	{
		info(who + " - " + what);
	}

	void FigureWindow::warn(const std::string& who, const std::string& what, bool only_inormative)
	{
		info(who + " - " + what);
	}


	std::map<std::string, FigureWindow*>& FigureWindow::instances()
	{
		static std::map<std::string, FigureWindow*> figures;
		return figures;
	}

	
	//
	/// Access to a database of Figure windows
	//

	void FigureWindow::instance_delete(const std::string& name)
	{
		auto it = instances().find(name);
		if (it != instances().end())
		{
			delete it->second;
			instances().erase(it);
		}
	}

	std::set<std::string> FigureWindow::instance_names(bool must_have_image, bool must_have_projection_matrix, bool must_be_visible)
	{
		std::set<std::string> ret;
		auto &figures = instances();
 		for (auto it=figures.begin();it!=figures.end();++it) {
			if (must_have_image && !(it->second->getImage())) continue;
			if (must_have_projection_matrix) {
				Geometry::ProjectionMatrix P=it->second->getProjectionMatrix();
				if (P.isZero() || P.isIdentity()) continue;
			}
			if (must_be_visible && !it->second->window->isVisible())
				continue;
			// add instance
			ret.insert(it->first);
		}
		return ret;
	}

	FigureWindow& FigureWindow::instance(const std::string& name)
	{
		auto &figures = instances();
		if (figures.find(name) == figures.end())
			figures[name] = new FigureWindow(name);
		return *figures[name];
	}

	bool FigureWindow::exists(const std::string& name, bool must_be_visible)
	{
		auto &figures = instances();
		if (figures.find(name) == figures.end())
			return false;
		else if (!must_be_visible) return true;
		return instance(name).window->isVisible();
	}

	void FigureWindow::instance_delete_all() {
		for (auto it=instances().begin();it!=instances().end();++it)
			delete it->second;
		instances().clear();
	}

} // namespace UtilsQt
