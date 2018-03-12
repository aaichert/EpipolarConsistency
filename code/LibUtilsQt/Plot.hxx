#ifndef __qcp_utils
#define __qcp_utils
// Created by aaichert on April 28th 2016
// A plot window and some info we need
#include "QCustomPlot.h"

#include <QFileDialog>

#include <numeric>

#include <GetSetGui/GetSetTabWidget.h>

namespace UtilsQt {

	// Defined in FigureWindow.cpp if the whole UtilsQt library is available.
	std::map<std::string, QCustomPlot*>& plotsByName();
	/*
	/// Access to a database of plot windows
	inline std::map<std::string, QCustomPlot*>& plotsByName()
	{
		static std::map<std::string, QCustomPlot*> plots;
		return plots;
	}
	*/

	/// Access a plot window
	inline QCustomPlot& plotByTitle(const std::string& title)
	{
		QCustomPlot* plot=0x0;
		auto it=plotsByName().find(title);
		if (it==plotsByName().end())
		{
			plot=new QCustomPlot();
			plot->resize(500,300);
			plot->setWindowTitle(title.c_str());
			plotsByName()[title]=plot;
		}
		else
			plot=it->second;
		return *plot;
	}

	/// Close and delete a plot window
	inline void plotDelete(const std::string& title)
	{
		auto it=plotsByName().find(title);
		if (it!=plotsByName().end())
		{
			delete it->second;
			plotsByName().erase(it);
		}
	}

	/// Accesses a graph in a plot by name and index. If is negative, a new graoh is created.
	inline QCPGraph& plotGraphByTitleIdx(const std::string& title, int graph_idx)
	{
		auto &plot=plotByTitle(title);
		while (plot.graphCount()<=graph_idx || graph_idx<0)
		{
			int c=plot.graphCount();
			if (graph_idx<0) graph_idx=c;
			plot.addGraph();
			QPen pen;
			pen.setColor(QColor(255 * (c % 2), 255 * (c / 2 % 2), 255 * (c / 4 % 2)));
			pen.setStyle(Qt::PenStyle((c+1)-((int)(c/6)))); // Qt::NoPen, Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine, Qt::DashDotDotLine, Qt::CustomDashLine
			plot.graph(c)->setPen(pen);
		}
		return *plot.graph(graph_idx);
	}

	/// A simple and general interface for 2D plots. Backend is the QCP library, see also plotByTitle(...) and plotGraphByTitleIdx(...)
	class Plot
	{
		const std::string title;
		bool              no_force_update;
		bool              no_force_show;
	public:

		/// An empty plot window
		Plot(const std::string& _title, bool reset=false)
			: title(_title) , no_force_update(false) , no_force_show(false)
		{
			if (reset) close();
		}

		/// A simple 2D plot from an std::vector. See also addGraph(...) .
		Plot(const std::string& _title, const std::vector<double>& y, bool reset=true)
			: title(_title) , no_force_update(false) , no_force_show(false)
		{
			close();
			graph(0).setData(y);
		}

		/// A single 2D plot from std::vectors
		Plot(const std::string& _title, const std::vector<double>& x, const std::vector<double>& y)
			: title(_title) , no_force_update(false) , no_force_show(false)
		{
			close();
			graph(0).setData(x,y);
		}

		/// Do not show this plot just yet
		Plot& no_show()   { no_force_show   =true; return *this; }

		/// Do not force an immediate update (do not process events)
		Plot& no_update() { no_force_update =true; return *this; }

		/// Immediately show the plot
		~Plot() {
			if (!no_force_show)
			{
				auto& qcp=plotByTitle(title);
				qcp.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
				qcp.rescaleAxes();
				qcp.replot();
				qcp.show();
			}
			if (!no_force_update)
				QCoreApplication::processEvents();
		}

		/// Save plot in PDF format
		bool savePdf(const std::string& path)
		{
			return plotByTitle(title).savePdf(path.c_str());
		}

		/// Close Plot window (and destroy)
		void close()
		{
			plotDelete(title);
		}

		/// Add text to the top or bottom of a plot
		Plot& addText(const std::string& caption, bool top=false)
		{
			auto& plot(plotByTitle(title));
			QCPTextElement* element=new QCPTextElement(&plot, caption.c_str(), QFont("sans", 12, QFont::Bold));
			if (top)
			{
				plot.plotLayout()->insertRow(0);
				plot.plotLayout()->addElement(0, 0, element);
			}
			else
				plot.plotLayout()->addElement(element);
			return *this;
		}

		/// Define axis labels.
		Plot& setAxisLabels(const std::string& label_x, const std::string& label_y)
		{
			auto &plot=plotByTitle(title);
			plot.xAxis->setLabel(label_x.c_str());
			plot.yAxis->setLabel(label_y.c_str());
			return *this;
		}

		/// The x-axis shows an angle. Optionally converts radians to degrees.
		Plot& setAxisAngularX(bool convert_to_degrees=true)
		{
			auto ticker=QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi());
			if (convert_to_degrees) {
				ticker->setPiSymbol(QString::fromLatin1("\u00B0"));
				ticker->setPiValue(Geometry::Pi/180);
			}
			UtilsQt::plotByTitle(title).xAxis->setTicker(ticker);
			return *this;
		}

		/// Draw a vertical line at position x.
		Plot& drawVerticalLine(float x, QColor color, double thickness)
		{
			auto &plot=plotByTitle(title);
			QCPItemLine *line = new QCPItemLine(&plot);
			QPen pen(color);
			pen.setWidthF(thickness);
			line->setPen(pen);
			line->start->setCoords(x, -1e7);
			line->end  ->setCoords(x,  1e7);
			return *this;
		}

		/// Remove additional items from plot (lines etc.)
		Plot& clearItems()
		{
			auto &plot=plotByTitle(title);
			plot.clearItems();
			return *this;
		}

		/// Shows legend. See also graph(...).setName(...).
		Plot& showLegend(bool visible=true)
		{
			plotByTitle(title).legend->setVisible(visible);
			return *this;
		}
		
		/// Add data as a graph to this plot.
		Plot& addGraph(const std::string& name, const std::vector<double>& x, const std::vector<double>& y)
		{
			graph().setName(name).setData(x,y);
			return showLegend();
		}

		/// Add data as a graph to this plot.
		Plot& addGraph(const std::string& name, const std::vector<double>& y)
		{
			graph().setName(name).setData(y);
			return showLegend();
		}

		/// Close all plots.
		static void closeAll()
		{
			for (auto it=plotsByName().begin();it!=plotsByName().end();++it)
				if (it->second) delete it->second;
			plotsByName().clear();
		}

		/// A 2D graph within a plot window.
		class Graph {
			QCPGraph& graph;
			
			friend class Plot;
			Graph(QCPGraph& _graph)
				: graph(_graph)
			{}

		public:

			/// Direct access to QCP (avoid usage)
			QCPGraph& qcp()
			{
				return graph;
			}

			/// Define the name of this graph to be shown in the legend.
			Graph& setName(const std::string graph_name)
			{
				graph.setName(graph_name.c_str());
				return *this;
			}

			/// Define a color by three values between 0 abd 1.
			Graph& setColor(float r, float g, float b)
			{
				graph.setPen(QPen(QColor(255*r,255*g,255*b)));
				return *this;
			}

			/// Define a color by QColor
			Graph& setColor(QColor c)
			{
				graph.setPen(QPen(c));
				return *this;
			}

			/// Set the data from an std::vector
			Graph& setData(const std::vector<double>& y)
			{
				std::vector<double> x(y.size());
				std::iota(x.begin(),x.end(),0);
				return setData(x,y);
			}

			/// Set the data from std::vectors
			Graph& setData(const std::vector<double>& x, const std::vector<double>& y)
			{
				graph.setData(QVector<qreal>::fromStdVector(x),QVector<qreal>::fromStdVector(y));
				return *this;
			}

			/// Set the data from raw data
			template<typename Scalar>
			Graph& setData(int n, const Scalar* ydata, int ystride = 1)
			{
				std::vector<double> y(n);
				for (int i=0;i<n;i++)
					y[i]=(double)ydata[i*ystride];
				return setData(y);
			}

			/// Set the data from raw data
			template<typename ScalarX, typename ScalarY>
			Graph& setData(int n, const ScalarX * xdata, const ScalarY* ydata, int xstride = 1, int ystride = 1)
			{
				std::vector<double> x(n), y(n);
				for (int i=0;i<n;i++)
				{
					x[i]=(double)xdata[i*xstride];
					y[i]=(double)ydata[i*ystride];
				}
				return setData(x,y);
			}

			/// Add a single sample to a plot
			Graph& addData(double x, double y)
			{
				graph.addData(x,y);
				return *this;
			}

			/// Add a single sample to a plot
			Graph& addData(double y)
			{
				return addData(graph.data()->size(),y);
				// addData(QVector<qreal>(1,x),QVector<qreal>(1,y));
			}

		};

		/// Access a graph within this plot by its index. If no index is provided, a new graph will be added.s
		Graph graph(int idx=-1)
		{
			return Graph(plotGraphByTitleIdx(title,idx));
		}

	};

		
//////////////////
// Plot Editor


	inline GetSetInternal::Dictionary& plotEditorDictionary()
	{
		static GetSetInternal::Dictionary plot_edit_dict;
		return plot_edit_dict;
	}

	void gui_plot_edit(const GetSetInternal::Node& node);

	inline GetSetHandler& plotEditorHandler()
	{
		static GetSetHandler *callback=0x0;
		if (!callback)
			callback=new GetSetHandler(gui_plot_edit,plotEditorDictionary());
		return *callback;
	}

	inline void gui_plot_edit(const GetSetInternal::Node& node)
	{
		auto &callback(plotEditorHandler());
		auto& plot_edit_dict=plotEditorDictionary();

		/// All things relating to a whole plot
		std::string active_plot=GetSet<>("Available Plots/Plot Name",plot_edit_dict);
		if (active_plot.empty()) return;
		if (node.name=="Save as PDF")
		{
			std::string path = QFileDialog::getSaveFileName(0x0, "Export as PDF", active_plot.c_str(), "Portable Document Format (*.pdf);;All Files (*)").toStdString();
			if (path.empty()) return;
			plotByTitle(active_plot).savePdf(path.c_str());
		}

		if (node.name=="Plot Name")
		{
			callback.ignoreNotifications(true);	
			GetSet<>("Available Plots/Label X",plot_edit_dict)=plotByTitle(active_plot).xAxis->label().toStdString();
			GetSet<>("Available Plots/Label Y",plot_edit_dict)=plotByTitle(active_plot).yAxis->label().toStdString();
			callback.ignoreNotifications(false);
		}

		if (node.name=="Label X")
			plotByTitle(active_plot).xAxis->setLabel(GetSet<>("Available Plots/Label X",plot_edit_dict).getString().c_str());

		if (node.name=="Label Y")
			plotByTitle(active_plot).yAxis->setLabel(GetSet<>("Available Plots/Label Y",plot_edit_dict).getString().c_str());

		/// All things specific to a graph inside a plot
		int ngraph=plotByTitle(active_plot).graphCount();
		int graph=GetSet<int>("Graphs/Graph Index",plot_edit_dict);
		if (graph<0||graph>=ngraph)
		{
			callback.ignoreNotifications(true);	
			GetSet<>("Graphs/Data X",plot_edit_dict)="";
			GetSet<>("Graphs/Data Y",plot_edit_dict)="";
			GetSet<>("Graphs/Legend",plot_edit_dict)="";
			callback.ignoreNotifications(false);
			return;
		}
		
		if (node.name=="Graph Index")
		{
			callback.ignoreNotifications(true);	
			// Get current data
			std::vector<double> keys;
			std::vector<double> values;
			auto graph_data=plotByTitle(active_plot).graph(graph)->data();
			for (auto it=graph_data->begin();it!=graph_data->end(); ++it)
			{
				keys.push_back(it->key);
				values.push_back(it->value);
			}
			GetSet<std::vector<double> >("Graphs/Data X",plot_edit_dict)=keys;
			GetSet<std::vector<double> >("Graphs/Data Y",plot_edit_dict)=values;
			GetSet<>("Graphs/Legend",plot_edit_dict)=plotByTitle(active_plot).graph(graph)->name().toStdString();

			int style=0;
			QPen pen=plotByTitle(active_plot).graph(graph)->pen();
			if (pen.style()==Qt::DashLine)	  style=1;
			if (pen.style()==Qt::DotLine)	  style=2;
			if (pen.style()==Qt::DashDotLine) style=3;
			GetSetGui::Enum("Graphs/Pen Style",plot_edit_dict).setChoices("SolidLine;DashLine;DotLine;DashDotLine")=style;

			QColor color=pen.color();
			GetSet<int>("Graphs/Pen Color/0 Red"  ,plot_edit_dict)=color.red();
			GetSet<int>("Graphs/Pen Color/1 Green",plot_edit_dict)=color.green();
			GetSet<int>("Graphs/Pen Color/2 Blue" ,plot_edit_dict)=color.blue();
			GetSet<int>("Graphs/Pen Color/3 Alpha",plot_edit_dict)=color.alpha();
			callback.ignoreNotifications(false);
		}

		if (node.name=="Pen Style")
		{
			QPen pen=plotByTitle(active_plot).graph(graph)->pen();
			int style=GetSet<int>("Graphs/Pen Style",plot_edit_dict);
			if (false) ;
			else if (style==1) pen.setStyle(Qt::DashLine); 
			else if (style==2) pen.setStyle(Qt::DotLine);
			else if (style==3) pen.setStyle(Qt::DashDotLine);
			else               pen.setStyle(Qt::SolidLine);
			plotByTitle(active_plot).graph(graph)->setPen(pen);
		}

		if (node.super_section=="Graphs/Pen Color")
		{
			QColor color(
				GetSet<int>("Graphs/Pen Color/0 Red"  ,plot_edit_dict),
				GetSet<int>("Graphs/Pen Color/1 Green",plot_edit_dict),
				GetSet<int>("Graphs/Pen Color/2 Blue" ,plot_edit_dict),
				GetSet<int>("Graphs/Pen Color/3 Alpha",plot_edit_dict)
				);
			QPen pen=plotByTitle(active_plot).graph(graph)->pen();
			pen.setColor(color);
			plotByTitle(active_plot).graph(graph)->setPen(pen);
		}

		if (node.name=="Legend")
			Plot(active_plot).graph(graph).setName(GetSet<>("Graphs/Legend",plot_edit_dict));

		plotByTitle(active_plot).replot();
	}

	inline GetSetGui::GetSetTabWidget* showPlotEditor(bool force_open = false)
	{
		// List Plots
		std::vector<std::string> plots;
		auto& plot_map=UtilsQt::plotsByName();
		if (!force_open && plot_map.empty()) {
			std::cout << "There are no plots to edit.\n";
			return 0x0;
		}
		for (auto it=plot_map.begin();it!=plot_map.end();++it)
			plots.push_back(it->first);
		auto& plot_edit_dict=plotEditorDictionary();
		// Show
		auto &callback(plotEditorHandler());
		callback.ignoreNotifications(true);
		GetSetGui::Button("Available Plots/Save as PDF",plot_edit_dict)="Export...";
		callback.ignoreNotifications(false);
		GetSetGui::Enum("Available Plots/Plot Name",plot_edit_dict).setChoices(vectorToString(plots,";"))=0;
		GetSet<int>("Graphs/Graph Index",plot_edit_dict)=0;
		GetSetGui::GetSetTabWidget *save_diag=new GetSetGui::GetSetTabWidget(0x0,plot_edit_dict);
		save_diag->setAttribute(Qt::WA_DeleteOnClose);
		save_diag->show();
		return save_diag;
	}

//////////////////
} // namespace UtilsQt

#endif // __qt_utils
