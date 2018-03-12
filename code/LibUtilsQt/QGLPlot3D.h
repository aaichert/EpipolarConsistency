// Created by Andre Aichert <aaichert@gmail.com> in 2011
#ifndef QGLPLOT3D_H
#define QGLPLOT3D_H

#include <QtOpenGL/QGLWidget>

#include <string>

namespace UtilsQt {

	class QGLPlot3D : public QGLWidget {
	public:
		QGLPlot3D(QWidget* parent=0x0);
		QGLPlot3D(const std::string& path, QWidget* parent=0x0);
		// Data is stored in a 3*size*size matrix stores, unlike in MATLAB, in row-major order.
		QGLPlot3D(int size, float* data, QWidget* parent = 0);
		// Data is stored in a single-channel w*h image
		QGLPlot3D(int w, int h, float* data, double minx=-1, double maxx=1, double miny=-1, double maxy=1, QWidget* parent = 0);
		void setLabel(const std::string& lx, const std::string& ly) { m_label_x = lx; m_label_y = ly; }
		~QGLPlot3D();

		/// Save graph as text file.
		bool save(const std::string& path);
		/// Read file saved by QGLPlot3D
		bool load(const std::string& path);

	protected:
			/// Set the data to plot, takes ownership of data;
		void setData(int size, float *data);

		void initializeGL();
		void paintGL();
		void resizeGL(int width, int height);

		void mouseMoveEvent(QMouseEvent *event);
		void wheelEvent(QWheelEvent *event);
		void keyPressEvent(QKeyEvent *event);

		/// Cross product of 3-vectors
		void crossprod(float *v1, float *v2, float *prod);
		/// Normalizes a 3-vector
		void normalize(float *v);
		/// Draws a surface point with the right color & normal
		void draw_vertex(int i, int j);
		/// Draws grid lines
		void drawGrid();
		/// Render offset text
		void renderOffsetText(double x, double y, double z, const std::string& txt, const QFont& f);
		/// Draws text
		void drawText();

		float *m_data;				///< The surface data
		float *m_normals;			///< The surface normals
		int m_size;					///< Width and height of the plot
		double m_bounds[6];			///< Min/Max in X,Y,Z

		// Display Options
		bool m_optGrid;				///< Show grid (key 'g')
		bool m_optHalfGrid;			///< Fine grid on/off (key 'v')
		bool m_optColors;			///< Color code heatmap or just gray (key 'c')
		bool m_optFlatShaded;		///< Flat shading (key 'f')
		bool m_optLighting;			///< Light on/off (key 'l')
		bool m_optIsoLines;			///< Show 10 iso-levels by shading (key 'i')
		bool m_optOrtho;			///< Perspective and Orthographic projection (key 'p')
		bool m_optNoNumbers;		///< Print scale next to axes (key 'n' as in Numbers)
		bool m_optBig;				///< Thicker lines and larger text (key 'b')
		bool m_optMovingLight;		///< Light is fixed relative to view. (key 'm')

		double	m_view[3];			///< x/y viewing angles and distance from center
		int		m_mouse_drag[2];	///< x/y position of mouse
		double	m_isoLevel;			///< z-value of threshold plane. (key 't')

		std::string m_label_x;
		std::string m_label_y;

		static const float CMAP[196];		///< MATLAB-style colormap
		static const float LightSource[4];	///< Position of the diffuse light source
		static const float LightAmbient[4];	///< Ambient light color
		static const float LightWhite[4];	///< Diffuse and specular light color
	};

} // namespace UtilsQt

#endif // QGLPLOT3D_H
