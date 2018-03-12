
#include "QGLPlot3D.h"

#include <QMouseEvent>
#include <QFileDialog>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cmath>

#include <NRRD/nrrd_image.hxx>

namespace UtilsQt {

	/// Convert anything to std::string (could #include <Utils.h> instead)
	template <typename T> inline std::string toString(const T& in)
	{
		std::ostringstream strstr;
	//	if (in<1)
			strstr << std::setprecision(4) << in;
	//	else
	//		strstr << in;
		return strstr.str();
	}

	QGLPlot3D::QGLPlot3D(QWidget* parent) :
	QGLWidget(parent),
	m_data(0x0),
	m_normals(0x0),
	m_optBig(0),m_optMovingLight(0),
	m_isoLevel(-.5)
	{
		setAttribute(Qt::WA_DeleteOnClose);
		setMouseTracking(true);
		QString path = QFileDialog::getOpenFileName(0x0, "Open QGLPlot3D Plot", ".", "QGLPlot3D Plot (*.m *.plot)");
			if (!load(path.toStdString()))
				std::cerr << "Failed to load plot." << std::endl;
	}

	QGLPlot3D::QGLPlot3D(const std::string& path, QWidget* parent) :
	QGLWidget(parent),
	m_data(0x0),
	m_normals(0x0),
	m_optBig(0),m_optMovingLight(0),
	m_isoLevel(-.5)
	{
		setAttribute(Qt::WA_DeleteOnClose);
		setMouseTracking(true);
		if (!load(path))
				std::cerr << "Failed to load plot." << std::endl;
	}

	QGLPlot3D::QGLPlot3D(int size, float* data, QWidget* parent):
	QGLWidget(parent),
	m_data(0x0),
	m_normals(0x0),
	m_optGrid(1),
	m_optHalfGrid(0),
	m_optColors(1),
	m_optFlatShaded(0),
	m_optLighting(1),
	m_optIsoLines(1),
	m_optOrtho(0),
	m_optNoNumbers(0),
	m_optBig(0),
	m_optMovingLight(0),
	m_isoLevel(-.5)
	{
		int l=size*size*3;
		float *copy_data=new float[l];
		for (int i=0;i<l;i++) copy_data[i]=data[i];
		setAttribute(Qt::WA_DeleteOnClose);
		setData(size, copy_data);
		m_view[0]=0.9;
		m_view[1]=3.6;
		m_view[2]=5.0;
		setMouseTracking(true);
	}

	QGLPlot3D::QGLPlot3D(int w, int h, float* data, double minx, double maxx, double miny, double maxy, QWidget* parent):
	QGLWidget(parent),
	m_data(0x0),
	m_normals(0x0),
	m_optGrid(1),
	m_optHalfGrid(0),
	m_optColors(1),
	m_optFlatShaded(0),
	m_optLighting(1),
	m_optIsoLines(1),
	m_optOrtho(0),
	m_optNoNumbers(0),
	m_optBig(0),
	m_optMovingLight(0),
	m_isoLevel(-.5)
	{
		NRRD::ImageView<float> input(w,h,1,data);
		int size=std::max(w,h);
		NRRD::Image<float>     copy_data(3,size,size);
		for (int y=0;y<size;y++)
			for (int x=0;x<size;x++)
			{
				double rel_x=(double)x/(size-1);
				double rel_y=(double)y/(size-1);
				float *vertex=&copy_data.pixel(0,x,y);
				vertex[0]=minx*rel_x+maxx*(1.0-rel_x);
				vertex[1]=miny*rel_y+maxy*(1.0-rel_y);
				vertex[2]=input(rel_x*(w-1),rel_y*(h-1));
			}
		setAttribute(Qt::WA_DeleteOnClose);
		setData(size, copy_data.passOwnership());
		m_view[0]=0.9;
		m_view[1]=3.6;
		m_view[2]=5.0;
		setMouseTracking(true);
	}

	QGLPlot3D::~QGLPlot3D() {
		if (m_normals) delete [] m_normals;
		if (m_data) delete[] m_data;
	}

	void QGLPlot3D::crossprod(float *v1, float *v2, float *prod) {
		prod[0] = v1[1] * v2[2] - v2[1] * v1[2];
		prod[1] = v1[2] * v2[0] - v2[2] * v1[0];
		prod[2] = v1[0] * v2[1] - v2[0] * v1[1];
	}

	void QGLPlot3D::normalize(float *v) {
		float d;
		d = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		if (d != 0.0) {
			d = 1 / d;
			v[0] *= d;
			v[1] *= d;
			v[2] *= d;
		}
	}

	void QGLPlot3D::setData(int size, float *data) {
		m_size = size;
		if (m_data) delete [] m_data;
		m_data = data;
		if (m_normals) delete [] m_normals;
		m_normals = new float[size*size*3];
		int i, j, k;
		float min[3], max[3], minpos[2], maxpos[2];
		for (i = 0; i < 3; i++) {
			min[i] = m_data[i]; max[i] = m_data[i];
		}
		maxpos[0] = m_data[0]; maxpos[1] = m_data[1];
		minpos[0] = m_data[0]; minpos[1] = m_data[1];
		for (i = 0; i < m_size*m_size; i++) {
			for (j = 0; j < 3; j++) {
				if (min[j] > m_data[3*i+j]) {
					min[j] = m_data[3*i+j];
					if (j == 2) { minpos[0] = m_data[3*i]; minpos[1] = m_data[3*i+1]; }
				}
				if (max[j] < m_data[3*i+j]) {
					max[j] = m_data[3*i+j];
					if (j == 2) { maxpos[0] = m_data[3*i]; maxpos[1] = m_data[3*i+1]; }
				}
			}
		}
		std::cout << "Parameter range [" << min[0] << ".." << max[0] << "], ["
			<< min[1] << ".." << max[1] << "]" << std::endl;
		std::cout << "Value range " << min[2] << " - " << max[2] << ", minimum at ["
			<< minpos[0] << ", " << minpos[1] << "], maximum at ["
			<< maxpos[0] << ", " << maxpos[1] << "]" << std::endl;
		float zRange = max[2]-min[2];
		if (zRange == 0.0f)
			zRange = 1.0f;
		for (i = 0; i < m_size*m_size; i++) {
			m_data[3*i  ] = (2.0f*(m_data[3*i  ]-min[0])/(max[0]-min[0]))-1.0f;
			m_data[3*i+1] = (2.0f*(m_data[3*i+1]-min[1])/(max[1]-min[1]))-1.0f;
			m_data[3*i+2] = (m_data[3*i+2]-min[2])/zRange-0.5f;
		}

		// The following code computes nice normals for the surface
		float x1[3], x2[3], y1[3], y2[3];
		for (i = 0; i < m_size; i++) {
			for (j = 0; j < m_size; j++) {
				/* Compute difference vectors in all four directions, but
				only if they exist (correct treatment of borders) */
				for (k = 0; k < 3; k++) {
					if (i < m_size-1) x1[k] =
					data[3*(j+m_size*(i+1))+k] - data[3*(j+m_size*i)+k];
					if (i) x2[k] = data[3*(j+m_size*(i-1))+k] - data[3*(j+m_size*i)+k];
					if (j) y1[k] = data[3*((j-1)+m_size*i)+k] - data[3*(j+m_size*i)+k];
					if (j < m_size-1) y2[k] = data[3*((j+1)+m_size*i)+k] - data[3*(j+m_size*i)+k];
				}
				// Normalize all vectors
				normalize(x1); normalize(x2);
				normalize(y1); normalize(y2);
				/* Compute average vector for x and y directions, respectively. Again,
				check if we are at the border. */
				for (k = 0; k < 3; k++) {
					if ((i) && (i < m_size-1)) x1[k] = 0.5f * (x1[k] - x2[k]);
					else if (i) x1[k] = -x2[k];
					if ((j) && (j < m_size-1)) y1[k] = 0.5f * (y1[k] - y2[k]);
					else if (j < m_size-1) y1[k] = -y2[k];
				}
				// Finally, compute the normal
				crossprod(y1, x1, &m_normals[3*(j+m_size*i)]);
				normalize(&m_normals[3*(j+m_size*i)]);
			}
		}

		m_bounds[0]=min[0];
		m_bounds[1]=max[0];
		m_bounds[2]=min[1];
		m_bounds[3]=max[1];
		m_bounds[4]=min[2];
		m_bounds[5]=max[2];
	}

	void QGLPlot3D::draw_vertex(int i, int j) {
		if (m_optColors)
		{
			// Interpolate from color table
			float c = (m_data[3*(j+m_size*i)+2]+0.5f)*62.999f;
			int c2 = (int)c; c = c-c2;
			float color[3];
			for (int k = 0; k < 3; k++)
				if (3*c2+k>=0 && 3*(c2+1)+k<=196)
					color[k] = CMAP[3*c2+k]*(1.0f-c)+CMAP[3*(c2+1)+k]*c;
			// Draw vertex
			glColor3fv(color);
		}
		glNormal3fv(&m_normals[3*(j+m_size*i)]);
		glVertex3fv(&m_data[3*(j+m_size*i)]);
	}

	void QGLPlot3D::initializeGL()
	{
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glEnable(GL_DEPTH_TEST);
		glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, LightWhite);
		glLightfv(GL_LIGHT0, GL_SPECULAR, LightWhite);
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHTING);
		glLightModelf(GL_LIGHT_MODEL_TWO_SIDE,1.0);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128);
		glColorMaterial(GL_BACK,GL_AMBIENT_AND_DIFFUSE);
		glColorMaterial(GL_FRONT,GL_AMBIENT);
		glEnable(GL_COLOR_MATERIAL);
		glPolygonOffset(1, 1);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glEnable(GL_NORMALIZE);
	}

	bool QGLPlot3D::save(const std::string& path)
	{
		std::ofstream file(path.c_str());
		if (!file.good()) return false;
		file << "%% QGLPlot3D saved plot v.2.0" << std::endl;
		file << "%% x: " << m_label_x << std::endl;
		file << "%% y: " << m_label_y << std::endl;

		file << "%% view: "		<< m_view[0] << " "
								<< m_view[1] << " "
								<< m_view[2] << std::endl;

		file << "%% bounds: "	<< m_bounds[0] << " "
								<< m_bounds[1] << " "
								<< m_bounds[2] << " "
								<< m_bounds[3] << " "
								<< m_bounds[4] << " "
								<< m_bounds[5] << std::endl;

		file << "%% size: " << m_size << std::endl;

		file << "%% options: ";
		if (m_optGrid)			file << "showGrid ";
		if (m_optHalfGrid)		file << "showHalfGrid ";
		if (m_optColors)		file << "showColors ";
		if (m_optFlatShaded)	file << "showFlat ";
		if (m_optLighting)		file << "showLighting ";
		if (m_optIsoLines)		file << "showIsoLines ";
		if (m_optOrtho)			file << "showOrtho ";
		if (m_optNoNumbers)		file << "showNoNumbers ";
		if (m_optBig)			file << "showBig ";
		if (m_optMovingLight)	file << "movingLight ";
		file << std::endl << std::endl;

		file << "Z = [\n";
		double zrange=m_bounds[5]-m_bounds[4];
		double minz=m_bounds[4];
		for (int x=0;x<m_size;x++)
		{
			for (int y=0;y<m_size;y++)
				file << (m_data[(y*m_size+x)*3+2]+.5)*zrange+minz << " ";
			file << std::endl;
		}
		file << "]';\n";
		file << std::endl;
		file << "range = [\n";
		file << m_bounds[0] << " " << m_bounds[1] << std::endl;
		file << m_bounds[2] << " " << m_bounds[3] << std::endl;
		file << "];\n" << std::endl;
		file << "surf(Z)\n";
		file.close();
		setWindowTitle((std::string("QGLPlot3D - ")+path).c_str());
		return true;
	}

	bool QGLPlot3D::load(const std::string& path)
	{
		std::ifstream file(path.c_str());
		if (!file.good()) return false;
		file.ignore('\n'); // Plotview bla

		file.ignore(999,':'); // x:
		getline(file,m_label_x);
		file.ignore(999,':'); // y:
		getline(file,m_label_y);

		file.ignore(999,':'); // view:
		file >> m_view[0] >> m_view[1] >> m_view[2];

		file.ignore(999,':'); // bounds:
		double bounds[6];
		file	>> bounds[0] >> bounds[1]
				>> bounds[2] >> bounds[3]
				>> bounds[4] >> bounds[5];

		file.ignore(999,':'); // size:
		int size;
		file >> size;
		if (size<0||size>500)
		{
			std::cerr << "Plot data size exceeds bounds.\n" << std::endl;
			return false;
		}
		float *d=new float[3*size*size];

		file.ignore(999,':'); // options:
		std::string options;
		getline(file,options);
		std::istringstream str(options);
		m_optGrid= m_optHalfGrid=
		m_optColors= m_optFlatShaded=
		m_optLighting= m_optIsoLines=
		m_optOrtho= m_optNoNumbers=
		m_optMovingLight = m_optBig=0;
		while (!str.eof())
		{
			std::string opt;
			str >> opt;
			if (opt=="showGrid")		m_optGrid=1;
			if (opt=="showHalfGrid")	m_optHalfGrid=1;
			if (opt=="showColors")		m_optColors=1;
			if (opt=="showFlat")		m_optFlatShaded=1;
			if (opt=="showLighting")	m_optLighting=1;
			if (opt=="showIsoLines")	m_optIsoLines=1;
			if (opt=="showOrtho")		m_optOrtho=1;
			if (opt=="showNoNumbers")	m_optNoNumbers=1;
			if (opt=="showBig")			m_optBig=1;
			if (opt=="optMovingLight")	m_optMovingLight=1;
		}

		file.ignore(999,'['); // Z = [

		float oos=1.0/(size-1);
		for (int x=0;x<size;x++)
			for (int y=0;y<size;y++)
		{
			int i=(y*size+x)*3;
			d[i+0]=x*oos*(bounds[3]-bounds[2])+bounds[2];
			d[i+1]=y*oos*(bounds[1]-bounds[0])+bounds[0];
			file >> d[i+2];
		}
		setData(size,d);
		file.close();
		setWindowTitle((std::string("QGLPlot3D - ")+path).c_str());
		resize(400,300);
		return true;
	}

	void QGLPlot3D::resizeGL(int width, int height)
	{
		glViewport(0, 0, width, height);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		double aspect = (double)width / (double)height;
	//	if (m_optOrtho)
			glOrtho(-1.5*aspect,1.5*aspect,-1.5,1.5,1,20);
	//	else
	//	    gluPerspective(35.0f, aspect, 1, 20.0);
	}

	void QGLPlot3D::drawGrid()
	{
		int a=1+3*(int)m_optHalfGrid;
		if (m_optBig)
			glLineWidth(2.0);
		else
			glLineWidth(1.0);
		glColor4d(0,0,0,0.7);
		for (int i = 0; i < m_size ; i+=a)
		{
			glBegin (GL_LINE_STRIP);
			for (int j = 0; j < m_size ; j++)
				glVertex3fv(&m_data[3*(j+m_size*i)]);
			glEnd();
		}
		for (int j = 0; j < m_size ; j+=a)
		{
			glBegin (GL_LINE_STRIP);
			for (int i = 0; i < m_size ; i++)
				glVertex3fv(&m_data[3*(j+m_size*i)]);
			glEnd();
		}
	}

	void drawPlaneZ(double zval)
	{
		glBegin(GL_QUADS);
			glVertex3f( 1.0, 1.0, zval);
			glVertex3f(-1.0, 1.0, zval);
			glVertex3f(-1.0,-1.0, zval);
			glVertex3f( 1.0,-1.0, zval);
		glEnd();
	}

	void drawBox()
	{
		glBegin(GL_QUADS);
			//right.
			glNormal3f(0,1,0.2f);
			glVertex3f(-1.0f, -1.0f,  0.5f);
			glVertex3f(-1.0f, -1.0f, -0.5f);
			glVertex3f( 1.0f, -1.0f, -0.5f);
			glVertex3f( 1.0f, -1.0f,  0.5f);
			//back left.
			glNormal3f(0,-1,0.2f);
			glVertex3f(-1.0f, 1.0f, -0.5f);
			glVertex3f(-1.0f, 1.0f,  0.5f);
			glVertex3f( 1.0f, 1.0f,  0.5f);
			glVertex3f( 1.0f, 1.0f, -0.5f);
			//left.
			glNormal3f(1,0,0.2f);
			glVertex3f(-1.0f,-1.0f,  0.5f);
			glVertex3f(-1.0f, 1.0f,  0.5f);
			glVertex3f(-1.0f, 1.0f, -0.5f);
			glVertex3f(-1.0f,-1.0f, -0.5f);
			//back right.
			glNormal3f(-1,0,0.2f);
			glVertex3f( 1.0f,-1.0f, -0.5f);
			glVertex3f( 1.0f, 1.0f, -0.5f);
			glVertex3f( 1.0f, 1.0f,  0.5f);
			glVertex3f( 1.0f,-1.0f,  0.5f);
		glEnd();
	}

	void QGLPlot3D::renderOffsetText(double x, double y, double z, const std::string& txt, const QFont& f)
	{
		double factor=0.016;
		if (m_optBig) factor*=1.5;
		double textoffset=factor*txt.length();
		double xoff=textoffset*sin(m_view[1]);
		double yoff=-textoffset*cos(m_view[1]);
		//renderText(x+xoff,y+yoff,z,txt.c_str(),f);
		renderText(x*1.1+xoff,y*1.1+yoff,z,txt.c_str(),f);
	}

	void QGLPlot3D::drawText()
	{
		glColor3f(0.0, 0.0, 0.0);
		QFont f;
		if (m_optBig)
			f=QFont("Times",40);
	//		f=QFont("Times",22);

		// lables and scale X, offset depends on text length.
		if (m_view[1]>3.1)
		{
			renderOffsetText(0.0, -1.3, -0.5, m_label_x.c_str(),f);
			renderOffsetText(-0.85,-1.15, -0.5, toString(m_bounds[0]).c_str(), f);
			renderOffsetText( 0.85,-1.15, -0.5, toString(m_bounds[1]).c_str(), f);
		}
		else
		{
			renderOffsetText(0.0, 1.3, -0.5, m_label_x.c_str(),f);
			renderOffsetText(-0.85, 1.15, -0.5, toString(m_bounds[0]).c_str(), f);
			renderOffsetText( 0.85, 1.15, -0.5, toString(m_bounds[1]).c_str(), f);
		}
		// lables and scale Y
		if (m_view[1]>1.6 && m_view[1]<4.7)
		{
			renderOffsetText(-1.3, 0.0, -0.5, m_label_y.c_str(),f);
			renderOffsetText(-1.15,-0.85, -0.5, toString(m_bounds[2]).c_str(), f);
			renderOffsetText(-1.15, 0.85, -0.5, toString(m_bounds[3]).c_str(), f);
		}
		else
		{
			renderOffsetText(1.3, 0.0, -0.5, m_label_y.c_str(),f);
			renderOffsetText( 1.15,-0.85, -0.5, toString(m_bounds[2]).c_str(), f);
			renderOffsetText( 1.15, 0.85, -0.5, toString(m_bounds[3]).c_str(), f);
		}
		// scale Z
		if (m_view[1]<1.6)
		{
			renderText(-1.05, 1.05,  0.0, toString(0.5*(m_bounds[4]+m_bounds[5])).c_str(), f);
			renderText(-1.05, 1.05, -0.5, toString(m_bounds[4]).c_str(), f);
			renderText(-1.05, 1.05,  0.5, toString(m_bounds[5]).c_str(), f);
		}
		else if (m_view[1]<3.1)
		{
			renderText(-1.05,-1.05,  0.0, toString(0.5*(m_bounds[4]+m_bounds[5])).c_str(), f);
			renderText(-1.05,-1.05, -0.5, toString(m_bounds[4]).c_str(), f);
			renderText(-1.05,-1.05,  0.5, toString(m_bounds[5]).c_str(), f);
		}
		else if (m_view[1]<4.7)
		{
			renderText( 1.05,-1.05,  0.0, toString(0.5*(m_bounds[4]+m_bounds[5])).c_str(), f);
			renderText( 1.05,-1.05, -0.5, toString(m_bounds[4]).c_str(), f);
			renderText( 1.05,-1.05,  0.5, toString(m_bounds[5]).c_str(), f);
		}
		else
		{
			renderText( 1.05, 1.05,  0.0, toString(0.5*(m_bounds[4]+m_bounds[5])).c_str(), f);
			renderText( 1.05, 1.05, -0.5, toString(m_bounds[4]).c_str(), f);
			renderText( 1.05, 1.05,  0.5, toString(m_bounds[5]).c_str(), f);
		}
	}

	void QGLPlot3D::paintGL()
	{
		int i, j;
		if (m_optBig)
			glClearColor(1.0f,1.0f,1.0f,1.0f);
		else
			glClearColor(0.9f,0.9f,0.9f,1.0f);
		glClearStencil(0);
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
		if ((m_data == 0) || (m_normals == 0)) return;

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();


		glTranslated(-.2,.1,0); // small offset to left-top because of lables

		// Camera
		if (m_optOrtho)
		{
			glTranslatef(0,0,-15.0);
			double scale=5.0/m_view[2];
			glScalef(scale,scale,scale);
		}
		else
			glTranslatef(0,0,-m_view[2]);
	
		if (m_optMovingLight)
		{
			float light[]={-1,-1,-1,0};
			glLightfv(GL_LIGHT0, GL_POSITION, light);
		}

		double rad2deg=57.2957795;
		glRotatef(rad2deg*m_view[0],1,0,0);
		glRotatef(-rad2deg*m_view[1]-90.0f,0,1,0);
		glRotatef(-90.0,1,0,0);

		// En-/Disable smooth shading
		if (m_optFlatShaded)
			glShadeModel(GL_FLAT);
		else
			glShadeModel(GL_SMOOTH);

		// Draw bounding box
		glEnable(GL_LIGHTING);
		glDepthMask(0);
		glColor3f(1,1,1);
		glPolygonMode(GL_FRONT,GL_LINE);
		drawBox();
		glDisable(GL_LIGHTING);
		glDepthMask(1);
		glPolygonMode(GL_BACK,GL_LINE);
		if (m_optBig)
			glLineWidth(4);
		else
			glLineWidth(2);
		glColor3f(0.2f, 0.2f, 0.2f);
		drawBox();
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

		// Draw letters for x and y axes & origin
		if (!m_optNoNumbers) drawText();

		// Draw actual graph
		if (!m_optMovingLight)
			glLightfv(GL_LIGHT0, GL_POSITION, LightSource);
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
		if (m_optLighting)
		{
			glColor3f(1,1,1);
			glEnable(GL_LIGHTING);
		}
		else
		{
			glColor3d(.7,.7,.7);
			glDisable(GL_LIGHTING);
		}
		glBegin(GL_QUADS);
		glColor3d(0.3, 0.7, 0.4);
		for (j = 0; j < m_size - 1; j++) {
			for (i = 0; i < m_size - 1; i++) {
				draw_vertex(i, j+1);
				draw_vertex(i+1, j+1);
				draw_vertex(i+1, j);
				draw_vertex(i, j);
			}
		}
		glEnd();

		// Draw Grid
		glDisable(GL_LIGHTING);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		if (m_optGrid) drawGrid();

		// Draw Iso-lines
		if (m_optIsoLines)
		{
			// draw large x/y-planes, do not mess with depth buffer
			glEnable(GL_DEPTH_TEST);
			glDepthMask(0);
			glPushMatrix();
			glScalef(100,100,1);
			{
				// This block (stencil) is optional, barring the clear color is black.
				glEnable(GL_STENCIL_TEST);
				glStencilOp(GL_ZERO, GL_REPLACE, GL_ZERO);
				glStencilFunc(GL_ALWAYS, 1, 0xffffffff);
				glColorMask(0,0,0,0);
				drawPlaneZ(-1.0);
				glColorMask(1,1,1,1);
				// now keep the stencil buffer as it is
				glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
				// draw only to fragments where stencil is not equal to zero
				glStencilFunc(GL_NOTEQUAL, 0, 0xffffffff);
			}
			glDepthFunc(GL_GREATER);
			glColor4d(0,0,0,.1);
			for (float z=0.f;z<.5;z+=0.1f)
				drawPlaneZ(z);
			glDepthFunc(GL_LEQUAL);
			glEnable(GL_BLEND);
			glDepthMask(0);
			glColor4d(0,0,0,.2);
			for (float z=-.1f;z>-.5;z-=0.1f)
				drawPlaneZ(z);
			glPopMatrix();
			glDepthMask(1);
			glDisable(GL_STENCIL_TEST);
		}
		glColor4f(0,0,0,.5);
		drawPlaneZ(m_isoLevel);
		glDisable(GL_BLEND);

	}

	void QGLPlot3D::mouseMoveEvent(QMouseEvent *event) {
		double dx = event->x() - m_mouse_drag[0];
		double dy = event->y() - m_mouse_drag[1];
		m_mouse_drag[0] = event->x();
		m_mouse_drag[1] = event->y();

		if (event->buttons()&Qt::LeftButton)
		{
			m_view[0]+=dy*0.005;
			m_view[1]-=dx*0.005;        
			// Make sure rotations remain in range.
			if (m_view[0]>1.5)
				m_view[0]=1.5;
			if (m_view[0]<-1.5)
				m_view[0]=-1.5;
			if (m_view[1]<0) m_view[1]+=6.28318531;
			if (m_view[1]>6.28318531) m_view[1]-=6.28318531;
			updateGL();
		}
	}

	void QGLPlot3D::wheelEvent(QWheelEvent *event)
	{
		m_view[2]-=event->delta()*0.005;
		if (m_view[2]<1)
			m_view[2]=1;
		updateGL();
	}

	void QGLPlot3D::keyPressEvent(QKeyEvent *event) {
		int key=event->key();

		if (key==Qt::Key_Up||key==Qt::Key_Down)
		{
			if (key==Qt::Key_Up)
				m_view[0]+=0.01;
			else
				m_view[0]-=0.01;
			if (m_view[0]>1.5)
				m_view[0]=1.5;
			if (m_view[0]<-1.5)
				m_view[0]=-1.5;
		}
		if (key==Qt::Key_Right||key==Qt::Key_Left)
		{
			if (key==Qt::Key_Up)
				m_view[1]+=0.01;
			else
				m_view[1]-=0.01;
			if (m_view[1]<0) m_view[1]+=6.28318531;
			if (m_view[1]>6.28318531) m_view[1]-=6.28318531;
		}
		if (key == 'G') // show or hide black grid overlay
			m_optGrid=!m_optGrid;
		if (key == 'V') // reduce number of subdivisions
			m_optHalfGrid=!m_optHalfGrid;
		if (key == 'C') // toggle heatmap colors
			m_optColors=!m_optColors;
		if (key == 'F') // toggle flat shading
			m_optFlatShaded=!m_optFlatShaded;
		if (key == 'L') // toggle lighting
			m_optLighting=!m_optLighting;
		if (key == 'I') // show iso-lines
			m_optIsoLines=!m_optIsoLines;
		if (key == 'P') // orthographic projection or perspective
		{
			m_optOrtho=!m_optOrtho;
			resizeGL(width(),height());
		}
		if (key == 'N') // axes lables with scales
			m_optNoNumbers=!m_optNoNumbers;
		if (key == 'B') // big front and thick lines
		{
			this->resize(1200,1000);
			m_optBig=!m_optBig;
		}
		if (key == 'M') // big front and thick lines
			m_optMovingLight=!m_optMovingLight;

		if (key=='T') // show threshold plane
		{
			if (m_isoLevel==-.5)
				m_isoLevel=0;
			else
				m_isoLevel=-.5;
		}
		if (key=='+') // move threshold plane
			m_isoLevel+=0.05*(m_bounds[5]-m_bounds[4]);
		if (key=='-') // move threshold plane
			m_isoLevel-=0.05*(m_bounds[5]-m_bounds[4]);
		if (m_isoLevel<-.5)
			m_isoLevel=-.5;
		if (m_isoLevel>.5)
			m_isoLevel=.5;

		if (key=='R') // reset view
		{
			m_view[0]=0.9;
			m_view[1]=3.6;
			m_view[2]=5.0;
		}

		if (key=='S') // save
		{
			QString path = QFileDialog::getSaveFileName(0x0, "Save QGLPlot3D Plot", ".", "QGLPlot3D Plot (*.m)");
			if (save(path.toStdString()))
				std::cout << "Saved plot." << std::endl;
			else
				std::cerr << "Failed to save plot." << std::endl;
		}

		if (key=='O') // open
		{
			QString path = QFileDialog::getOpenFileName(0x0, "Open QGLPlot3D Plot", ".", "QGLPlot3D Plot (*.m *.plot)");
			if (!load(path.toStdString()))
				std::cerr << "Failed to load plot." << std::endl;
		}

		if (key=='X')
			this->grabFrameBuffer().save(QFileDialog::getSaveFileName(this,"Export Image File",".", "Portable Network Graphis (*.png);;JPEG (*jpg);;All Files (*)"));

		if (key=='H')
		{
			std::cout << "###\nUsage:\n\n"
			" \"o\" - Open...\n"
			" \"s\" - Save...\n"
			" \"x\" - Export Image...\n"
			" \"a\" - Create animation (console)...\n"
			" \"r\" - Reset View\n"
			"\n"
			" \"g\" - Show grid\n"
			" \"v\" - Fine grid on/off\n"
			" \"c\" - Color code heatmap or just gray\n"
			" \"f\" - Smooth or flat shading\n"
			" \"l\" - Light on/off\n"
			" \"m\" - Moving Light on/off\n"
			" \"i\" - Show 10 iso-levels by shading\n"
			"\n"
			" \"p\" - Projective or orthographic projection\n"
			" \"n\" - Print scale next to axes\n"
			" \"b\" - Thicker lines and larger text\n"
			"\n"
			" \"t\" - z-value of threshold plane\n"
			" \"+\"/\"-\" - adjust z-value of plane\n"
			" \"h\" - Print this help text\n###\n";
		}

		// hack for animations...
		if (key == 'A')
		{
			int n;
			int deg;
			int x,y;
			std::cout << "Create an animation from plot.\n";
			std::cout << "Type number of frames: (<=0 to cancel)";
			std::cin >> n;
			if (n<=0) return;
			std::cout << "How many degrees? (<=0 for complete rotation)";
			std::cin >> deg;
			if (deg<=0) deg=360;
			std::cout << "Image size [px]";
			std::cin >> x >> y;
			double r=6.28318531*(double)deg/360.0;
			double oldview=m_view[1];
			for (int i=0;i<n;i++)
			{
				m_view[1]=oldview+(double)i/n*r;
				if (m_view[1]>6.28318531) m_view[1]-=6.28318531;
				QPixmap::grabWidget(this,this->rect()).save((std::string("plotRot")+toString(1000+i)+".png").c_str());
			}
			m_view[1]=oldview;
		}

		updateGL();
	}

	const float QGLPlot3D::LightSource[] = {0.5f, 5.0f, -5.0f, 0.0f};
	const float QGLPlot3D::LightAmbient[] = {0.3f, 0.3f, 0.3f, 1.0f};
	const float QGLPlot3D::LightWhite[] = {0.75f, 0.75f, 0.75f, 1.0f};

	const float QGLPlot3D::CMAP[] = {
		0.000000, 0.000000, 0.562500,
		0.000000, 0.000000, 0.625000,
		0.000000, 0.000000, 0.687500,
		0.000000, 0.000000, 0.750000,
		0.000000, 0.000000, 0.812500,
		0.000000, 0.000000, 0.875000,
		0.000000, 0.000000, 0.937500,
		0.000000, 0.000000, 1.000000,
		0.000000, 0.062500, 1.000000,
		0.000000, 0.125000, 1.000000,
		0.000000, 0.187500, 1.000000,
		0.000000, 0.250000, 1.000000,
		0.000000, 0.312500, 1.000000,
		0.000000, 0.375000, 1.000000,
		0.000000, 0.437500, 1.000000,
		0.000000, 0.500000, 1.000000,
		0.000000, 0.562500, 1.000000,
		0.000000, 0.625000, 1.000000,
		0.000000, 0.687500, 1.000000,
		0.000000, 0.750000, 1.000000,
		0.000000, 0.812500, 1.000000,
		0.000000, 0.875000, 1.000000,
		0.000000, 0.937500, 1.000000,
		0.000000, 1.000000, 1.000000,
		0.062500, 1.000000, 0.937500,
		0.125000, 1.000000, 0.875000,
		0.187500, 1.000000, 0.812500,
		0.250000, 1.000000, 0.750000,
		0.312500, 1.000000, 0.687500,
		0.375000, 1.000000, 0.625000,
		0.437500, 1.000000, 0.562500,
		0.500000, 1.000000, 0.500000,
		0.562500, 1.000000, 0.437500,
		0.625000, 1.000000, 0.375000,
		0.687500, 1.000000, 0.312500,
		0.750000, 1.000000, 0.250000,
		0.812500, 1.000000, 0.187500,
		0.875000, 1.000000, 0.125000,
		0.937500, 1.000000, 0.062500,
		1.000000, 1.000000, 0.000000,
		1.000000, 0.937500, 0.000000,
		1.000000, 0.875000, 0.000000,
		1.000000, 0.812500, 0.000000,
		1.000000, 0.750000, 0.000000,
		1.000000, 0.687500, 0.000000,
		1.000000, 0.625000, 0.000000,
		1.000000, 0.562500, 0.000000,
		1.000000, 0.500000, 0.000000,
		1.000000, 0.437500, 0.000000,
		1.000000, 0.375000, 0.000000,
		1.000000, 0.312500, 0.000000,
		1.000000, 0.250000, 0.000000,
		1.000000, 0.187500, 0.000000,
		1.000000, 0.125000, 0.000000,
		1.000000, 0.062500, 0.000000,
		1.000000, 0.000000, 0.000000,
		0.937500, 0.000000, 0.000000,
		0.875000, 0.000000, 0.000000,
		0.812500, 0.000000, 0.000000,
		0.750000, 0.000000, 0.000000,
		0.687500, 0.000000, 0.000000,
		0.625000, 0.000000, 0.000000,
		0.562500, 0.000000, 0.000000,
		0.500000, 0.000000, 0.000000
	};

} // namespace UtilsQt
