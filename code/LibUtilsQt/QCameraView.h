#ifndef __camera_view
#define __camera_view

#define WITH_GETSET

#ifdef WITH_GETSET
#include <GetSet/GetSet.hxx>
#endif

#include <QtWidgets/QWidget>
#include <QtGui/QPainter>
#include <QPaintEvent>
#include <QImage>

#include <LibProjectiveGeometry/GeometryVisualization.hxx>
#include <LibProjectiveGeometry/CameraOpenGL.hxx>
#include <LibProjectiveGeometry/EigenToStr.hxx>

#include <GetSet/GetSet.hxx>

#include "ProjectionParameters.hxx"

#include "GraphicsItems.hxx"
#include "GraphicsItemConvexMesh.hxx"

class QCameraView : public QWidget
{
	Q_OBJECT
public:
	Geometry::ProjectionMatrix     P;				//< Projection Matrix for 3D wireframe scene.
	UtilsQt::ProjectionParameters  *projection;		//< Parameters for automatic rotation etc.
	GraphicsItems::Group           graphics;		//< Graphics items automatically drawn.

	QCameraView(QWidget* parent = 0x0, UtilsQt::ProjectionParameters * proj=0x0);

	// Set function pointers
	void setDrawFunc(void (*f)(QPainter&,QCameraView*)) {draw=f;}
	void setKeyboardFunc(void (*f)(unsigned char key, int x, int y));
	void setKeyboardUpFunc(void (*f)(unsigned char key, int x, int y));
	void setSpecialFunc(void (*f)(int key, int x, int y));
	void setSpecialUpFunc(void (*f)(int key, int x, int y));
	void setMouseFunc(void (*f)(int button, int state, int x, int y));
	void setMotionFunc(void (*f)(int x, int y, int last_button));
	void setPassiveMotionFunc(void (*f)(int x, int y));
	void setWheelFunc(void (*f)(int anglex, int angley));
	void setResizeFunc(void (*f)(int w, int h));
	void setCloseFunc(void (*f)());

	virtual void paintEvent(QPaintEvent *event);

	void savePDF(const std::string& path);

private:
	// Mouse state
	int m_mouseDown;
	int m_mouseX;
	int m_mouseY;
	UtilsQt::ProjectionParameters default_projection;

	// Events for call-back mechanism
	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseReleaseEvent(QMouseEvent *event);
	virtual void mouseMoveEvent(QMouseEvent *event);
	virtual void keyPressEvent(QKeyEvent *event);
	virtual void keyReleaseEvent(QKeyEvent *event);
	virtual void wheelEvent(QWheelEvent *event);
	virtual void resizeEvent(QResizeEvent *event);
	virtual void closeEvent(QCloseEvent *event);

	// function pointers
	void (*draw)(QPainter&,QCameraView*);
	void (*keyboardFunc)(unsigned char key, int x, int y);
	void (*keyboardUpFunc)(unsigned char key, int x, int y);
	void (*mouseFunc)(int button, int state, int x, int y);
	void (*motionFunc)(int x, int y, int last_button);
	void (*passiveMotionFunc)(int x, int y);
	void (*specialFunc)(int key, int x, int y);
	void (*specialUpFunc)(int key, int x, int y);
	void (*wheelFunc)(int anglex, int angley);
	void (*resizeFunc)(int w, int h);
	void (*closeFunc)();

};

#endif // __camera_view
