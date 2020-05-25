#ifndef SIMPLE_QT_GL
#define SIMPLE_QT_GL

#include "GL/glad.h"

#include <iostream>

#include <QtCore/QString>

#include <QtOpenGL/QGLWidget>

#include <QApplication>
#include <QKeyEvent>
#include <QMouseEvent>

#include <QTimer>

#include <QElapsedTimer>

class SimpleQTGL: public QGLWidget
{
	Q_OBJECT

public slots:
	void lazyUpdate();

public:
	// Creates its own independent context
	SimpleQTGL();

	// Shares context of parent
	SimpleQTGL(QGLWidget *mainWnd);

	virtual ~SimpleQTGL();

	bool			redisplay;
	QTimer			m_timer;
	QElapsedTimer	m_frameTime;
	// Mouse state
	int m_mouseDown;
	int m_mouseX;
	int m_mouseY;


protected:
	QGLWidget *contextOwner;

	// Quit on close IF we are contextOwner
	void closeEvent(QCloseEvent *event);

	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseReleaseEvent(QMouseEvent *event);
	virtual void mouseMoveEvent(QMouseEvent *event);
	virtual void keyPressEvent(QKeyEvent *event);
	virtual void keyReleaseEvent(QKeyEvent *event);
	virtual void wheelEvent(QWheelEvent *event);

	virtual void initializeGL();
	virtual void resizeGL( int width, int height );
	virtual void paintGL();
  

protected:
	// function pointers
	void (*displayFunc)(void);
	void (*reshapeFunc)(int width, int height);
	void (*keyboardFunc)(unsigned char key, int x, int y);
	void (*keyboardUpFunc)(unsigned char key, int x, int y);
	void (*mouseFunc)(int button, int state, int x, int y);
	void (*motionFunc)(int dx, int dy);
	void (*wheelFunc)(int dx, int dy);
	void (*passiveMotionFunc)(int x, int y);

	void (*idleFunc)(void);

	void (*specialFunc)(int key, int x, int y);
	void (*specialUpFunc)(int key, int x, int y);

public:
	void setDisplayFunc(void (*f)(void));
	void setReshapeFunc(void (*f)(int width, int height));
	
	void setKeyboardFunc(void (*f)(unsigned char key, int x, int y));
	void setKeyboardUpFunc(void (*f)(unsigned char key, int x, int y));
	
	void setSpecialFunc(void (*f)(int key, int x, int y));
	void setSpecialUpFunc(void (*f)(int key, int x, int y));

	void setMouseFunc(void (*f)(int button, int state, int x, int y));
	void setMotionFunc(void (*f)(int dx, int dy));
	void setWheelFunc(void (*f)(int dx, int dy));
	void setPassiveMotionFunc(void (*f)(int x, int y));

	/// Called when no redisplay is required
	void setIdleFunc(void (*f)(void));

	/// Renders another frame after this one. Multiple calls are fine and do not lead to recursion. 
	void postRedisplay();

	/// Acquires context and immediately draws texture of id.
	void immediateShowTexture(int id);

	/// Acquires context and immediately calls displayFunc.
	void immediateUpdate();

};
#endif // SIMPLE_QT_GL