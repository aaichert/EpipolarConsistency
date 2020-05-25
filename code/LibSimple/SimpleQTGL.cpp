#include "SimpleQTGL.h"

void SimpleQTGL::lazyUpdate()
{
	if (redisplay)
	{
		if (m_frameTime.elapsed()<30)
		{
			m_timer.start();
			return;
		}
		redisplay=false;
		updateGL();
	}
	else if (idleFunc)
		idleFunc();
}

void SimpleQTGL::setIdleFunc(void (*f)(void))
{
	idleFunc=f;
	m_timer.start();
}

void SimpleQTGL::postRedisplay()
{
	if (!redisplay)
	{
		redisplay=1;
		m_timer.start();
	}
}

static int g_texID=0;
void showTexture()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glClearColor(1,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glBindTexture(GL_TEXTURE_2D, g_texID);
	glColor3f(1,1,1);
	glBegin(GL_QUADS);
		glTexCoord2i(0,1); glVertex2i(-1,1);
		glTexCoord2i(1,1); glVertex2i(1,1);
		glTexCoord2i(1,0); glVertex2i(1,-1);
		glTexCoord2i(0,0); glVertex2i(-1,-1);
	glEnd();
	glPopAttrib();
}

void SimpleQTGL::immediateShowTexture(int id)
{
	g_texID=id;
	void (*d)()=showTexture;
	std::swap(d,displayFunc);
	immediateUpdate();
	std::swap(d,displayFunc);
}

void SimpleQTGL::immediateUpdate()
{
	if (!displayFunc || !isVisible() ) return;
	if (contextOwner)
		makeCurrent();
	updateGL();
	if (contextOwner)
		contextOwner->makeCurrent();
}


SimpleQTGL::SimpleQTGL()
	: QGLWidget()
	, contextOwner(0x0)
	, displayFunc(0x0)
	, reshapeFunc(0x0)
	, keyboardFunc(0x0)
	, keyboardUpFunc(0x0)
	, mouseFunc(0x0)
	, motionFunc(0x0)
	, passiveMotionFunc(0x0)
	, idleFunc(0x0)
	, specialFunc(0x0)
	, specialUpFunc(0x0)
	, m_mouseDown(0)
{
	redisplay=false;
	m_timer.setInterval(10);
	m_timer.setSingleShot(true);
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(lazyUpdate()));
}

SimpleQTGL::SimpleQTGL(QGLWidget *mainWnd)
	: QGLWidget(mainWnd->context()->format(), 0x0, mainWnd)
	, contextOwner(mainWnd)
	, displayFunc(0x0)
	, reshapeFunc(0x0)
	, keyboardFunc(0x0)
	, keyboardUpFunc(0x0)
	, mouseFunc(0x0)
	, motionFunc(0x0)
	, wheelFunc(0x0)
	, passiveMotionFunc(0x0)
	, idleFunc(0x0)
	, specialFunc(0x0)
	, specialUpFunc(0x0)
	, m_mouseDown(0)
{
	redisplay=false;
	m_timer.setInterval(10);
	m_timer.setSingleShot(true);
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(lazyUpdate()));	
}

SimpleQTGL::~SimpleQTGL() {}

void SimpleQTGL::initializeGL()
{
	if (!gladLoadGL()) {
		std::cerr << "fatal: Failed to initialize OpenGL.\n" << __FILE__ << std::endl;
		exit(1);
	}
}

void SimpleQTGL::resizeGL( int width, int height )
{
	glViewport (0, 0, (GLsizei) width, (GLsizei) height); 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (reshapeFunc)
		reshapeFunc(width, height);
}

void SimpleQTGL::paintGL()
{
	m_frameTime.restart();
	if (displayFunc) displayFunc();
}

void SimpleQTGL::closeEvent(QCloseEvent *event) {if (!contextOwner) exit(0);}

void SimpleQTGL::mousePressEvent(QMouseEvent *event)
{
	int x=event->x();
	int y=event->y();
	m_mouseX=x;
	m_mouseY=y;
	m_mouseDown=1;
	if (mouseFunc) mouseFunc(event->button()-1,1,x,y); 
}


void SimpleQTGL::mouseReleaseEvent(QMouseEvent *event)
{
	m_mouseDown=0;
	if (mouseFunc) mouseFunc(event->button()-1,0,event->x(), event->y()); 
}

void SimpleQTGL::mouseMoveEvent(QMouseEvent *event)
{
	int x=event->x();
	int y=event->y();
	if (m_mouseDown)
		if (motionFunc) motionFunc(m_mouseX-x,m_mouseY-y);
	else
		if (passiveMotionFunc) passiveMotionFunc(x,y);
	m_mouseX=x;
	m_mouseY=y;
}

void SimpleQTGL::keyPressEvent(QKeyEvent *event)
{
	if (!keyboardFunc) return;
	unsigned char key=0;
	key=(unsigned char)event->text()[0].toLatin1();
	keyboardFunc(key,0,0);
}

void SimpleQTGL::keyReleaseEvent(QKeyEvent *event)
{
	if (!keyboardFunc) return;
	unsigned char key=0;
	key=(unsigned char)event->text()[0].toLatin1();
	keyboardUpFunc(key,0,0);
}

void SimpleQTGL::wheelEvent(QWheelEvent *event)
{
	if (wheelFunc)
		wheelFunc(event->angleDelta().x(),event->angleDelta().y());
}

void SimpleQTGL::setDisplayFunc(void (*f)(void)) {displayFunc=f;}
void SimpleQTGL::setReshapeFunc(void (*f)(int width, int height)) {reshapeFunc=f;}
	
void SimpleQTGL::setKeyboardFunc(void (*f)(unsigned char key, int x, int y)) {keyboardFunc=f;}
void SimpleQTGL::setKeyboardUpFunc(void (*f)(unsigned char key, int x, int y)) {keyboardUpFunc=f;};
	
void SimpleQTGL::setSpecialFunc(void (*f)(int key, int x, int y)) {specialFunc=f;}
void SimpleQTGL::setSpecialUpFunc(void (*f)(int key, int x, int y)) {specialUpFunc=f;}

void SimpleQTGL::setMouseFunc(void (*f)(int button, int state, int x, int y)) {mouseFunc=f;}
void SimpleQTGL::setMotionFunc(void (*f)(int dx, int dy)) {motionFunc=f;}
void SimpleQTGL::setWheelFunc(void (*f)(int dx, int dy)) {wheelFunc=f;}
void SimpleQTGL::setPassiveMotionFunc(void (*f)(int x, int y)) {passiveMotionFunc=f;}
