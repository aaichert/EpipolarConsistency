#include "QCameraView.h"

#include <QTransform>

#include <QPicture>
#include <QPrinter>

QCameraView::QCameraView(QWidget* parent, UtilsQt::ProjectionParameters * proj)
	: QWidget(parent)
	, projection(proj)
	, keyboardFunc(0x0)
	, keyboardUpFunc(0x0)
	, mouseFunc(0x0)
	, motionFunc(0x0)
	, wheelFunc(0x0)
	, passiveMotionFunc(0x0)
	, specialFunc(0x0)
	, specialUpFunc(0x0)
	, resizeFunc(0x0)
	, closeFunc(0x0)
	, draw(0x0)
{
	if (!projection) projection=&default_projection;
}

void QCameraView::paintEvent(QPaintEvent *event)
{
	QPainter p(this);
	p.setRenderHint(QPainter::Antialiasing);

	if (projection)
		P=projection->getProjectionMatrix(width(),height());

	if (draw) draw(p,this);

	graphics.draw(p,P,1);
}

void QCameraView::savePDF(const std::string& path)
{
	// Paint to QPicture to explicitly ask for vector graphics.
	QPicture pic;
	QPainter p(&pic);
	if (draw) draw(p,this);
	graphics.draw(p,P,1);
	p.end();

	// Set up a PDF printer
	QPrinter printer;
	printer.setOutputFormat(QPrinter::PdfFormat);
	printer.setOutputFileName(path.c_str());
	printer.setPageMargins(0,0,0,0,QPrinter::Unit::Millimeter);
	printer.setPaperSize(QSizeF((double)width()/100,(double)height()/100), QPrinter::Unit::Inch);

	// Finally, print to PDF
	QPainter pdf_painter(&printer);
	pdf_painter.drawPicture(0,0,pic);
	pdf_painter.end();
}

void QCameraView::mousePressEvent(QMouseEvent *event)
{
	int x=event->x();
	int y=event->y();
	m_mouseX=x;
	m_mouseY=y;
	m_mouseDown=event->button();
	if (mouseFunc) mouseFunc(event->button()-1,1,x,y); 
}

void QCameraView::mouseReleaseEvent(QMouseEvent *event)
{
	m_mouseDown=0;
	if (mouseFunc) mouseFunc(event->button()-1,0,event->x(), event->y()); 
}

void QCameraView::mouseMoveEvent(QMouseEvent *event)
{
	int x=event->x();
	int y=event->y();
	int dx=m_mouseX-x;
	int dy=m_mouseY-y;
	m_mouseX=x;	
	m_mouseY=y;
	if (m_mouseDown)
	{
		if (projection)
		{
			projection->mouse(dx,dy,m_mouseDown);
			update();
		}
		if (motionFunc) motionFunc(dx,dy, m_mouseDown);
	}
	else
		if (passiveMotionFunc) passiveMotionFunc(dx,dy);
}

void QCameraView::keyPressEvent(QKeyEvent *event)
{
	if (!keyboardFunc) return;
	unsigned char key=0;
	key=(unsigned char)event->text()[0].toLatin1();
	keyboardFunc(key,0,0);
}

void QCameraView::keyReleaseEvent(QKeyEvent *event)
{
	if (!keyboardFunc) return;
	unsigned char key=0;
	key=(unsigned char)event->text()[0].toLatin1();
	keyboardUpFunc(key,0,0);
}
	
void QCameraView::wheelEvent(QWheelEvent *event)
{
	if (projection)
	{
		projection->wheel(event->angleDelta().y());
		update();
	}
	if (wheelFunc)
		wheelFunc(event->angleDelta().x(),event->angleDelta().y());
}

void QCameraView::resizeEvent(QResizeEvent *event)
{
	if (resizeFunc) resizeFunc(width(),height());
}

void QCameraView::closeEvent(QCloseEvent *event)
{
	if (closeFunc) closeFunc();
}

void QCameraView::setKeyboardFunc(void (*f)(unsigned char key, int x, int y)) {keyboardFunc=f;}
void QCameraView::setKeyboardUpFunc(void (*f)(unsigned char key, int x, int y)) {keyboardUpFunc=f;};
	
void QCameraView::setSpecialFunc(void (*f)(int key, int x, int y)) {specialFunc=f;}
void QCameraView::setSpecialUpFunc(void (*f)(int key, int x, int y)) {specialUpFunc=f;}

void QCameraView::setMouseFunc(void (*f)(int button, int state, int x, int y)) {mouseFunc=f;}
void QCameraView::setMotionFunc(void (*f)(int dx, int dy, int last_button)) {motionFunc=f;}
void QCameraView::setPassiveMotionFunc(void (*f)(int x, int y)) {passiveMotionFunc=f;}
void QCameraView::setWheelFunc(void (*f)(int anglex, int angley)) {wheelFunc=f;}

void QCameraView::setResizeFunc(void (*f)(int w, int h)) {resizeFunc=f;}
void QCameraView::setCloseFunc(void (*f)()) {closeFunc=f;}
