#include "EventFilters.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QCloseEvent>

namespace UtilsQt {

	EventFilterMouse::EventFilterMouse(QWidget * parent) : QObject(parent) {
		parent->installEventFilter(this);
		parent->setMouseTracking(true);
	}

	EventFilterMouse::~EventFilterMouse() { disconnect(); }

	bool EventFilterMouse::eventFilter(QObject * obj, QEvent * ev)
	{
		if (!obj->isWidgetType()) return false;
		QMouseEvent* me=dynamic_cast<QMouseEvent*>(ev);
		if (!me) {
			QWheelEvent *we=dynamic_cast<QWheelEvent*>(ev);
			if (we){
				bool mod_shit=we->modifiers() & Qt::ShiftModifier;
				bool mod_ctrl=we->modifiers() & Qt::ControlModifier;
				emit mouse_wheel(we->angleDelta().x(),we->angleDelta().y(), mod_shit, mod_ctrl);
			}
			return false;
		}
		int x=me->x();
		int y=me->y();
		int dx=mouse_x-x;
		int dy=mouse_y-y;
		mouse_x=x;
		mouse_y=y;
		left_button=me->buttons() & Qt::LeftButton;
		mouse_x=x;	
		mouse_y=y;
		if      (ev->type() == QEvent::MouseButtonPress)    emit mouse_click(x,y,left_button, false);
		else if (ev->type() == QEvent::MouseButtonRelease)  emit mouse_click(x,y,left_button, true);
		else if (ev->type() == QEvent::MouseButtonDblClick) emit mouse_double_click(x,y,left_button);
		else if (ev->type() == QEvent::MouseMove && me->buttons()==0) emit mouse_move(x,y,dx,dy);
		else if (ev->type() == QEvent::MouseMove && me->buttons()!=0) emit mouse_drag(x,y,dx,dy);
		return false;
	}
	
	EventFilterWindow::EventFilterWindow(QWidget * parent) : QObject(parent) {
		parent->installEventFilter(this);
	}

	EventFilterWindow::~EventFilterWindow() { disconnect(); }

	bool EventFilterWindow::eventFilter(QObject * obj, QEvent * ev)
	{
		if (!obj->isWidgetType()) return false;
		QCloseEvent* ce=dynamic_cast<QCloseEvent*>(ev);
		if (ce) emit closed();
		else if (ev->type()==QEvent::WindowActivate)
			emit activation_change(true);
		else if (ev->type()==QEvent::WindowDeactivate)
			emit activation_change(true);
		else
		{
			QResizeEvent* re=dynamic_cast<QResizeEvent*>(ev);
			if (re) resize(re->size().width(), re->size().height());
		}
		return false;
	}

} // namespace UtilsQt
