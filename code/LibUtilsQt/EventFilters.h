#ifndef __event_filters_h
#define __event_filters_h

#include <QWidget>
#include <QEvent>

namespace UtilsQt {

	/// An object attached to a widget to capture mouse events.
	class EventFilterMouse : public QObject {
		Q_OBJECT

		int mouse_x;
		int mouse_y;
		bool left_button;

		bool eventFilter(QObject* obj, QEvent* ev) Q_DECL_OVERRIDE ;
	public:
		Q_SIGNAL void mouse_click       (int x, int y, bool left_button, bool release);
		Q_SIGNAL void mouse_double_click(int x, int y, bool left_button);
		Q_SIGNAL void mouse_move        (int x, int y, int dx, int dy);
		Q_SIGNAL void mouse_drag        (int x, int y, int dx, int dy);
		Q_SIGNAL void mouse_wheel       (int dx, int dy, bool shift, bool ctrl);

		bool is_left_button() { return left_button; }

		EventFilterMouse(QWidget* parent);
		~EventFilterMouse();
	};
	
	/// An object attached to a widget to capture window closed and resize events.
	class EventFilterWindow : public QObject {
		Q_OBJECT
		
		bool eventFilter(QObject* obj, QEvent* ev) Q_DECL_OVERRIDE ;
	public:
		Q_SIGNAL void closed();
		Q_SIGNAL void resize(int w, int h);
		Q_SIGNAL void activation_change(bool becomes_active);

		EventFilterWindow(QWidget* parent);
		~EventFilterWindow();
	};
	
	
} // namespace UtilsQt
	
#endif // __event_filters_h
	