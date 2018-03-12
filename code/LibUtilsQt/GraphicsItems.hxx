#ifndef __graphics_items_hxx
#define __graphics_items_hxx

#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>
#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibProjectiveGeometry/GeometryVisualization.hxx>
#include <LibProjectiveGeometry/EigenToStr.hxx>

#include "ProjectionParameters.hxx"

#include <QPainter>
// FIXME DOCUMENTATION

class GraphicsItem {
public:
	virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification) = 0;
	virtual GraphicsItem* clone() const = 0;
};

namespace GraphicsItems
{

	inline QColor colorByIndex(int i, int alpha=255) {
		i=i+1;										// avoid black
		if (i==8) return QColor(255,128,128,alpha); // avoid white
		int scale_r=255-128*((i/8 )%2), scale_g=255-128*((i/16)%2), scale_b=255-128*((i/32)%2);
		int       r=scale_r*(i%2),            g=scale_g*((i/2)%2),        b=scale_b*((i/4)%2);
		return QColor(r,g,b,alpha);
	}

	class SubFigure;
	
	/// A Collection of unnamed GraphicsItems.
	template <class GraphicsItemType>
	struct Collection : public GraphicsItem
	{
		std::vector<GraphicsItemType> items;

		Collection<GraphicsItemType>() {}

		Collection<GraphicsItemType>(const Collection<GraphicsItemType>& other) {
			this->items=other.items;
		}

		virtual Collection<GraphicsItemType>* clone() const {
			return new Collection<GraphicsItemType>(*this);
		}

		virtual void draw(QPainter& ptr, const Geometry::ProjectionMatrix& P, double magnification)
		{
			for (int i=0;i<(int)items.size();i++)
				items[i].draw(ptr,P,magnification);
		}
	};


	/// A Group of named GraphicsItems.
	class Group : public GraphicsItem {
		std::map<std::string, GraphicsItem*> items;
		Geometry::RP3Homography T;
	public:


		Group(const Geometry::RP3Homography& _T=Geometry::RP3Homography::Identity()) : GraphicsItem(), T(_T) {}
		
		~Group()
		{
			for (auto it=items.begin();it!=items.end();++it)
				delete it->second;
		}
		
		Group(const Group& other)
		{
			for (auto it=other.items.begin();it!=other.items.end();++it)
				items[it->first]=it->second->clone();
			T=other.T;
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			Geometry::ProjectionMatrix PT=P*T;
			for (auto it=items.begin();it!=items.end();++it)
				it->second->draw(p,PT,magnification);
		}

		/// Delete all items in this group
		Group& clear()
		{
			for (auto it=items.begin();it!=items.end();++it)
				delete it->second;
			items.clear();
			return *this;
		}

		/// Erase element by key.
		void erase(const std::string& key)
		{
			auto it=items.find(key);
			if (it!=items.end())
			{
				delete it->second;
				items.erase(it);
			}
		}

		/// Access or create a group with a certain name
		Group& group(const std::string& key)
		{
			Group* existing=get<Group>(key);
			if (existing) return *existing;
			else return set(key,Group());
		}

		/// Create a subfigure
		inline SubFigure& subfigure(const std::string& key, double x, double y, double sx, double sy, bool is_placement_relative=true);

		/// Access GraphicsItem at key or return null if it does not exist. Optional template argument allows an automatic dynamic_cast to a derived type.
		template<class GraphicsItemDerived=GraphicsItem> 
		GraphicsItemDerived* get(const std::string& key) const
		{
			auto it=items.find(key);
			if (it==items.end()) return 0x0;
			return dynamic_cast<GraphicsItemDerived*>(it->second);
		}

		/// Makes a copy of object and stores it. Reference to new object is returned.
		template<class GraphicsItemDerived> 
		GraphicsItemDerived& set(const std::string& key, const GraphicsItemDerived& obj)
		{
			erase(key);
			GraphicsItemDerived* ret=(GraphicsItemDerived*)obj.clone();
			items[key]=ret;
			return *ret;
		}

		/// Add an item with a unique name. Prefix is optional.
		Group& add(const GraphicsItem& obj, const std::string& prefix="Item")
		{
			// Linear (slow) search for free spot...
			int i=0;
			while (items.find(prefix+toString(i,4,' '))!=items.end()) i++;
			items[prefix+toString(i,4,' ')]=obj.clone();
			return *this;
		}

		/// Get current 3D transformation of items in group. (does not affect 2D items)
		const Geometry::RP3Homography& getTransform() const { return T;}

		/// Set 3D transformation of items in group. (does not affect 2D items)
		Group&  setTransform(const Geometry::RP3Homography& _T) { T=_T; return *this; }

		virtual GraphicsItem* clone() const
		{
			return new Group(*this);
		}

		Group& operator=(const Group& other)
		{
			if (this!=&other)
			{
				Group copy(other);
				std::swap(items,copy.items);
			}
			return *this;
		}

	};

	/// A sub-figure with different projection and/or size
	class SubFigure : public Group {
	public:
		double pos_x;
		double pos_y;
		double size_x;
		double size_y;
		bool   placement_relative;
		bool   different_projection;
		UtilsQt::ProjectionParameters projection;

		SubFigure(double x, double y, double sx, double sy, bool is_placement_relative=true)
			: Group()
			, pos_x(x)
			, pos_y(y)
			, size_x(sx)
			, size_y(sy)
			, placement_relative(is_placement_relative)
			, different_projection(false)
		{}

		SubFigure(const SubFigure& other) : Group(other) {
			pos_x                = other.pos_x               ;
			pos_y                = other.pos_y               ;
			size_x               = other.size_x              ;
			size_y               = other.size_y              ;
			placement_relative   = other.placement_relative  ;
			different_projection = other.different_projection;
			projection           = other.projection          ;
		}

		virtual GraphicsItem* clone() const
		{
			return new SubFigure(*this);
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			double w=p.device()->width();
			double h=p.device()->height();
			double tx=placement_relative? w*pos_x : pos_x*magnification ;
			double ty=placement_relative? h*pos_y : pos_y*magnification ;
			double sx=placement_relative? w*size_x: size_x*magnification;
			double sy=placement_relative? h*size_y: size_y*magnification;
			double s =std::min(sx/w,sy/h);
			if (different_projection)
				Group::draw(p,Geometry::Translation(tx,ty)*projection.getProjectionMatrix(sx/magnification,sy/magnification), magnification);
			else Group::draw(p,Geometry::Translation(tx,ty)*Geometry::Scale(s,s)*P,magnification);
		}

	};

	SubFigure& Group::subfigure(const std::string& key, double x, double y, double sx, double sy, bool is_placement_relative)
	{
		erase(key);
		return set(key,SubFigure(x,y,sx,sy,is_placement_relative));
	}

	enum MarkerShape {
		Point,     //< fast and simple square of pixels
		Circle,
		BoxFrame,
		BoxFilled, //< filled.
		Dot,       //< filled.
		Cross,
		Plus,
	};
	
	struct Point2D : public GraphicsItem
	{
		MarkerShape marker;
		Eigen::Vector2d x;
		double radius;
		QColor color;
		double width;

		Point2D(double px=0, double py=0, double _radius=3, const QColor& _color=QColor(0,0,0,255), MarkerShape _marker=Point, double _width=1)
			: marker(_marker)
			, x(px,py)
			, radius(_radius)
			, color(_color)
			, width(_width)
		{}

		virtual Point2D* clone() const {
			return new Point2D(*this);
		}

		virtual void draw(QPainter& ptr, const Geometry::ProjectionMatrix& P, double magnification)
		{
			if (marker==Point) {
				QPen pen=color;
				pen.setWidth(radius*magnification);
				ptr.drawPoint(QPointF(x[0]*magnification,x[1]*magnification));
				return;
			}
			QPen pen=color;
			pen.setWidth(width*magnification);
			ptr.setPen(pen);
			if (marker==Dot||marker==BoxFilled)
				ptr.setBrush(color);
			else
				ptr.setBrush(QBrush(Qt::NoBrush));
			if (marker==Circle||marker==Dot)
				ptr.drawEllipse(QPointF(x[0]*magnification,x[1]*magnification),radius*magnification,radius*magnification);
			else if (marker==BoxFilled||marker==BoxFrame)
				ptr.drawRect(QRectF(x[0]*magnification-radius*magnification,x[1]*magnification-radius*magnification,2.0*radius*magnification,2.0*radius*magnification));
			else if (marker==Plus) {
				ptr.drawLine(x[0]*magnification,x[1]*magnification-radius*magnification,x[0]*magnification,x[1]*magnification+radius*magnification);
				ptr.drawLine(x[0]*magnification-radius*magnification,x[1]*magnification,x[0]*magnification+radius*magnification,x[1]*magnification);
			}
			else if (marker==Cross) {
				ptr.drawLine(x[0]*magnification-radius*magnification,x[1]*magnification-radius*magnification,x[0]*magnification+radius*magnification,x[1]*magnification+radius*magnification);
				ptr.drawLine(x[0]*magnification+radius*magnification,x[1]*magnification-radius*magnification,x[0]*magnification-radius*magnification,x[1]*magnification+radius*magnification);
			}
		}
	};

	struct Point3D : public Point2D
	{
		Geometry::RP3Point X;
		
		Point3D(const Geometry::RP3Point& _X, double _radius=3, const QColor& _color=QColor(0,0,0,255), MarkerShape _marker=Point)
			: Point2D(0,0,_radius,_color,_marker)
			, X(_X)
		{}

		virtual Point3D* clone() const {
			return new Point3D(*this);
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			x=Geometry::euclidian2(P*X);
			Point2D::draw(p,P,magnification);
		}
	};

	struct Rectangle : public GraphicsItem
	{
		Eigen::Vector2d x;
		Eigen::Vector2d size;
		QColor color;
		
		Rectangle(double px=1, double py=1, double width=1, double height=1, const QColor& _color=QColor(255,255,255,128))
			: x(px,py)
			, size(width,height)
			, color(_color)
		{}

		virtual GraphicsItem* clone() const {
			return new Rectangle(*this);
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			p.setPen(color);
			p.setBrush(color);
			p.drawRect(x[0]*magnification, x[1]*magnification, size[0]*magnification, size[1]*magnification);
		}

	};

	struct Text2D : public GraphicsItem
	{
		std::string text;
		Eigen::Vector2d x;
		QColor color;
		QFont font;

		Text2D(const std::string& _text="", double px=0, double py=0, const QColor& _color=QColor(0,0,0,255), const QFont& _font=QFont())
			: GraphicsItem()
			, x(px,py)
			, text(_text)
			, color(_color)
			, font(_font)
		{}
		
		Text2D(const std::string& _text, const Eigen::Vector2d& _x, const QColor& _color=QColor(0,0,0,255), const QFont& _font=QFont())
			: GraphicsItem()
			, text(_text)
			, x(_x)
			, color(_color)
			, font(_font)
		{}

		virtual GraphicsItem* clone() const {
			return new Text2D(*this);
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			if (text.empty()|| !std::isfinite(x[0]) || !std::isfinite(x[1]))
				return;
			p.setPen(color);
			QFont font_mag=font;
			font_mag.setPointSizeF(font.pointSizeF()*magnification);
			p.setFont(font_mag);
			p.drawText(QPoint(x[0]*magnification, x[1]*magnification),text.c_str());
		}

	};

	struct Text3D : public Text2D {

		Geometry::RP3Point X;

		Text3D(const std::string& _text="", const Geometry::RP3Point& _X=Geometry::RP3Point(0,0,0,0), const QColor& _color=QColor(0,0,0,255), const QFont& _font=QFont())
			: Text2D(_text, Eigen::Vector2d::Zero(), _color, _font)
			, X(_X)
		{}

		virtual GraphicsItem* clone() const {
			return new Text3D(*this);
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			x=Geometry::euclidian2(P*X);
			x+=Eigen::Vector2d(15,15);
			Text2D::draw(p,P,magnification);
		}

	};

	struct Line2D : public GraphicsItem {
		Eigen::Vector2d a;
		Eigen::Vector2d b;
		double width;
		QColor color;

		Line2D(double ax=0, double ay=0, double bx=0, double by=0, double _width=1, QColor _color=QColor(0,0,255))
			: a(ax,ay)
			, b(bx,by)
			, width(_width)
			, color(_color)
		{}

		virtual GraphicsItem* clone() const {
			return new Line2D(*this);
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			QPen pen(color);
			pen.setWidth(width*magnification);
			p.setPen(pen);
			p.drawLine(QPointF(a[0]*magnification,a[1]*magnification),QPointF(b[0]*magnification,b[1]*magnification));
		}
	};

	struct Line3D : public Line2D {
		Geometry::RP3Point A;
		Geometry::RP3Point B;

		Line3D(Geometry::RP3Point _A, Geometry::RP3Point _B, double _width=1, QColor _color=QColor(0,0,255))
			: Line2D(0,0,0,0,_width,_color)
			, A(_A)
			, B(_B)
		{}

		virtual GraphicsItem* clone() const {
			return new Line3D(*this);
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			a=Geometry::euclidian2(P*A);
			b=Geometry::euclidian2(P*B);
			Line2D::draw(p,P,magnification);
		}
	};

	/// A 2D line (infinitely long) given in Hessian normal form.
	struct PlueckerLine2D : public Line2D {
		Geometry::RP2Line l;

		PlueckerLine2D(Geometry::RP2Line _l, double _width=1, QColor _color=QColor(0,0,255))
			: Line2D(0,0,0,0,_width,_color)
			, l(_l)
		{}

		virtual GraphicsItem* clone() const {
			return new PlueckerLine2D(*this);
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			int	width =p.viewport().width()/magnification;
			int height=p.viewport().height()/magnification;
			l=l/l.head(2).norm();
			double d[]={l[1],-l[0]};
			double o[]={-l[2]*l[0],-l[2]*l[1]};
			float v[]={
				(        1-o[0])/d[0], (width -1-o[0])/d[0],
				(        1-o[1])/d[1], (height-1-o[1])/d[1]
			};
			// Avoid Inf/NaN
			if (d[0]*d[0]<1e-12) v[0]=-(v[1]=1e10f);
			if (d[1]*d[1]<1e-12) v[2]=-(v[3]=1e10f);
			// Sort
			for (int j=0;j<3;j++)
				for (int i=0;i<3;i++)
					if (v[i]>v[i+1]) {
						float tmp=v[i];
						v[i]=v[i+1];
						v[i+1]=tmp;
					}
			// The middle two are image edges
			a=Eigen::Vector2d(o[0]+d[0]*v[1],o[1]+d[1]*v[1]);
			b=Eigen::Vector2d(o[0]+d[0]*v[2],o[1]+d[1]*v[2]);
			Line2D::draw(p,P,magnification);
		}
	};

	/// A 3D line (infinitely long) defined by Plücker coordinates.
	struct PlueckerLine3D : public PlueckerLine2D {
		Geometry::RP3Line L;

		PlueckerLine3D(Geometry::RP3Line _L, double _width=1, QColor _color=QColor(0,0,255))
			: PlueckerLine2D(Geometry::RP2Line(),_width,_color)
			, L(_L)
		{}

		virtual GraphicsItem* clone() const {
			return new PlueckerLine3D(*this);
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			l=Geometry::pluecker_project(L,P);
			PlueckerLine2D::draw(p,P,magnification);
		}
	};

	/// Arrow from o to x with an optional label
	struct Arrow2D : public Text2D {
		Eigen::Vector2d o;
		double arrow_head_size_px;
		double l_style;

		Arrow2D(const Eigen::Vector2d& _from=Eigen::Vector2d(0,0), const Eigen::Vector2d& _to=Eigen::Vector2d(0,0), const QColor& _color=QColor(0,0,0,255), double _l_style=2,  double _arrow_head_size_px=10, const std::string& _text="", const QFont& _font=QFont())
			: Text2D(_text, _to, _color, _font)
			, o(_from)
			, arrow_head_size_px(_arrow_head_size_px)
			, l_style(_l_style)
		{}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			x+=Eigen::Vector2d(15,15);
			if (text.empty()) text=" ";
			Text2D::draw(p,P,magnification);
			x-=Eigen::Vector2d(15,15);
			QPen pen=p.pen();
			pen.setWidthF(l_style*magnification);
			p.setPen(pen);
			p.drawLine(QPointF(o[0]*magnification,o[1]*magnification),QPointF(x[0]*magnification,x[1]*magnification));
			Eigen::Vector2d dir=(x-o).normalized()*(arrow_head_size_px/(sqrt(2)));
			if (!Geometry::allfinite(dir)) return;
			Eigen::Vector2d ort=Eigen::Vector2d(dir[1],-dir[0]);
			Eigen::Vector2d head0=x-(dir-ort);
			Eigen::Vector2d head1=x-(dir+ort);
			p.drawLine(QPointF(x[0]*magnification,x[1]*magnification),QPointF(head0[0]*magnification,head0[1]*magnification));
			p.drawLine(QPointF(x[0]*magnification,x[1]*magnification),QPointF(head1[0]*magnification,head1[1]*magnification));
		}

		virtual GraphicsItem* clone() const {
			return new Arrow2D(*this);
		}
	};

	struct Arrow3D : public Arrow2D {
		Geometry::RP3Point O;
		Geometry::RP3Point X;
		double arrow_head_size_mm;

		Arrow3D(const Geometry::RP3Point& _from=Geometry::RP3Point(0,0,0,0), const Geometry::RP3Point& _to=Geometry::RP3Point(0,0,0,0), const QColor& _color=QColor(0,0,0,255), double _l_style=2, double _arrow_head_size_mm=10, const std::string& _text="", const QFont& _font=QFont())
			: Arrow2D(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),  _color, _l_style, 0, _text, _font)
			, O(_from)
			, X(_to)
			, arrow_head_size_mm(_arrow_head_size_mm)
		{}

		virtual void draw(QPainter&	p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			o=Geometry::euclidian2(P*O);
			x=Geometry::euclidian2(P*X);
			if (!Geometry::allfinite(o) && !Geometry::allfinite(x))
				return;
			auto U=Geometry::getCameraAxisDirections(P).first.normalized();
			Eigen::Vector2d x_offset=Geometry::euclidian2(P*(X+U*arrow_head_size_mm));
			double line_length=(x-o).norm();
			arrow_head_size_px=(x-x_offset).norm()+3;
			if (arrow_head_size_px>line_length)
			{
				bool is_facing_the_camera=Geometry::getCameraDirection(P).dot(Geometry::euclidian3(X)-Geometry::euclidian3(O))<0;
				Point2D(o[0],o[1], 0.5*(arrow_head_size_px-line_length), color, is_facing_the_camera?MarkerShape::Circle:MarkerShape::Cross,l_style).draw(p,P,magnification);
				arrow_head_size_px=line_length;
			}
			Arrow2D::draw(p,P,magnification);
		}
		
		virtual GraphicsItem* clone() const {
			return new Arrow3D(*this);
		}
	};


	struct CoordinateAxes : public GraphicsItem {
		Arrow3D X;
		Arrow3D Y;
		Arrow3D Z;

		std::string origin_text;

		CoordinateAxes(double length=100, const std::string& _origin="",  const std::string& _x="X", const std::string& _y="Y", const std::string& _z="Z")
			: GraphicsItem()
			, origin_text(_origin)
			, X(Geometry::origin3,Geometry::RP3Point(length,0,0,1),QColor(255,0,0,255),2,length*0.1, _x)
			, Y(Geometry::origin3,Geometry::RP3Point(0,length,0,1),QColor(0,255,0,255),2,length*0.1, _y)
			, Z(Geometry::origin3,Geometry::RP3Point(0,0,length,1),QColor(0,0,255,255),2,length*0.1, _z)
		{}

		virtual void draw(QPainter&	p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			X.draw(p,P,magnification);
			Y.draw(p,P,magnification);
			Z.draw(p,P,magnification);
			if (!origin_text.empty())
				Text3D(origin_text,Geometry::origin3,QColor(128,128,128)).draw(p,P,magnification);
		}

		virtual GraphicsItem* clone() const {
			return new CoordinateAxes(*this);
		}
		
	};

} // namespace GraphicsItems
	
#endif // __graphics_items_hxx
