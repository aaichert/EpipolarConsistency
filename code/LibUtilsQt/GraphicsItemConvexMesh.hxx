#ifndef __graphics_item_convex_mesh_hxx
#define __graphics_item_convex_mesh_hxx

#include "GraphicsItems.hxx"

#include <Eigen/Dense>

namespace GraphicsItems
{
	struct ConvexMesh : public GraphicsItem {

		std::vector<Geometry::RP3Point> p_pos;		//< 3D Point Position
		QColor							p_color;	//< 3D Point Color
		double							p_style;	//< 3D Point Style (currently : width)
		std::vector<Eigen::Vector2i>	l_index;	//< 3D Line Indices
		QColor							l_color;	//< 3D Line Color
		double							l_style;	//< 3D Line Style (currently : width)
		std::vector<Eigen::Vector4i>	q_index;	//< Quad Indices
		std::vector<QColor>				q_color;	//< Quad Front Face Color
		QColor							q_color_back;//< Quad Back Face Color
		double							q_style;	//< -1: back lines only, 0: no lines, 1: front lines only. abs(q_style) is lines with.
		
		ConvexMesh()
		{
			clear();
		}

		virtual GraphicsItem* clone() const {
			return new ConvexMesh(*this);
		}

		ConvexMesh& clear()
		{
			p_pos.clear();
			p_color=QColor(255,0,0,255);
			p_style=1;
			l_index.clear();
			l_color=QColor(0,0,255,255);
			l_style=2;
			q_index.clear();
			q_color=std::vector<QColor>(1,QColor(0,255,0,60));
			q_color_back=QColor(0,0,0,0);
			q_style=1;
			return *this;
		}

		virtual void draw(QPainter& p, const Geometry::ProjectionMatrix& P, double magnification)
		{
			// Temporary 2d projections of points
			std::vector<Eigen::Vector2d> p2(p_pos.size());
			#pragma omp parallel for
			for(int i=0;i<(int)p_pos.size();i++)
			{
				Geometry::dehomogenize(p_pos[i]);
				Geometry::RP2Point x=P*p_pos[i];
				if (Geometry::dehomogenize(x))
					p2[i]=Eigen::Vector2d(x[0],x[1]);
				else
					p2[i].setConstant(0); // just bring infinite points to zero...
			}
			
			// Get current state of the pen and brush
			QPen   original_pen=p.pen();
			QBrush original_brush=p.brush();

			// Points
			if (p_color.alpha()>0)
			{
				QPen pen=p_color;
				pen.setWidthF(p_style*magnification);
				p.setPen(pen);
				for(int i=0;i<(int)p_pos.size();i++)
					p.drawPoint(QPointF(p2[i][0]*magnification,p2[i][1]*magnification));
			}
			if (l_color.alpha()>0)
			{
				QPen pen=l_color;
				pen.setWidthF(l_style*magnification);
				p.setPen(pen);
				// Lines
				for(int i=0;i<(int)l_index.size();i++)
					p.drawLine(
						QPointF(p2[l_index[i][0]][0]*magnification,p2[l_index[i][0]][1]*magnification),
						QPointF(p2[l_index[i][1]][0]*magnification,p2[l_index[i][1]][1]*magnification)
						);
			}

			// Draw quads
			if (l_color.alpha()>0)
			{
				int n_q=(int)q_index.size();
				std::vector<bool> polygon_seen_from_behind(n_q);
				auto C=Geometry::getCameraCenter(P);
				// Figure out front-/back-facing polygons
				for(int i=0;i<n_q;i++)
				{
					// Figure out if polygon is facing the camera or not
					Eigen::Vector3d o=p_pos[q_index[i][0]].head(3);
					Eigen::Vector3d a=p_pos[q_index[i][1]].head(3);
					Eigen::Vector3d b=p_pos[q_index[i][2]].head(3);
					Eigen::Vector3d face_normal=(a-o).cross(b-o).normalized();
					Eigen::Vector3d view_dir=Geometry::pluecker_direction(Geometry::join_pluecker(C,p_pos[q_index[i][0]])).normalized();
					polygon_seen_from_behind[i]=face_normal.dot(view_dir)<0;
				}
				// Drawing
				QPen pen=l_color;
				pen.setWidthF(std::abs(q_style)*magnification);
				// Line style and color for back faces
				pen.setStyle(q_style>=0?Qt::PenStyle::NoPen:Qt::PenStyle::SolidLine);
				p.setPen(pen);
				p.setBrush(q_color_back);
				bool draw_backfaces=q_color_back.alpha()>=0 || q_style<0;
				if (draw_backfaces)
					for(int i=0;i<(int)n_q;i++)
					{
						if (!polygon_seen_from_behind[i]) continue;
						// Get projection of corner points
						QPointF polygon[4] = {
							QPointF(p2[q_index[i][0]][0]*magnification,p2[q_index[i][0]][1]*magnification),
							QPointF(p2[q_index[i][1]][0]*magnification,p2[q_index[i][1]][1]*magnification),
							QPointF(p2[q_index[i][2]][0]*magnification,p2[q_index[i][2]][1]*magnification),
							QPointF(p2[q_index[i][3]][0]*magnification,p2[q_index[i][3]][1]*magnification)
						};
						p.drawConvexPolygon(polygon,4);
					}
				// Line style and color for front faces
				pen.setStyle(Qt::PenStyle::SolidLine);
				pen.setColor(q_style<=0?q_color[0]:l_color);
				p.setPen(pen);
				p.setBrush(q_color[0]);
				for(int i=0;i<(int)n_q;i++)
				{
					if (polygon_seen_from_behind[i]) continue;
					if (q_color.size()>1)
					{
						const QColor& c=q_color[i%n_q];
						if (q_style<=0)
						{
							pen.setColor(c);
							p.setPen(c);
						}
						p.setBrush(c);
					}
					// Get projection of corner points
					QPointF polygon[4] = {
						QPointF(p2[q_index[i][0]][0]*magnification,p2[q_index[i][0]][1]*magnification),
						QPointF(p2[q_index[i][1]][0]*magnification,p2[q_index[i][1]][1]*magnification),
						QPointF(p2[q_index[i][2]][0]*magnification,p2[q_index[i][2]][1]*magnification),
						QPointF(p2[q_index[i][3]][0]*magnification,p2[q_index[i][3]][1]*magnification)
					};
					p.drawConvexPolygon(polygon,4);
				}
			}

			// Restore pen and brush style
			p.setPen(original_pen);
			p.setBrush(original_brush);
		}

		static ConvexMesh Triangle(Geometry::RP3Point X0, Geometry::RP3Point X1, Geometry::RP3Point X2, const QColor& line_color=QColor(0,255,0,255),  const QColor& fill_color=QColor(0,255,0,32) )
		{
			ConvexMesh obj;
			obj.clear();
			obj.p_pos.push_back(X0);
			obj.p_pos.push_back(X1);
			obj.p_pos.push_back(X2);
			obj.l_index.push_back(Eigen::Vector2i(0,1));
			obj.l_index.push_back(Eigen::Vector2i(1,2));
			obj.l_index.push_back(Eigen::Vector2i(2,0));
			obj.q_index.push_back(Eigen::Vector4i(0,1,2,2));
			obj.q_index.push_back(Eigen::Vector4i(2,2,1,0));
			obj.q_color.push_back(fill_color);
			obj.q_color.push_back(fill_color);
			obj.q_color_back=fill_color;
			obj.l_color=line_color;
			obj.q_style=0;
			return obj;
		}

		static ConvexMesh WireCube(const Eigen::Vector3d& cube_min=Eigen::Vector3d(-50,-50,-50), const Eigen::Vector3d& cube_max=Eigen::Vector3d(50,50,50), const QColor& color=QColor(0,0,255,255))
		{
			ConvexMesh obj;
			obj.clear();
			obj.p_pos.push_back(Geometry::RP3Point(cube_min[0],cube_min[1],cube_min[2],1));
			obj.p_pos.push_back(Geometry::RP3Point(cube_max[0],cube_min[1],cube_min[2],1));
			obj.p_pos.push_back(Geometry::RP3Point(cube_min[0],cube_max[1],cube_min[2],1));
			obj.p_pos.push_back(Geometry::RP3Point(cube_max[0],cube_max[1],cube_min[2],1));
			obj.p_pos.push_back(Geometry::RP3Point(cube_min[0],cube_min[1],cube_max[2],1));
			obj.p_pos.push_back(Geometry::RP3Point(cube_max[0],cube_min[1],cube_max[2],1));
			obj.p_pos.push_back(Geometry::RP3Point(cube_min[0],cube_max[1],cube_max[2],1));
			obj.p_pos.push_back(Geometry::RP3Point(cube_max[0],cube_max[1],cube_max[2],1));
			for (int a = 0; a < 8; a++)
				for (int b = 0; b < 8; b++)
					if (( (int)(a%2!=b%2) + (int)((a/2)%2!=(b/2)%2) + (int)((a/4)%2!=(b/4)%2))==1 )
						obj.l_index.push_back(Eigen::Vector2i(a,b));
			obj.p_color=QColor(0,0,0,0);
			obj.l_color=color;
			return obj;
		}

		static ConvexMesh Cube(const Eigen::Vector3d& cube_min=Eigen::Vector3d(-50,-50,-50), const Eigen::Vector3d& cube_max=Eigen::Vector3d(50,50,50), const QColor& color=QColor(0,255,0,32), bool lines=false)
		{
			ConvexMesh obj=WireCube(cube_min, cube_max, QColor(0,0,0,50));
			if (lines) obj.l_style=1;
			else obj.l_index.clear();
			obj.q_index.push_back(Eigen::Vector4i(1,0,4,5));
			obj.q_index.push_back(Eigen::Vector4i(1,3,2,0));
			obj.q_index.push_back(Eigen::Vector4i(2,6,4,0));
			obj.q_index.push_back(Eigen::Vector4i(5,7,3,1));
			obj.q_index.push_back(Eigen::Vector4i(6,7,5,4));
			obj.q_index.push_back(Eigen::Vector4i(3,7,6,2));
			obj.q_color.clear();
			obj.q_color.push_back(color);
			obj.q_color_back=QColor(0,0,0,0);
			return obj;
		}

		static ConvexMesh ColorCube(const Eigen::Vector3d& cube_min=Eigen::Vector3d(-50,-50,-50), const Eigen::Vector3d& cube_max=Eigen::Vector3d(50,50,50), const QColor& color=QColor(0,255,0,32), bool lines=false)
		{
			ConvexMesh obj=Cube(cube_min, cube_max, QColor(0,0,0,50), lines);
			obj.q_color.clear();
			obj.q_color.push_back(QColor(128,255,128,92)); // y-
			obj.q_color.push_back(QColor(128,128,255,92)); // z-
			obj.q_color.push_back(QColor(255,128,128,92)); // x-
			obj.q_color.push_back(QColor(255,  0,  0,92)); // x+
			obj.q_color.push_back(QColor(  0,  0,255,92)); // z+
			obj.q_color.push_back(QColor(  0,255,  0,92)); // y+
			
			return obj;
		}

		static ConvexMesh Camera(const Geometry::ProjectionMatrix& P, Eigen::Vector4d image_rect_llur, double pixel_spacing, bool highlight=false, const QColor& color=QColor(255,0,0,32))
		{
			ConvexMesh obj;
			Geometry::SourceDetectorGeometry sdg(P,pixel_spacing);
			obj.p_color=QColor(0,0,0,0);
			obj.p_style=0;
			obj.p_pos.push_back(sdg.C);
			obj.p_pos.push_back(sdg.point_on_detector(image_rect_llur[2],image_rect_llur[1]));
			obj.p_pos.push_back(sdg.point_on_detector(image_rect_llur[2],image_rect_llur[3]));
			obj.p_pos.push_back(sdg.point_on_detector(image_rect_llur[0],image_rect_llur[3]));
			obj.p_pos.push_back(sdg.point_on_detector(image_rect_llur[0],image_rect_llur[1]));
			obj.p_pos.push_back(sdg.principal_point_3d);
			obj.q_index.push_back(Eigen::Vector4i(1,2,3,4));
			obj.q_color.clear();
			obj.q_color.push_back(color);
			obj.q_color_back=QColor(0,0,0,10);
			obj.q_style=-1;
			obj.l_color=color;
			obj.l_color.setAlpha(color.alpha()*4.0>255?255:color.alpha()*4.0);
			obj.l_style=2;
			if (highlight)
			{
				obj.l_index.push_back(Eigen::Vector2i(0,1));
				obj.l_index.push_back(Eigen::Vector2i(0,2));
				obj.l_index.push_back(Eigen::Vector2i(0,3));
				obj.l_index.push_back(Eigen::Vector2i(0,4));
			}
			else
			{
				obj.q_color.front()=QColor(0,0,0,25);
				obj.q_color_back   =QColor(0,0,0,10);
				obj.l_index.push_back(Eigen::Vector2i(0,5));
				obj.q_style=2;
			}
			return obj;
		}

	};

	inline Group FancyCamera(const Geometry::ProjectionMatrix& P, Eigen::Vector4d image_rect_llur, double pixel_spacing, const QColor& color=QColor(255,0,0,128))
	{
		Group group;
		auto& plain_cam=group.set("Frustum",ConvexMesh::Camera(P,image_rect_llur,pixel_spacing,true,color));

		// u- and v-axes
		Geometry::SourceDetectorGeometry sdg(P,pixel_spacing);
		auto pp=Geometry::getCameraPrincipalPoint(P);
		Geometry::RP3Point O=sdg.principal_point_3d;
		Geometry::RP3Point U=sdg.point_on_detector(Geometry::RP2Point(image_rect_llur[2],pp[1],1.0));
		Geometry::RP3Point V=sdg.point_on_detector(Geometry::RP2Point(pp[0],image_rect_llur[3],1.0));
		double arrow_head_size_mm=0.1*(Geometry::dehomogenized(O)-Geometry::dehomogenized(U)).norm();
		group.set("u",Arrow3D(O,U,QColor(255,0,255),2,arrow_head_size_mm,"u"));
		group.set("v",Arrow3D(O,V,QColor(0,255,255),2,arrow_head_size_mm,"v"));

		return group;
	}

} // namespace GraphicsItems
	
#endif // __graphics_item_convex_mesh_hxx
