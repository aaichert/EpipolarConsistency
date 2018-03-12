function ecc_show_lines(lines, n_x, n_y, line_color, line_width)
	l_left=[1;0;-n_x*0.5];
	l_right=[1;0;n_x*0.5];
	l_bottom=[0;1;-n_y*0.5];
	l_top=[0;1;n_y*0.5];
	for i=1:size(lines,2)
		l=lines(:,i);
		if l(1)>l(2)
			x0=cross(l,l_top);
			x1=cross(l,l_bottom);
		else
			x0=cross(l,l_left);
			x1=cross(l,l_right);
		end % if
		x0=x0./x0(3);
		x1=x1./x1(3);
		line( [x0(1)+n_x*0.5, x1(1)+n_x*0.5 ], [x0(2)+n_y*0.5, x1(2)+n_y*0.5 ], 'color', line_color , 'linewidth', line_width  );
	end % for

end % function
