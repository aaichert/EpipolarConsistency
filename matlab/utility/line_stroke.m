function line_stroke(from, to, color, width)

	if nargin < 2
		line( [from(1) to(1) ], [from(2) to(2) ] );
	else if nargin < 3
		line( [from(1) to(1) ], [from(2) to(2) ], 'color', color );
	else
		line( [from(1) to(1) ], [from(2) to(2) ], 'color', color , 'linewidth', width );
	end % if
	
end % function
