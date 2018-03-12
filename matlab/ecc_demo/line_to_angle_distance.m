function [alpha t] = line_to_angle_distance(l)
	% Length or normal vector
	length=norm(l(1:2));
	% Angle between line normal and x-axis
	alpha=atan2(l(2),l(1));
	% Distance to the origin (scaled to RDA bins)
	t=l(3)/length;
end % function
