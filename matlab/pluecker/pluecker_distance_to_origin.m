% Distance of the line L to the origin
function d=pluecker_distance_to_origin(L)
	d=norm(pluecker_moment(L))/norm(pluecker_direction(L))
end % function
