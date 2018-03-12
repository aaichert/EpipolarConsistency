% Compute the closest point of a line to the origin (Lx=pluecker_matrix(L); O=Lx*Lx*[0;0;0;1];)
function O = pluecker_closest_point_to_origin(L)
	O=[ L(5)*L(1)+L(2)*L(6); -L(1)*L(3)+L(4)*L(6); -L(2)*L(3)-L(4)*L(5); -L(3)*L(3)-L(5)*L(5)-L(6)*L(6) ];
end % function
