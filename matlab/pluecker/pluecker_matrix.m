% Anti-symmetric matrix for the meet operation dual Plücker coordinates
function Lx = pluecker_matrix(L)
Lx = [
				0 , - L(1), - L(2), - L(3);
				 + L(1),     0 , - L(4), - L(5);
				 + L(2), + L(4),     0 , - L(6);
				 + L(3), + L(5), + L(6),     0
				 ];
end % function