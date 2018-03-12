% Anti-symmetric matrix for the join operation using dual Plücker coordinates
function Lx = pluecker_matrix_dual(L)
Lx = [
			0 , + L(6), - L(5), + L(4);
			- L(6),     0 , + L(3), - L(2);
			+ L(5), - L(3),     0 , + L(1);
			- L(4), + L(2), - L(1),     0
				 ];
end % function