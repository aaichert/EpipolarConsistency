% plane P = line L join point X
function P=pluecker_join(L, X)

		P=[
				             + X(2)*L(6) - X(3)*L(5) + X(4)*L(4);
				 - X(1)*L(6)             + X(3)*L(3) - X(4)*L(2);
				 + X(1)*L(5) - X(2)*L(3)             + X(4)*L(1);
				 - X(1)*L(4) + X(2)*L(2) - X(3)*L(1)
			];

end % function