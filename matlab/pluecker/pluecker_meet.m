% point X = line L meet plane P
function X=pluecker_meet(L, P)

		X=[
				             - P(2)*L(1) - P(3)*L(2) - P(4)*L(3),
				 + P(1)*L(1)             - P(3)*L(4) - P(4)*L(5),
				 + P(1)*L(2) + P(2)*L(4)             - P(4)*L(6),
				 + P(1)*L(3) + P(2)*L(5) + P(3)*L(6)
			];

end % function