% line L = plane P meet plane Q
function L=meet(P, Q)
	L=[
			P(3)*Q(4)-P(4)*Q(3);
			P(4)*Q(2)-P(2)*Q(4);
			P(2)*Q(3)-P(3)*Q(2);
			P(1)*Q(4)-P(4)*Q(1);
			P(3)*Q(1)-P(1)*Q(3);
			P(1)*Q(2)-P(2)*Q(1)
	];
end % function
