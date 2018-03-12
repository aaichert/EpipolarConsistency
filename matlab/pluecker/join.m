% line L = point A join point B
function L=join(A, B)

	L=[
			A(1)*B(2)-A(2)*B(1);
			A(1)*B(3)-A(3)*B(1);
			A(1)*B(4)-A(4)*B(1);
			A(2)*B(3)-A(3)*B(2);
			A(2)*B(4)-A(4)*B(2);
			A(3)*B(4)-A(4)*B(3)
	];

end % function