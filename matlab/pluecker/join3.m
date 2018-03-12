% Plane through three points or point of intersection of three planes
function X=join3(A,B,C)
	ABC=[A,B,C];
	X=[
		+det(ABC([2 3 4],:))
		-det(ABC([1 3 4],:))
		+det(ABC([1 2 4],:))
		-det(ABC([1 2 3],:))
	];
end % function