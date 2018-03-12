% Normalize projection matrix with such that principal vector P(3,1:3) has unit length and points towards image
function P=camera_normalize(P)
	% Get length  of principal ray
	m3n=norm(P(3,1:3));
	% Enforce positivity of determinant
	if (det(P(:,1:3))<0)
		m3n=-m3n;
	end % if
	P=P/m3n;
end % function
