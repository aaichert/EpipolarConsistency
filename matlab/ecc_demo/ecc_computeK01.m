% [K0 K1]=ecc_computeK(P0, P1, n_x, n_y)
% Compute a mapping from an angle to a pair of epipolar lines.
% P0, P1 projection matrices;
% n_x, n_y projection image size in pixels.
% Usage: l0_kappa=K0*[cos(k); sin(k)];
%        l1_kappa=K1*[cos(k); sin(k)];
function [K0 K1]=ecc_computeK01(P0, P1, n_x, n_y);
	% Source positions
	C0=null(P0); C1=null(P1);
	% Plücker Coordinates of the baseline
	B01=C0(1)*C1(2)-C0(2)*C1(1);
	B02=C0(1)*C1(3)-C0(3)*C1(1);
	B03=C0(1)*C1(4)-C0(4)*C1(1);
	B12=C0(2)*C1(3)-C0(3)*C1(2);
	B13=C0(2)*C1(4)-C0(4)*C1(2);
	B23=C0(3)*C1(4)-C0(4)*C1(3);

	a2=[B12;-B02;B01];   % Plücker line moment
	a3=[-B03;-B13;-B23]; % Plücker direction of line

	% Normalization
	s2=norm(a2); s3=norm(a3);
	
	% Mapping from [cos(kappa) sin(kappa)]' to epipolar plane
	K = [ [ a2/s2 ; 0] , [ cross(a3,a2)/(s3*s2) ; -s2/s3 ] ];
	
	% Mappings to epipolar lines
	K0=pinv(P0)'*K; K1=pinv(P1)'*K;
	% Move origin from corner to center of image
	HinvT=[       1,       0, 	0;
			      0,       1, 	0;
			n_x*0.5, n_y*0.5, 	1];
	K0=HinvT*K0; K1=HinvT*K1;
	% Normalize with line though image of origin (unit normal)
	K0/=norm(K0(1:2,1)); K1/=norm(K1(1:2,1));
end % function
