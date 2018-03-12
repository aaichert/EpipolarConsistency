
% Edit this script to select two images
% Sets P0, P1 (projection images), I0, I1 (images), dtr0, dtr1 (derivative of Radon transforms)
ecc_load_data

%
% Visualization
%
%(images normalized and scaled by 20%)
s=0.20;
H=diag([s,s,1]);
pkg load image
I0=imresize(double(I0./max(I0(:))),s)*256;
I1=imresize(double(I1./max(I1(:))),s)*256;
% Projection matrices scaled accordingly
HP0=H*P0;
HP1=H*P1;
% Epipoles
e0=HP0*null(HP1);
e1=HP1*null(HP0);
% Two arbitrary world points
A=[ +10 +10 +10 1]';
B=[ -10 -10 -10 1]';
% Two sets of corresponding image points
a0=HP0*A;
a1=HP1*A;
b0=HP0*B;
b1=HP1*B;
% De-homogenization
e0=e0./e0(3);
e1=e1./e1(3);
a0=a0./a0(3);
a1=a1./a1(3);
b0=b0./b0(3);
b1=b1./b1(3);
% Drawing parts of two sets of epipolar lines
figure('Name','Image 0','NumberTitle','off');
hold on
axis equal
colormap(gray(256))
image(I0);
line_stroke(e0(1:2), a0(1:2), 'g', 2);
line_stroke(e0(1:2), b0(1:2), 'g', 2);
text(e0(1), e0(2), 'e0');
text(a0(1), a0(2), '(+10 +10 +10)');
text(b0(1), b0(2), '(-10 -10 -10)');
figure('Name','Image 1','NumberTitle','off');
hold on
axis equal
colormap(gray(256))
image(I1);
line_stroke(e1(1:2), a1(1:2), 'r', 2);
line_stroke(e1(1:2), b1(1:2), 'r', 2);
text(e1(1), e1(2), 'e1');
text(a1(1), a1(2), '(+10 +10 +10)');
text(b1(1), b1(2), '(-10 -10 -10)');

%
% ECC Geometry
%

% Mapping to epipolar lines
[K0 K1]=ecc_computeK01(P0,P1, n_x, n_y);

%
% Sampling Radon Derivatives
%

% For visualization, we remember intermediate values
kappas=[];
l0s=[]; x0s=[]; y0s=[]; v0s=[]; w0s=[];
l1s=[]; x1s=[]; y1s=[]; v1s=[]; w1s=[];

% Sum of inconsistencies = Consistency Metric
consistency=0;
% define spacing between epipolar lines: 0.05 degrees
disp('Iterating over epipolar planes. This may take a minute...')
dkappa=0.2*pi/180
for kappa=[dkappa:dkappa:pi*0.5]
	% We start with the reference point and go both ways
	kappas=[kappas kappa -kappa];
	
	% Compute epipolar line
	x_kappa=[cos(kappa); sin(kappa)];
	l0_plus_kappa=K0*x_kappa;
	l1_plus_kappa=K1*x_kappa;
	% Sample Radon derivatives at corresponding locations
	[v0 x0, y0, w0]=ecc_sample_dtr_debug(l0_plus_kappa, dtr0, range_t);
	[v1 x1, y1, w1]=ecc_sample_dtr_debug(l1_plus_kappa, dtr1, range_t);
	% Compute SSD
	v_plus=(v0-v1);
	% Remember values for visualization
	x0s=[x0s,x0]; y0s=[y0s,y0]; % Pixel location in dtr0
	v0s=[v0s,v0]; w0s=[w0s,w0]; % Corresponding value and weight
	x1s=[x1s,x1]; y1s=[y1s,y1]; % Pixel location in dtr1
	v1s=[v1s,v1]; w1s=[w1s,w1]; % Corresponding value and weight
	
	x_kappa=[cos(-kappa); sin(-kappa)];
	l0_minus_kappa=K0*x_kappa;
	l1_minus_kappa=K1*x_kappa;
	% Sample Radon derivatives at corresponding locations
	[v0 x0, y0, w0]=ecc_sample_dtr_debug(l0_minus_kappa, dtr0, range_t);
	[v1 x1, y1, w1]=ecc_sample_dtr_debug(l1_minus_kappa, dtr1, range_t);
	% Compute SSD
	v_minus=(v0-v1);
	% For visualization
	l0s=[l0s, l0_plus_kappa, l0_minus_kappa];
	x0s=[x0s,x0];
	y0s=[y0s,y0];
	v0s=[v0s,v0];
	w0s=[w0s,w0];
	l1s=[l1s, l1_plus_kappa, l1_minus_kappa];
	x1s=[x1s,x1];
	y1s=[y1s,y1];
	v1s=[v1s,v1];
	w1s=[w1s,w1];
	
	% Stopping criteria (important if epipolar not within images
	v=v_plus*v_plus+v_minus*v_minus;
	if v==0
		break;
	end % if
	consistency=consistency+v*dkappa;
end % for

%
% Visualization of dtr samples
%

% Show Figure
figure('name','Derivative of Radon Transform 0','numbertitle','off');
window=max(dtr0(:))/256;
hold on
colormap(gray(256));
image(dtr0./window+128);
plot(x0s,y0s,'r.');
figure('name','Derivative of Radon Transform 1','numbertitle','off');
hold on
colormap(gray(256));
image(dtr1./window+128);
plot(x1s,y1s,'g.');

%
% Visualization of redundant samples by angle
%

[s idx]=sort(kappas);
figure('name','Redundant Samples','numbertitle','off')
plot(s/pi*180,v0s(idx),'r');
hold on
plot(s/pi*180,v1s(idx),'g');
hold off
l=legend('Projection 0', 'Projection 1');
legend(l,'boxon');
xlabel('Angle to reference plane [°]');
ylabel('Derivtive of line integral [a.u.]');

%
% Visualization of epipolar lines
%

visualize_all_lines=false
if visualize_all_lines
	figure('name','Epipolar lines of Image 0 (red) and 1 (green)','numbertitle','off')
	axis([0,n_x,0,n_y]);
	axis equal
	% Show reference lines
	ecc_show_lines(l0s,n_x,n_y,'red', 2)
	ecc_show_lines(l1s,n_x,n_y,'green', 2)
end % if

% Result:
consistency % should be close to zero

% Optional: plot consistency for detector shifts u/v by 50 pixels
plot_uv = false;

if (plot_uv)
	% Note: this is very slow in Octave because of a lack of JIT compilation.
	% We therefore choose a very low dkappa!
	dkappa=dkappa*20;
	consistency_uv = ecc_plot_uv(50, 5, P0, P1, n_x, n_y, dtr0, dtr1, range_t, dkappa);
	surf(consistency_uv);
end % if
