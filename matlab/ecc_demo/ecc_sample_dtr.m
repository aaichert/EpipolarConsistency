function [deriv_line_integral]=ecc_sample_dtr(line, dtr, range_t)
	% Convert line to angle-distance
	[alpha, t]=line_to_angle_distance(line);
	
	% Scale to Radon bins
	dtr_x=alpha/pi+1; % 0 to 2 range
	dtr_y=t/range_t+0.5; % 0 to 1 with image center at .5

	% Compute normalized coordinates in Radon derivative
	% Also accounts for symmetry rho(alpha,t)=-rho(alpha+pi,-t)
	if (dtr_x>1)
		weight=-1;
		dtr_x=dtr_x-1.0;
		dtr_y=1.0-dtr_y;
	else
		weight=1;
	end % if
	
	% Scale to actual pixels (one-based and flip to be consistent with MATLAB's radon() function)
	dtr_x=(1-dtr_x)*size(dtr,1)+1;
	dtr_y=(1-dtr_y)*size(dtr,2)+1;
	% Sample image (linear interpolation)
	deriv_line_integral=weight*interp2(dtr,dtr_x,dtr_y,'linear',0);
	
end % function
