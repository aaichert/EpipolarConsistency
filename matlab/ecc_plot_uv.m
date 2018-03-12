% range_px   Number of pixles to shift u and v (positive and negatvie)
% num_steps  Number of step sin each direction (output will be (num_steps+1)x(num_steps+1) )
% P0, P1     Projection matrices
% n_x, n_y   Original image size in pixels
% dtr0, dtr1 Derivatives of Radon transforms of original images
% range_t    Range (pixels) of Radon transform. Usually equal to original image diagonal
% dkappa     Angular step between epipolar planes. Should be small enough to densely sample RDAs
function consistency_uv = ecc_plot_uv(range_px, num_steps, P0, P1, n_x, n_y, dtr0, dtr1, range_t, dkappa)

	consistency_uv=zeros(2*num_steps+1);
	
	 h = waitbar (0, 'Computing plot (slow!) 0.00%');
	for v_step=-num_steps:num_steps
		for u_step=-num_steps:num_steps
			progress=((v_step+num_steps)*(2*num_steps+1)+u_step+num_steps)/((2*num_steps+1)*(2*num_steps+1));
			waitbar(progress,h,sprintf('Computing plot (slow!) %.2f%%', 100*progress))
			% Build transformation matrix of image 0
			H=diag([1 1 1]);
			H(1:2,3)=[ u_step; v_step ].*range_px;
			% Compute ECC for transformation H of image 0
			consistency_uv(u_step+num_steps+1,v_step+num_steps+1)=ecc_compute_consistency(H*P0, P1, n_x, n_y, dtr0, dtr1, range_t, dkappa);
		end % for
	end % for
	
	% Close waitbar
	close(h);
	
end % function
