% P0, P1     Projection matrices
% n_x, n_y   Original image size in pixels
% dtr0, dtr1 Derivatives of Radon transforms of original images
% range_t    Range (pixels) of Radon transform. Usually equal to original image diagonal
% dkappa     Angular step between epipolar planes. Should be small enough to densely sample RDAs
function consistency=ecc_compute_consistency(P0, P1, n_x, n_y, dtr0, dtr1, range_t, dkappa)
	
	% Mapping to epipolar lines
	[K0 K1]=ecc_computeK01(P0,P1, n_x, n_y);

	% Sum of inconsistencies = Consistency Metric
	consistency=0;
	for kappa=[0:dkappa:pi*0.5]
		% Compute epipolar line for +kappa
		x_kappa=[cos(kappa); sin(kappa)];
		l0_plus_kappa=K0*x_kappa;
		l1_plus_kappa=K1*x_kappa;
		% Sample Radon derivatives at corresponding locations
		vp0=ecc_sample_dtr(l0_plus_kappa, dtr0, range_t);
		vp1=ecc_sample_dtr(l1_plus_kappa, dtr1, range_t);
		
		% Compute epipolar line for -kappa
		x_kappa=[cos(-kappa); sin(-kappa)];
		l0_minus_kappa=K0*x_kappa;
		l1_minus_kappa=K1*x_kappa;
		% Sample Radon derivatives at corresponding locations
		vm0=ecc_sample_dtr(l0_minus_kappa, dtr0, range_t);
		vm1=ecc_sample_dtr(l1_minus_kappa, dtr1, range_t);

		% Compute squared difference (for both +/- kappa)
		v=(vp0-vp1)*(vp0-vp1)+(vm0-vm1)*(vm0-vm1);

		% Stopping criteria (important if epipole not within images
		if v==0
			break;
		end % if
		
		% Compute sum
		consistency=consistency+v*dkappa;
	end % for
end % function
