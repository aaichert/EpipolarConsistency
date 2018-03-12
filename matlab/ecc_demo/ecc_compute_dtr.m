function dtr = ecc_compute_dtr(I)
	% Original image size
	[n_x n_y]=size(I);
	% Some appropriate bin size for angles
	diagonal=norm([n_x n_y]);
	radon_binsize_alpha_deg=180/diagonal;
	% Radon transform
	rhoI=radon(I,[0:radon_binsize_alpha_deg:180]);
	% Gradient computation (in t-direction)
	[unused dtr]=gradient(rhoI);
	% Number of pixels in original image, to which one bin of radon transform 
	radon_binsize_t=1;
end % function
