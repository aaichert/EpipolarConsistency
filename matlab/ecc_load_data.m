%
% Let Octave know the location of some additional functions
%
addpath ("./utility")
addpath ("./ecc_demo")

%
% Loaing data
%
file_dtr0='./ecc_demo_data/pumpkin/dtr/dtr_V01.nrrd';
file_dtr1='./ecc_demo_data/pumpkin/dtr/dtr_V13.nrrd';
file_projection0='./ecc_demo_data/pumpkin/V01.nrrd';
file_projection1='./ecc_demo_data/pumpkin/V13.nrrd';

% Load NRRD files
I0 = nrrdread(file_projection0);
I1 = nrrdread(file_projection1);
[dtr0, meta0] = nrrdread(file_dtr0);
[dtr1, meta1] = nrrdread(file_dtr1);

% Size of projection image
n_x=str2double(meta0.originalimagewidth);
n_y=str2double(meta0.originalimageheight);
% Number of pixels in original image, to which one bin of radon transform corresponds.
radon_binsize_t= str2double(meta0.stepintercept);
% One over the range of the t-axis of the radon transform in pixels 
range_t=size(dtr0,1)*radon_binsize_t;
% Projection matrices
eval(meta0.projectionmatrix);
P0=ans;
eval(meta1.projectionmatrix);
P1=ans;
