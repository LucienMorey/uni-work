% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 692.165816836222461 ; 690.098110008669096 ];

%-- Principal point:
cc = [ 284.671878851918677 ; 244.394134575495201 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.089289148555831 ; -0.204643851545819 ; -0.000918178758122 ; -0.001115726144705 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.778495367744966 ; 1.759236272747943 ];

%-- Principal point uncertainty:
cc_error = [ 2.954710471007772 ; 2.356310161780081 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.010236348607534 ; 0.033294902104486 ; 0.001289519837152 ; 0.001736120496400 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 10;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.077258e+00 ; 2.167844e+00 ; -1.546196e-01 ];
Tc_1  = [ -9.964467e+01 ; -1.189412e+02 ; 3.944219e+02 ];
omc_error_1 = [ 2.866570e-03 ; 3.211044e-03 ; 6.321197e-03 ];
Tc_error_1  = [ 1.707257e+00 ; 1.344495e+00 ; 1.257836e+00 ];

%-- Image #2:
omc_2 = [ 2.175385e+00 ; 2.247156e+00 ; 1.746217e-01 ];
Tc_2  = [ -1.170391e+02 ; -8.665948e+01 ; 4.317333e+02 ];
omc_error_2 = [ 3.677000e-03 ; 3.242100e-03 ; 7.539572e-03 ];
Tc_error_2  = [ 1.870407e+00 ; 1.502750e+00 ; 1.458965e+00 ];

%-- Image #3:
omc_3 = [ -1.955378e+00 ; -2.135523e+00 ; 4.125513e-01 ];
Tc_3  = [ -8.855449e+01 ; -1.201538e+02 ; 4.447711e+02 ];
omc_error_3 = [ 3.176764e-03 ; 3.082864e-03 ; 5.930348e-03 ];
Tc_error_3  = [ 1.906545e+00 ; 1.506205e+00 ; 1.282627e+00 ];

%-- Image #4:
omc_4 = [ 1.853823e+00 ; 1.801544e+00 ; -6.204133e-01 ];
Tc_4  = [ -1.230220e+02 ; -6.696992e+01 ; 4.930478e+02 ];
omc_error_4 = [ 2.560676e-03 ; 3.490346e-03 ; 5.359394e-03 ];
Tc_error_4  = [ 2.098432e+00 ; 1.680708e+00 ; 1.338760e+00 ];

%-- Image #5:
omc_5 = [ 1.981239e+00 ; 1.701744e+00 ; 2.138442e-01 ];
Tc_5  = [ -8.685589e+01 ; -1.073185e+02 ; 4.174579e+02 ];
omc_error_5 = [ 3.399764e-03 ; 3.025263e-03 ; 5.465098e-03 ];
Tc_error_5  = [ 1.811375e+00 ; 1.417925e+00 ; 1.365848e+00 ];

%-- Image #6:
omc_6 = [ -1.841622e+00 ; -1.904075e+00 ; -4.663544e-01 ];
Tc_6  = [ -9.922163e+01 ; -1.124746e+02 ; 3.512648e+02 ];
omc_error_6 = [ 2.293574e-03 ; 3.503290e-03 ; 5.550077e-03 ];
Tc_error_6  = [ 1.540753e+00 ; 1.222677e+00 ; 1.247524e+00 ];

%-- Image #7:
omc_7 = [ 1.630621e+00 ; 1.511816e+00 ; -1.517095e-02 ];
Tc_7  = [ -1.389192e+02 ; -9.436333e+01 ; 4.677728e+02 ];
omc_error_7 = [ 2.993107e-03 ; 3.501056e-03 ; 4.535340e-03 ];
Tc_error_7  = [ 2.016695e+00 ; 1.610074e+00 ; 1.516876e+00 ];

%-- Image #8:
omc_8 = [ 1.777171e+00 ; 2.188509e+00 ; -1.197258e+00 ];
Tc_8  = [ -8.935229e+01 ; -1.144863e+02 ; 5.489452e+02 ];
omc_error_8 = [ 2.079813e-03 ; 4.197209e-03 ; 6.045610e-03 ];
Tc_error_8  = [ 2.361180e+00 ; 1.882250e+00 ; 1.299957e+00 ];

%-- Image #9:
omc_9 = [ -1.337564e+00 ; -1.817182e+00 ; 6.357403e-01 ];
Tc_9  = [ 1.399647e-01 ; -1.678989e+02 ; 5.110795e+02 ];
omc_error_9 = [ 3.169302e-03 ; 3.427580e-03 ; 4.428224e-03 ];
Tc_error_9  = [ 2.216383e+00 ; 1.735028e+00 ; 1.322493e+00 ];

%-- Image #10:
omc_10 = [ 1.850442e+00 ; 1.368448e+00 ; 6.690423e-01 ];
Tc_10  = [ -1.118131e+02 ; -8.204562e+01 ; 4.559631e+02 ];
omc_error_10 = [ 3.733795e-03 ; 2.987237e-03 ; 4.912197e-03 ];
Tc_error_10  = [ 1.990916e+00 ; 1.567818e+00 ; 1.631739e+00 ];

