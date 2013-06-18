% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2091.251503402219441 ; 2082.889080248142818 ];

%-- Principal point:
cc = [ 703.604457604929621 ; 440.371260782006630 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.040684215109201 ; -0.053226408868170 ; -0.008708327610243 ; 0.031180770431267 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 46.570063857593325 ; 42.402544640804756 ];

%-- Principal point uncertainty:
cc_error = [ 40.813489779319731 ; 37.379545872558815 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.080412136415635 ; 0.206820599908138 ; 0.005706894658185 ; 0.014816607482463 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 960;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 9;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -1.108072e+00 ; -2.796211e+00 ; 2.896802e-01 ];
Tc_1  = [ -1.688000e+03 ; -4.067591e+02 ; 7.215986e+03 ];
omc_error_1 = [ 7.951672e-03 ; 1.630565e-02 ; 2.784998e-02 ];
Tc_error_1  = [ 1.373898e+02 ; 1.308494e+02 ; 1.626117e+02 ];

%-- Image #2:
omc_2 = [ -1.166061e+00 ; -2.885323e+00 ; 3.483168e-01 ];
Tc_2  = [ -1.808526e+03 ; -4.216280e+02 ; 7.147377e+03 ];
omc_error_2 = [ 8.625850e-03 ; 1.615031e-02 ; 2.964140e-02 ];
Tc_error_2  = [ 1.363987e+02 ; 1.293582e+02 ; 1.644800e+02 ];

%-- Image #3:
omc_3 = [ 1.150992e+00 ; 2.855687e+00 ; -4.923578e-01 ];
Tc_3  = [ -1.754172e+03 ; -4.056935e+02 ; 7.167280e+03 ];
omc_error_3 = [ 3.824019e-03 ; 1.460121e-02 ; 2.944079e-02 ];
Tc_error_3  = [ 1.372037e+02 ; 1.294488e+02 ; 1.669063e+02 ];

%-- Image #4:
omc_4 = [ 1.049681e+00 ; 2.777103e+00 ; -4.668203e-01 ];
Tc_4  = [ -1.692306e+03 ; -3.237180e+02 ; 7.126389e+03 ];
omc_error_4 = [ 4.168200e-03 ; 1.506895e-02 ; 2.929692e-02 ];
Tc_error_4  = [ 1.369935e+02 ; 1.286820e+02 ; 1.665004e+02 ];

%-- Image #5:
omc_5 = [ 9.470624e-01 ; 2.743121e+00 ; -6.371014e-01 ];
Tc_5  = [ -1.737336e+03 ; -2.824564e+02 ; 7.187654e+03 ];
omc_error_5 = [ 3.942235e-03 ; 1.547145e-02 ; 2.704213e-02 ];
Tc_error_5  = [ 1.385948e+02 ; 1.298956e+02 ; 1.665485e+02 ];

%-- Image #6:
omc_6 = [ 9.253421e-01 ; 2.539755e+00 ; -8.794066e-01 ];
Tc_6  = [ -1.740481e+03 ; -2.045411e+02 ; 7.226115e+03 ];
omc_error_6 = [ 6.071218e-03 ; 1.611944e-02 ; 2.558331e-02 ];
Tc_error_6  = [ 1.397995e+02 ; 1.308330e+02 ; 1.610913e+02 ];

%-- Image #7:
omc_7 = [ 9.090228e-01 ; 2.505586e+00 ; -9.437329e-01 ];
Tc_7  = [ -1.677004e+03 ; -1.864720e+02 ; 7.240352e+03 ];
omc_error_7 = [ 6.683496e-03 ; 1.620515e-02 ; 2.547534e-02 ];
Tc_error_7  = [ 1.401329e+02 ; 1.310809e+02 ; 1.601945e+02 ];

%-- Image #8:
omc_8 = [ 8.961132e-01 ; 2.423314e+00 ; -1.039779e+00 ];
Tc_8  = [ -1.372221e+03 ; -1.579821e+02 ; 7.149541e+03 ];
omc_error_8 = [ 7.910528e-03 ; 1.639325e-02 ; 2.530790e-02 ];
Tc_error_8  = [ 1.385121e+02 ; 1.292521e+02 ; 1.551853e+02 ];

%-- Image #9:
omc_9 = [ 8.562507e-01 ; 2.202637e+00 ; -9.595627e-01 ];
Tc_9  = [ -1.009894e+03 ; -1.461467e+02 ; 7.075484e+03 ];
omc_error_9 = [ 9.606890e-03 ; 1.699542e-02 ; 2.385389e-02 ];
Tc_error_9  = [ 1.373495e+02 ; 1.276793e+02 ; 1.474935e+02 ];

