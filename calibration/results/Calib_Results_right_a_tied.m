% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2093.003663620762836 ; 2096.128212454517325 ];

%-- Principal point:
cc = [ 646.834350604957649 ; 442.473378973852846 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.124050916716394 ; -1.146897650540894 ; -0.001186453446960 ; -0.001876831802890 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 53.485207952901419 ; 52.289850554616656 ];

%-- Principal point uncertainty:
cc_error = [ 22.944148092672766 ; 28.400010932881116 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.058305677018989 ; 0.918193256530127 ; 0.003104519828594 ; 0.002011287038310 ; 0.000000000000000 ];

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
omc_1 = [ 1.089835e+00 ; 2.919271e+00 ; -3.760647e-01 ];
Tc_1  = [ 5.351189e+02 ; -5.856318e+02 ; 6.447173e+03 ];
omc_error_1 = [ 3.322859e-03 ; 1.092003e-02 ; 1.943746e-02 ];
Tc_error_1  = [ 7.073067e+01 ; 8.705591e+01 ; 1.647934e+02 ];

%-- Image #2:
omc_2 = [ 9.219074e-01 ; 2.713959e+00 ; -9.217001e-01 ];
Tc_2  = [ 3.357143e+02 ; -2.187766e+02 ; 8.133034e+03 ];
omc_error_2 = [ 3.732374e-03 ; 1.114933e-02 ; 1.929548e-02 ];
Tc_error_2  = [ 8.903470e+01 ; 1.100565e+02 ; 2.009006e+02 ];

%-- Image #3:
omc_3 = [ 9.926808e-01 ; 2.788659e+00 ; -7.614694e-01 ];
Tc_3  = [ 1.749768e+02 ; -2.323216e+02 ; 7.811968e+03 ];
omc_error_3 = [ 2.773924e-03 ; 1.084236e-02 ; 1.982495e-02 ];
Tc_error_3  = [ 8.551484e+01 ; 1.057030e+02 ; 1.951108e+02 ];

%-- Image #4:
omc_4 = [ 1.161717e+00 ; 2.858604e+00 ; -2.586034e-01 ];
Tc_4  = [ -1.150583e+02 ; -2.276576e+02 ; 7.448174e+03 ];
omc_error_4 = [ 4.142178e-03 ; 1.126455e-02 ; 1.954193e-02 ];
Tc_error_4  = [ 8.148051e+01 ; 1.007772e+02 ; 1.917897e+02 ];

%-- Image #5:
omc_5 = [ 1.038484e+00 ; 2.864650e+00 ; -1.562618e-01 ];
Tc_5  = [ -4.251667e+01 ; -2.159484e+02 ; 7.362089e+03 ];
omc_error_5 = [ 4.032804e-03 ; 1.068600e-02 ; 1.889865e-02 ];
Tc_error_5  = [ 8.057622e+01 ; 9.962112e+01 ; 1.890054e+02 ];

%-- Image #6:
omc_6 = [ 7.035946e-01 ; 2.995733e+00 ; -1.136551e-01 ];
Tc_6  = [ 3.367413e+02 ; -2.034420e+02 ; 7.303663e+03 ];
omc_error_6 = [ 3.014647e-03 ; 1.087735e-02 ; 1.898016e-02 ];
Tc_error_6  = [ 7.991652e+01 ; 9.890016e+01 ; 1.862761e+02 ];

%-- Image #7:
omc_7 = [ 1.751774e-01 ; 3.129203e+00 ; -1.098336e-01 ];
Tc_7  = [ 8.679788e+02 ; -1.909857e+01 ; 7.255251e+03 ];
omc_error_7 = [ 1.659274e-03 ; 1.241994e-02 ; 2.101266e-02 ];
Tc_error_7  = [ 7.946332e+01 ; 9.851316e+01 ; 1.838699e+02 ];

%-- Image #8:
omc_8 = [ -4.331759e-02 ; -3.104079e+00 ; 6.843793e-02 ];
Tc_8  = [ 9.816483e+02 ; 5.270344e+01 ; 7.178584e+03 ];
omc_error_8 = [ 1.158571e-03 ; 1.451011e-02 ; 2.226983e-02 ];
Tc_error_8  = [ 7.871637e+01 ; 9.763289e+01 ; 1.815023e+02 ];

%-- Image #9:
omc_9 = [ -4.323069e-02 ; -3.103245e+00 ; 6.294909e-02 ];
Tc_9  = [ 9.815614e+02 ; 5.233757e+01 ; 7.171650e+03 ];
omc_error_9 = [ 1.144989e-03 ; 1.448639e-02 ; 2.215398e-02 ];
Tc_error_9  = [ 7.864421e+01 ; 9.754648e+01 ; 1.813459e+02 ];

