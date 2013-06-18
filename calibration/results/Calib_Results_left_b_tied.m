% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2249.380368590224407 ; 2250.851755171743662 ];

%-- Principal point:
cc = [ 662.767049513547704 ; 449.625813801248910 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.189668411965876 ; 0.169692985526995 ; -0.002599755829990 ; -0.000077982209805 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 3.633927829720296 ; 3.525730513440871 ];

%-- Principal point uncertainty:
cc_error = [ 5.381291060001431 ; 7.338034001983632 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.008457664757102 ; 0.082521096359926 ; 0.000551724998210 ; 0.000448498346628 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 960;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 23;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.133987e-01 ; 2.246520e+00 ; -5.245673e-02 ];
Tc_1  = [ -3.374648e+02 ; -6.148565e+02 ; 3.363506e+03 ];
omc_error_1 = [ 1.529856e-03 ; 2.362668e-03 ; 3.694777e-03 ];
Tc_error_1  = [ 8.138278e+00 ; 1.099412e+01 ; 5.668113e+00 ];

%-- Image #2:
omc_2 = [ 3.809799e-03 ; 2.275702e+00 ; -6.595521e-02 ];
Tc_2  = [ -2.342738e+02 ; -5.498776e+02 ; 3.272338e+03 ];
omc_error_2 = [ 1.513540e-03 ; 2.373154e-03 ; 3.742821e-03 ];
Tc_error_2  = [ 7.897615e+00 ; 1.067680e+01 ; 5.388622e+00 ];

%-- Image #3:
omc_3 = [ 1.464195e-01 ; 2.744623e+00 ; -9.490057e-02 ];
Tc_3  = [ 5.706725e+01 ; -5.159570e+02 ; 3.595792e+03 ];
omc_error_3 = [ 8.362404e-04 ; 2.576853e-03 ; 4.562055e-03 ];
Tc_error_3  = [ 8.594949e+00 ; 1.170777e+01 ; 6.019426e+00 ];

%-- Image #4:
omc_4 = [ 1.828331e-01 ; 3.054435e+00 ; -1.485110e-01 ];
Tc_4  = [ 3.893768e+02 ; -4.577095e+02 ; 3.988200e+03 ];
omc_error_4 = [ 6.151231e-04 ; 3.875202e-03 ; 6.234252e-03 ];
Tc_error_4  = [ 9.520070e+00 ; 1.302361e+01 ; 6.877672e+00 ];

%-- Image #5:
omc_5 = [ -2.250495e-01 ; -3.064917e+00 ; 6.985020e-02 ];
Tc_5  = [ 5.493676e+02 ; -4.246306e+02 ; 4.024439e+03 ];
omc_error_5 = [ 5.085680e-04 ; 5.314848e-03 ; 6.909973e-03 ];
Tc_error_5  = [ 9.645923e+00 ; 1.318777e+01 ; 7.279121e+00 ];

%-- Image #6:
omc_6 = [ -2.275394e-01 ; -2.977112e+00 ; 8.045602e-02 ];
Tc_6  = [ 7.167440e+02 ; -3.974629e+02 ; 4.097884e+03 ];
omc_error_6 = [ 5.533039e-04 ; 5.277229e-03 ; 6.600372e-03 ];
Tc_error_6  = [ 9.842083e+00 ; 1.347688e+01 ; 7.494354e+00 ];

%-- Image #7:
omc_7 = [ -1.832032e-01 ; -2.912117e+00 ; 9.814432e-02 ];
Tc_7  = [ 8.511843e+02 ; -4.067643e+02 ; 4.133122e+03 ];
omc_error_7 = [ 5.931691e-04 ; 4.817134e-03 ; 6.106214e-03 ];
Tc_error_7  = [ 9.946679e+00 ; 1.363936e+01 ; 7.579138e+00 ];

%-- Image #8:
omc_8 = [ 1.890204e-01 ; 2.536210e+00 ; -1.045867e+00 ];
Tc_8  = [ 1.636217e+02 ; -2.367507e+02 ; 4.378405e+03 ];
omc_error_8 = [ 1.470765e-03 ; 2.896813e-03 ; 4.166500e-03 ];
Tc_error_8  = [ 1.045665e+01 ; 1.427849e+01 ; 6.544454e+00 ];

%-- Image #9:
omc_9 = [ 9.006591e-02 ; 2.854868e+00 ; -2.463689e-01 ];
Tc_9  = [ 9.856403e+01 ; -5.281615e+02 ; 3.655146e+03 ];
omc_error_9 = [ 6.786344e-04 ; 2.701267e-03 ; 4.929322e-03 ];
Tc_error_9  = [ 8.742375e+00 ; 1.189059e+01 ; 6.117052e+00 ];

%-- Image #10:
omc_10 = [ 3.206949e-02 ; 2.491530e+00 ; -1.140286e+00 ];
Tc_10  = [ -5.442072e+00 ; -3.931865e+02 ; 3.974606e+03 ];
omc_error_10 = [ 1.620195e-03 ; 2.811727e-03 ; 4.203145e-03 ];
Tc_error_10  = [ 9.521937e+00 ; 1.295930e+01 ; 5.919386e+00 ];

%-- Image #11:
omc_11 = [ 4.148159e-02 ; 2.386591e+00 ; -1.222681e+00 ];
Tc_11  = [ -2.895544e+01 ; -3.532297e+02 ; 4.036948e+03 ];
omc_error_11 = [ 1.765769e-03 ; 2.877209e-03 ; 4.029906e-03 ];
Tc_error_11  = [ 9.669714e+00 ; 1.316929e+01 ; 5.909489e+00 ];

%-- Image #12:
omc_12 = [ -1.822472e-01 ; -3.018585e+00 ; 2.650873e-01 ];
Tc_12  = [ 9.226535e+02 ; -3.529794e+02 ; 3.923423e+03 ];
omc_error_12 = [ 7.947555e-04 ; 4.358020e-03 ; 7.190658e-03 ];
Tc_error_12  = [ 9.418025e+00 ; 1.300207e+01 ; 7.086228e+00 ];

%-- Image #13:
omc_13 = [ -1.065995e-01 ; 2.821701e+00 ; -9.220440e-01 ];
Tc_13  = [ 1.008805e+03 ; -3.131874e+02 ; 4.307447e+03 ];
omc_error_13 = [ 1.472482e-03 ; 2.953337e-03 ; 4.766920e-03 ];
Tc_error_13  = [ 1.033435e+01 ; 1.430272e+01 ; 7.276283e+00 ];

%-- Image #14:
omc_14 = [ 1.871024e-01 ; 2.758397e+00 ; -3.227361e-01 ];
Tc_14  = [ 4.094535e+02 ; -4.075895e+02 ; 3.958919e+03 ];
omc_error_14 = [ 1.049427e-03 ; 2.655446e-03 ; 4.735937e-03 ];
Tc_error_14  = [ 9.435894e+00 ; 1.294698e+01 ; 6.451980e+00 ];

%-- Image #15:
omc_15 = [ 2.817155e-01 ; 2.720353e+00 ; -1.851608e-01 ];
Tc_15  = [ 3.334846e+02 ; -3.832306e+02 ; 3.823840e+03 ];
omc_error_15 = [ 1.102878e-03 ; 2.641087e-03 ; 4.719673e-03 ];
Tc_error_15  = [ 9.112097e+00 ; 1.249426e+01 ; 6.272950e+00 ];

%-- Image #16:
omc_16 = [ -2.380197e-02 ; 2.780874e+00 ; 4.333655e-01 ];
Tc_16  = [ 7.476818e+02 ; -1.654618e+02 ; 3.290970e+03 ];
omc_error_16 = [ 1.276994e-03 ; 2.697323e-03 ; 4.879812e-03 ];
Tc_error_16  = [ 7.878279e+00 ; 1.086541e+01 ; 5.940408e+00 ];

%-- Image #17:
omc_17 = [ -1.794634e-01 ; 2.876265e+00 ; 2.505015e-01 ];
Tc_17  = [ 8.593144e+02 ; -7.645442e+01 ; 3.409024e+03 ];
omc_error_17 = [ 1.132321e-03 ; 2.960340e-03 ; 5.475774e-03 ];
Tc_error_17  = [ 8.177082e+00 ; 1.130287e+01 ; 6.126104e+00 ];

%-- Image #18:
omc_18 = [ -2.518802e-02 ; 2.284915e+00 ; 5.180687e-01 ];
Tc_18  = [ 6.552201e+01 ; -3.069718e+02 ; 3.123819e+03 ];
omc_error_18 = [ 1.668386e-03 ; 2.496615e-03 ; 3.818142e-03 ];
Tc_error_18  = [ 7.455225e+00 ; 1.016714e+01 ; 5.313619e+00 ];

%-- Image #19:
omc_19 = [ 3.392566e-03 ; -2.308265e+00 ; 3.463833e-01 ];
Tc_19  = [ -2.455960e+02 ; -3.653889e+02 ; 3.090630e+03 ];
omc_error_19 = [ 1.974457e-03 ; 2.569836e-03 ; 3.816242e-03 ];
Tc_error_19  = [ 7.452149e+00 ; 1.009750e+01 ; 5.435745e+00 ];

%-- Image #20:
omc_20 = [ -9.604468e-03 ; -2.144442e+00 ; 3.305516e-01 ];
Tc_20  = [ -3.207557e+02 ; -3.719036e+02 ; 3.068974e+03 ];
omc_error_20 = [ 2.189490e-03 ; 2.528095e-03 ; 3.480683e-03 ];
Tc_error_20  = [ 7.415978e+00 ; 1.004653e+01 ; 5.470606e+00 ];

%-- Image #21:
omc_21 = [ 1.129784e-02 ; -2.134454e+00 ; 2.757871e-01 ];
Tc_21  = [ 2.703600e+02 ; -2.824087e+02 ; 3.354618e+03 ];
omc_error_21 = [ 1.925172e-03 ; 2.508359e-03 ; 3.527871e-03 ];
Tc_error_21  = [ 7.994362e+00 ; 1.095231e+01 ; 5.963991e+00 ];

%-- Image #22:
omc_22 = [ -1.202190e-02 ; -2.124927e+00 ; 2.539918e-01 ];
Tc_22  = [ 4.534278e+02 ; -2.470639e+02 ; 3.585004e+03 ];
omc_error_22 = [ 1.862219e-03 ; 2.507142e-03 ; 3.525473e-03 ];
Tc_error_22  = [ 8.548932e+00 ; 1.174166e+01 ; 6.399796e+00 ];

%-- Image #23:
omc_23 = [ -4.370475e-03 ; -2.266064e+00 ; 3.091387e-01 ];
Tc_23  = [ 6.083387e+02 ; -3.897759e+02 ; 3.779012e+03 ];
omc_error_23 = [ 1.623739e-03 ; 2.532357e-03 ; 3.742384e-03 ];
Tc_error_23  = [ 9.029771e+00 ; 1.240478e+01 ; 6.814676e+00 ];

