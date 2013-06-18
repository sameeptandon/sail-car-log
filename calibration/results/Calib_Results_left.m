% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2247.991831429305876 ; 2258.863060349502121 ];

%-- Principal point:
cc = [ 720.884234446858272 ; 687.130491747326232 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.160281007166319 ; 0.347490701854282 ; 0.012856436456152 ; 0.003136042773537 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 43.140614769087193 ; 44.254394222476336 ];

%-- Principal point uncertainty:
cc_error = [ 45.474686263507337 ; 48.378210479157943 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.044496640906151 ; 0.209932994380736 ; 0.005254645624871 ; 0.003220585383417 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 960;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 27;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.995124e+00 ; 1.945581e+00 ; -3.566002e-01 ];
Tc_1  = [ 3.909738e+02 ; -9.595934e+01 ; 3.337215e+03 ];
omc_error_1 = [ 1.574285e-02 ; 1.505804e-02 ; 2.872287e-02 ];
Tc_error_1  = [ 6.739742e+01 ; 7.163855e+01 ; 6.653846e+01 ];

%-- Image #2:
omc_2 = [ 2.005518e+00 ; 2.006688e+00 ; -3.653899e-01 ];
Tc_2  = [ 2.507346e+02 ; -8.244054e+01 ; 3.258071e+03 ];
omc_error_2 = [ 1.481291e-02 ; 1.560783e-02 ; 2.885449e-02 ];
Tc_error_2  = [ 6.584729e+01 ; 6.977706e+01 ; 6.319791e+01 ];

%-- Image #3:
omc_3 = [ NaN ; NaN ; NaN ];
Tc_3  = [ NaN ; NaN ; NaN ];
omc_error_3 = [ NaN ; NaN ; NaN ];
Tc_error_3  = [ NaN ; NaN ; NaN ];

%-- Image #4:
omc_4 = [ NaN ; NaN ; NaN ];
Tc_4  = [ NaN ; NaN ; NaN ];
omc_error_4 = [ NaN ; NaN ; NaN ];
Tc_error_4  = [ NaN ; NaN ; NaN ];

%-- Image #5:
omc_5 = [ NaN ; NaN ; NaN ];
Tc_5  = [ NaN ; NaN ; NaN ];
omc_error_5 = [ NaN ; NaN ; NaN ];
Tc_error_5  = [ NaN ; NaN ; NaN ];

%-- Image #6:
omc_6 = [ 1.763505e+00 ; 1.656951e+00 ; -9.064568e-01 ];
Tc_6  = [ 3.971009e+02 ; -1.468486e+02 ; 3.995551e+03 ];
omc_error_6 = [ 1.658303e-02 ; 1.852461e-02 ; 2.605668e-02 ];
Tc_error_6  = [ 8.108296e+01 ; 8.566638e+01 ; 7.505600e+01 ];

%-- Image #7:
omc_7 = [ 1.582360e+00 ; 1.827265e+00 ; -9.299427e-01 ];
Tc_7  = [ 1.423680e+02 ; -2.117048e+02 ; 3.982807e+03 ];
omc_error_7 = [ 1.473660e-02 ; 1.980817e-02 ; 2.673502e-02 ];
Tc_error_7  = [ 8.080730e+01 ; 8.520605e+01 ; 7.305287e+01 ];

%-- Image #8:
omc_8 = [ 1.571955e+00 ; 1.913354e+00 ; -6.030091e-01 ];
Tc_8  = [ 1.589315e+02 ; -2.360348e+02 ; 3.942744e+03 ];
omc_error_8 = [ 1.433907e-02 ; 1.862020e-02 ; 2.685052e-02 ];
Tc_error_8  = [ 7.999653e+01 ; 8.435214e+01 ; 7.372157e+01 ];

%-- Image #9:
omc_9 = [ 1.805372e+00 ; 2.259956e+00 ; -5.187498e-01 ];
Tc_9  = [ 4.088968e+02 ; -3.426826e+02 ; 4.003159e+03 ];
omc_error_9 = [ 1.258581e-02 ; 1.871117e-02 ; 3.181845e-02 ];
Tc_error_9  = [ 8.137317e+01 ; 8.588974e+01 ; 7.706408e+01 ];

%-- Image #10:
omc_10 = [ -1.658458e+00 ; -2.558671e+00 ; -1.943781e-02 ];
Tc_10  = [ 5.067486e+02 ; -4.767593e+02 ; 4.660602e+03 ];
omc_error_10 = [ 2.065546e-02 ; 3.390190e-02 ; 5.690556e-02 ];
Tc_error_10  = [ 9.468217e+01 ; 1.000787e+02 ; 9.314867e+01 ];

%-- Image #11:
omc_11 = [ -1.908327e+00 ; -2.349170e+00 ; -1.021769e-01 ];
Tc_11  = [ 3.483526e+02 ; -5.954560e+02 ; 4.504221e+03 ];
omc_error_11 = [ 2.020793e-02 ; 2.670589e-02 ; 4.878021e-02 ];
Tc_error_11  = [ 9.164455e+01 ; 9.659439e+01 ; 8.972851e+01 ];

%-- Image #12:
omc_12 = [ -2.579566e+00 ; -1.088199e+00 ; -2.896037e-02 ];
Tc_12  = [ -9.375573e+01 ; -5.311209e+02 ; 4.397216e+03 ];
omc_error_12 = [ 2.145003e-02 ; 8.772474e-03 ; 2.997886e-02 ];
Tc_error_12  = [ 8.951907e+01 ; 9.423155e+01 ; 8.392408e+01 ];

%-- Image #13:
omc_13 = [ -2.044114e+00 ; -2.000038e+00 ; -1.421531e-01 ];
Tc_13  = [ 4.164195e+02 ; -6.292324e+02 ; 4.174683e+03 ];
omc_error_13 = [ 1.479667e-02 ; 1.705998e-02 ; 3.636436e-02 ];
Tc_error_13  = [ 8.524416e+01 ; 8.977120e+01 ; 8.314418e+01 ];

%-- Image #14:
omc_14 = [ -1.420119e+00 ; -2.219952e+00 ; -4.312496e-01 ];
Tc_14  = [ 6.258178e+02 ; -1.148986e+03 ; 4.015856e+03 ];
omc_error_14 = [ 1.022476e-02 ; 1.775442e-02 ; 3.017152e-02 ];
Tc_error_14  = [ 8.383575e+01 ; 8.778416e+01 ; 8.774527e+01 ];

%-- Image #15:
omc_15 = [ -2.034552e+00 ; -1.692261e+00 ; -5.204473e-01 ];
Tc_15  = [ 1.545940e+02 ; -1.092289e+03 ; 3.942988e+03 ];
omc_error_15 = [ 1.418811e-02 ; 1.507379e-02 ; 2.792153e-02 ];
Tc_error_15  = [ 8.176627e+01 ; 8.581834e+01 ; 8.289925e+01 ];

%-- Image #16:
omc_16 = [ -2.051119e+00 ; -1.679641e+00 ; -5.197624e-01 ];
Tc_16  = [ 1.831246e+02 ; -7.192468e+02 ; 3.822572e+03 ];
omc_error_16 = [ 1.399980e-02 ; 1.619176e-02 ; 2.797672e-02 ];
Tc_error_16  = [ 7.837159e+01 ; 8.229637e+01 ; 7.679312e+01 ];

%-- Image #17:
omc_17 = [ -1.029980e+00 ; -2.435127e+00 ; -1.567085e-02 ];
Tc_17  = [ 6.927918e+02 ; -7.647395e+02 ; 4.348656e+03 ];
omc_error_17 = [ 9.776342e-03 ; 1.903297e-02 ; 3.024931e-02 ];
Tc_error_17  = [ 8.945290e+01 ; 9.397631e+01 ; 9.053001e+01 ];

%-- Image #18:
omc_18 = [ -5.777071e-01 ; -3.067547e+00 ; -1.124857e-01 ];
Tc_18  = [ 6.440486e+02 ; -6.899424e+02 ; 5.260328e+03 ];
omc_error_18 = [ 7.522493e-03 ; 3.201300e-02 ; 5.000233e-02 ];
Tc_error_18  = [ 1.071388e+02 ; 1.130999e+02 ; 1.064682e+02 ];

%-- Image #19:
omc_19 = [ -2.624586e+00 ; -9.778625e-01 ; -2.514024e-01 ];
Tc_19  = [ 1.378257e+02 ; -6.748594e+02 ; 5.308808e+03 ];
omc_error_19 = [ 2.001455e-02 ; 1.051365e-02 ; 2.884730e-02 ];
Tc_error_19  = [ 1.081594e+02 ; 1.139381e+02 ; 1.026793e+02 ];

%-- Image #20:
omc_20 = [ -2.626547e+00 ; -1.015404e+00 ; -2.659941e-01 ];
Tc_20  = [ -2.329213e+02 ; -1.098444e+03 ; 5.270269e+03 ];
omc_error_20 = [ 2.281605e-02 ; 9.759733e-03 ; 3.506174e-02 ];
Tc_error_20  = [ 1.083385e+02 ; 1.142417e+02 ; 1.048632e+02 ];

%-- Image #21:
omc_21 = [ -1.771641e+00 ; -2.581680e+00 ; 2.027050e-01 ];
Tc_21  = [ 6.601955e+02 ; -1.205283e+03 ; 5.061387e+03 ];
omc_error_21 = [ 1.812034e-02 ; 1.940511e-02 ; 4.027056e-02 ];
Tc_error_21  = [ 1.048684e+02 ; 1.097388e+02 ; 1.047722e+02 ];

%-- Image #22:
omc_22 = [ 1.555561e+00 ; 1.992758e+00 ; -5.710994e-01 ];
Tc_22  = [ 7.693545e+02 ; -6.050081e+02 ; 5.190932e+03 ];
omc_error_22 = [ 1.543790e-02 ; 1.929918e-02 ; 2.764588e-02 ];
Tc_error_22  = [ 1.059879e+02 ; 1.119752e+02 ; 1.028230e+02 ];

%-- Image #23:
omc_23 = [ 2.402914e+00 ; 1.790955e+00 ; -8.683644e-01 ];
Tc_23  = [ 1.363379e+02 ; -4.661153e+02 ; 5.123434e+03 ];
omc_error_23 = [ 1.450302e-02 ; 1.908523e-02 ; 3.495924e-02 ];
Tc_error_23  = [ 1.041491e+02 ; 1.097129e+02 ; 9.616881e+01 ];

%-- Image #24:
omc_24 = [ -2.769799e+00 ; -1.310918e+00 ; -9.702135e-02 ];
Tc_24  = [ -1.719007e+02 ; -5.456381e+02 ; 5.007843e+03 ];
omc_error_24 = [ 3.760577e-02 ; 1.791876e-02 ; 6.507481e-02 ];
Tc_error_24  = [ 1.019086e+02 ; 1.071932e+02 ; 9.969244e+01 ];

%-- Image #25:
omc_25 = [ -5.033452e-01 ; -2.460728e+00 ; -2.727209e-01 ];
Tc_25  = [ 8.069660e+02 ; -7.816348e+02 ; 4.747003e+03 ];
omc_error_25 = [ 7.423773e-03 ; 2.025427e-02 ; 2.831439e-02 ];
Tc_error_25  = [ 9.753104e+01 ; 1.026876e+02 ; 1.001098e+02 ];

%-- Image #26:
omc_26 = [ -7.424143e-02 ; -2.450976e+00 ; -2.191424e-01 ];
Tc_26  = [ 4.928169e+02 ; -8.650237e+02 ; 4.724025e+03 ];
omc_error_26 = [ 8.610929e-03 ; 2.081669e-02 ; 2.833919e-02 ];
Tc_error_26  = [ 9.681958e+01 ; 1.014114e+02 ; 9.967860e+01 ];

%-- Image #27:
omc_27 = [ -4.109334e-01 ; -3.181377e+00 ; -1.944888e-01 ];
Tc_27  = [ 1.263714e+02 ; -1.005577e+03 ; 4.728307e+03 ];
omc_error_27 = [ 7.557179e-03 ; 3.019011e-02 ; 4.639153e-02 ];
Tc_error_27  = [ 9.710270e+01 ; 1.014125e+02 ; 9.779011e+01 ];

