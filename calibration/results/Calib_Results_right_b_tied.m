% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2277.881756946957921 ; 2278.644315311918945 ];

%-- Principal point:
cc = [ 622.754205234751794 ; 418.007565860560817 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.198090181491310 ; 0.328969817188631 ; -0.004099071938090 ; -0.001439868517800 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 5.542883951275813 ; 5.358058500237886 ];

%-- Principal point uncertainty:
cc_error = [ 8.232389524434668 ; 10.207685273339854 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.013545738597365 ; 0.139314601289044 ; 0.000762588908744 ; 0.000749652394411 ; 0.000000000000000 ];

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
omc_1 = [ -5.458824e-02 ; -2.246968e+00 ; 2.239288e-01 ];
Tc_1  = [ 7.246096e+02 ; -6.051497e+02 ; 3.768089e+03 ];
omc_error_1 = [ 2.248110e-03 ; 3.826150e-03 ; 5.133669e-03 ];
Tc_error_1  = [ 1.366662e+01 ; 1.706176e+01 ; 1.038784e+01 ];

%-- Image #2:
omc_2 = [ -5.330179e-02 ; -2.101555e+00 ; 1.663796e-01 ];
Tc_2  = [ 5.835782e+02 ; -4.302896e+02 ; 3.598758e+03 ];
omc_error_2 = [ 2.551931e-03 ; 3.766407e-03 ; 4.803039e-03 ];
Tc_error_2  = [ 1.299996e+01 ; 1.625703e+01 ; 9.715384e+00 ];

%-- Image #3:
omc_3 = [ -3.168150e-02 ; -2.100478e+00 ; 1.782296e-01 ];
Tc_3  = [ 4.006154e+02 ; -4.427559e+02 ; 3.369807e+03 ];
omc_error_3 = [ 2.646190e-03 ; 3.754896e-03 ; 4.785951e-03 ];
Tc_error_3  = [ 1.215254e+01 ; 1.515981e+01 ; 9.047966e+00 ];

%-- Image #4:
omc_4 = [ -4.063851e-02 ; -2.110942e+00 ; 2.498108e-01 ];
Tc_4  = [ -2.080902e+02 ; -4.838204e+02 ; 3.084204e+03 ];
omc_error_4 = [ 3.009108e-03 ; 3.792946e-03 ; 4.726738e-03 ];
Tc_error_4  = [ 1.122243e+01 ; 1.383704e+01 ; 8.142686e+00 ];

%-- Image #5:
omc_5 = [ -5.084601e-02 ; -2.267846e+00 ; 2.558210e-01 ];
Tc_5  = [ -1.666989e+02 ; -4.833689e+02 ; 3.094687e+03 ];
omc_error_5 = [ 2.734665e-03 ; 3.850027e-03 ; 5.141680e-03 ];
Tc_error_5  = [ 1.125196e+01 ; 1.387414e+01 ; 8.093851e+00 ];

%-- Image #6:
omc_6 = [ 7.654042e-02 ; 2.303914e+00 ; 5.338128e-01 ];
Tc_6  = [ 1.697593e+02 ; -4.468234e+02 ; 3.132335e+03 ];
omc_error_6 = [ 2.396891e-03 ; 3.672530e-03 ; 5.350029e-03 ];
Tc_error_6  = [ 1.129051e+01 ; 1.401342e+01 ; 7.904704e+00 ];

%-- Image #7:
omc_7 = [ -7.599498e-02 ; 2.892689e+00 ; 3.260782e-01 ];
Tc_7  = [ 9.851950e+02 ; -2.784697e+02 ; 3.386744e+03 ];
omc_error_7 = [ 1.655580e-03 ; 4.189887e-03 ; 7.291846e-03 ];
Tc_error_7  = [ 1.229832e+01 ; 1.550400e+01 ; 9.334671e+00 ];

%-- Image #8:
omc_8 = [ 8.291756e-02 ; 2.790400e+00 ; 4.795700e-01 ];
Tc_8  = [ 8.617400e+02 ; -3.581991e+02 ; 3.284244e+03 ];
omc_error_8 = [ 1.945231e-03 ; 3.990572e-03 ; 6.798149e-03 ];
Tc_error_8  = [ 1.190893e+01 ; 1.496221e+01 ; 9.128789e+00 ];

%-- Image #9:
omc_9 = [ 3.956146e-01 ; 2.738153e+00 ; -1.334765e-01 ];
Tc_9  = [ 4.362942e+02 ; -5.683355e+02 ; 3.808801e+03 ];
omc_error_9 = [ 1.661080e-03 ; 4.004358e-03 ; 6.759930e-03 ];
Tc_error_9  = [ 1.374071e+01 ; 1.715905e+01 ; 9.505382e+00 ];

%-- Image #10:
omc_10 = [ 3.007323e-01 ; 2.794091e+00 ; -2.664725e-01 ];
Tc_10  = [ 5.156748e+02 ; -6.039612e+02 ; 3.934519e+03 ];
omc_error_10 = [ 1.538663e-03 ; 4.112576e-03 ; 6.917703e-03 ];
Tc_error_10  = [ 1.421225e+01 ; 1.775228e+01 ; 9.786771e+00 ];

%-- Image #11:
omc_11 = [ -6.261503e-03 ; 2.873379e+00 ; -8.730751e-01 ];
Tc_11  = [ 1.025061e+03 ; -5.703680e+02 ; 4.281442e+03 ];
omc_error_11 = [ 2.199532e-03 ; 4.488371e-03 ; 6.911220e-03 ];
Tc_error_11  = [ 1.558416e+01 ; 1.961017e+01 ; 1.100704e+01 ];

%-- Image #12:
omc_12 = [ -2.711697e-01 ; -2.978317e+00 ; 1.899866e-01 ];
Tc_12  = [ 8.642624e+02 ; -5.505330e+02 ; 3.930129e+03 ];
omc_error_12 = [ 1.401371e-03 ; 9.487372e-03 ; 1.116664e-02 ];
Tc_error_12  = [ 1.430295e+01 ; 1.787903e+01 ; 1.061125e+01 ];

%-- Image #13:
omc_13 = [ 1.234267e-01 ; 2.459223e+00 ; -1.195455e+00 ];
Tc_13  = [ 9.035028e+01 ; -5.313892e+02 ; 4.037753e+03 ];
omc_error_13 = [ 2.555170e-03 ; 4.317056e-03 ; 5.665106e-03 ];
Tc_error_13  = [ 1.464343e+01 ; 1.812596e+01 ; 8.984785e+00 ];

%-- Image #14:
omc_14 = [ 1.087984e-01 ; 2.534023e+00 ; -1.144383e+00 ];
Tc_14  = [ 1.079459e+02 ; -5.554605e+02 ; 3.990385e+03 ];
omc_error_14 = [ 2.421101e-03 ; 4.270347e-03 ; 5.844814e-03 ];
Tc_error_14  = [ 1.447532e+01 ; 1.790811e+01 ; 8.972161e+00 ];

%-- Image #15:
omc_15 = [ 1.961868e-01 ; 2.876925e+00 ; -2.264214e-01 ];
Tc_15  = [ 2.086451e+02 ; -5.941318e+02 ; 3.644504e+03 ];
omc_error_15 = [ 1.061591e-03 ; 4.320265e-03 ; 7.397605e-03 ];
Tc_error_15  = [ 1.318371e+01 ; 1.632563e+01 ; 9.045346e+00 ];

%-- Image #16:
omc_16 = [ 2.795331e-01 ; 2.592926e+00 ; -1.022734e+00 ];
Tc_16  = [ 3.042455e+02 ; -4.392373e+02 ; 4.385113e+03 ];
omc_error_16 = [ 2.142833e-03 ; 4.377682e-03 ; 5.870412e-03 ];
Tc_error_16  = [ 1.584683e+01 ; 1.969504e+01 ; 1.000967e+01 ];

%-- Image #17:
omc_17 = [ -2.622280e-01 ; -2.873251e+00 ; 2.586712e-02 ];
Tc_17  = [ 9.852719e+02 ; -6.435941e+02 ; 4.116231e+03 ];
omc_error_17 = [ 1.190514e-03 ; 7.994724e-03 ; 9.319830e-03 ];
Tc_error_17  = [ 1.506876e+01 ; 1.880355e+01 ; 1.154703e+01 ];

%-- Image #18:
omc_18 = [ -3.154950e-01 ; -2.927379e+00 ; 8.315102e-03 ];
Tc_18  = [ 8.426070e+02 ; -6.229054e+02 ; 4.091486e+03 ];
omc_error_18 = [ 1.216706e-03 ; 8.499122e-03 ; 9.982294e-03 ];
Tc_error_18  = [ 1.493470e+01 ; 1.860809e+01 ; 1.149127e+01 ];

%-- Image #19:
omc_19 = [ -3.218223e-01 ; -3.011449e+00 ; 7.904538e-03 ];
Tc_19  = [ 6.723979e+02 ; -6.353968e+02 ; 4.016855e+03 ];
omc_error_19 = [ 1.189592e-03 ; 8.643931e-03 ; 1.049322e-02 ];
Tc_error_19  = [ 1.461669e+01 ; 1.816717e+01 ; 1.121207e+01 ];

%-- Image #20:
omc_20 = [ 2.913719e-01 ; 3.085969e+00 ; -7.038928e-02 ];
Tc_20  = [ 5.258679e+02 ; -6.536825e+02 ; 3.990908e+03 ];
omc_error_20 = [ 1.058389e-03 ; 6.313917e-03 ; 9.747166e-03 ];
Tc_error_20  = [ 1.444047e+01 ; 1.796887e+01 ; 1.060144e+01 ];

%-- Image #21:
omc_21 = [ 2.562934e-01 ; 2.778182e+00 ; -5.976880e-02 ];
Tc_21  = [ 1.801758e+02 ; -5.754058e+02 ; 3.616109e+03 ];
omc_error_21 = [ 1.272919e-03 ; 4.144070e-03 ; 6.930782e-03 ];
Tc_error_21  = [ 1.306636e+01 ; 1.620600e+01 ; 9.012736e+00 ];

%-- Image #22:
omc_22 = [ 8.776877e-02 ; 2.314017e+00 ; -5.566419e-02 ];
Tc_22  = [ -1.087560e+02 ; -4.830742e+02 ; 3.269123e+03 ];
omc_error_22 = [ 2.093188e-03 ; 3.612784e-03 ; 5.372585e-03 ];
Tc_error_22  = [ 1.187360e+01 ; 1.463632e+01 ; 7.611619e+00 ];

%-- Image #23:
omc_23 = [ 2.105895e-01 ; 2.264606e+00 ; -4.470597e-02 ];
Tc_23  = [ -2.220535e+02 ; -5.438869e+02 ; 3.361920e+03 ];
omc_error_23 = [ 2.151940e-03 ; 3.584966e-03 ; 5.237472e-03 ];
Tc_error_23  = [ 1.224002e+01 ; 1.507193e+01 ; 7.935789e+00 ];

