% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2223.484302119193671 ; 2206.728881569085388 ];

%-- Principal point:
cc = [ 815.954285240489071 ; 765.861936166988812 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.142700747520210 ; 0.074955802902143 ; 0.011899624653834 ; 0.012207179799330 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 52.192530847835883 ; 51.230345670136728 ];

%-- Principal point uncertainty:
cc_error = [ 52.861632975889009 ; 42.654109594192668 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.044432839575884 ; 0.083990557275220 ; 0.003498436193299 ; 0.006409675904406 ; 0.000000000000000 ];

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
omc_1 = [ 1.983551e+00 ; 1.913018e+00 ; -1.981603e-01 ];
Tc_1  = [ -8.721325e+02 ; -2.050430e+02 ; 3.175685e+03 ];
omc_error_1 = [ 1.286680e-02 ; 1.918957e-02 ; 3.018211e-02 ];
Tc_error_1  = [ 7.590596e+01 ; 6.250736e+01 ; 7.395741e+01 ];

%-- Image #2:
omc_2 = [ 1.990428e+00 ; 1.984472e+00 ; -1.978908e-01 ];
Tc_2  = [ -1.017362e+03 ; -1.917255e+02 ; 3.085598e+03 ];
omc_error_2 = [ 1.258531e-02 ; 2.024250e-02 ; 2.946771e-02 ];
Tc_error_2  = [ 7.361383e+01 ; 6.122948e+01 ; 7.492816e+01 ];

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
omc_6 = [ 1.799641e+00 ; 1.615765e+00 ; -7.642409e-01 ];
Tc_6  = [ -9.313948e+02 ; -2.695839e+02 ; 3.827637e+03 ];
omc_error_6 = [ 1.189483e-02 ; 2.250753e-02 ; 2.656541e-02 ];
Tc_error_6  = [ 9.162657e+01 ; 7.512814e+01 ; 8.612576e+01 ];

%-- Image #7:
omc_7 = [ 1.615080e+00 ; 1.773967e+00 ; -8.074779e-01 ];
Tc_7  = [ -1.187966e+03 ; -3.388620e+02 ; 3.796509e+03 ];
omc_error_7 = [ 1.039698e-02 ; 2.414971e-02 ; 2.619166e-02 ];
Tc_error_7  = [ 9.095942e+01 ; 7.529827e+01 ; 8.821639e+01 ];

%-- Image #8:
omc_8 = [ 1.572227e+00 ; 1.882852e+00 ; -4.471933e-01 ];
Tc_8  = [ -1.186974e+03 ; -3.693086e+02 ; 3.752150e+03 ];
omc_error_8 = [ 1.026959e-02 ; 2.317090e-02 ; 2.623490e-02 ];
Tc_error_8  = [ 8.993812e+01 ; 7.440955e+01 ; 8.765751e+01 ];

%-- Image #9:
omc_9 = [ 1.795424e+00 ; 2.231520e+00 ; -3.595161e-01 ];
Tc_9  = [ -9.381632e+02 ; -4.669109e+02 ; 3.830992e+03 ];
omc_error_9 = [ 9.240225e-03 ; 2.220055e-02 ; 3.094018e-02 ];
Tc_error_9  = [ 9.195592e+01 ; 7.520691e+01 ; 8.894419e+01 ];

%-- Image #10:
omc_10 = [ -1.669219e+00 ; -2.611503e+00 ; -2.185449e-01 ];
Tc_10  = [ -8.904234e+02 ; -6.140249e+02 ; 4.420699e+03 ];
omc_error_10 = [ 1.632093e-02 ; 2.175692e-02 ; 4.533632e-02 ];
Tc_error_10  = [ 1.055976e+02 ; 8.651631e+01 ; 1.026366e+02 ];

%-- Image #11:
omc_11 = [ -1.884381e+00 ; -2.395413e+00 ; -2.584011e-01 ];
Tc_11  = [ -1.025199e+03 ; -7.286593e+02 ; 4.264116e+03 ];
omc_error_11 = [ 1.806224e-02 ; 1.923935e-02 ; 3.992759e-02 ];
Tc_error_11  = [ 1.021432e+02 ; 8.391087e+01 ; 1.011718e+02 ];

%-- Image #12:
omc_12 = [ -2.572230e+00 ; -1.123000e+00 ; -1.404852e-01 ];
Tc_12  = [ -1.469313e+03 ; -6.700776e+02 ; 4.148201e+03 ];
omc_error_12 = [ 2.210221e-02 ; 7.426264e-03 ; 3.412275e-02 ];
Tc_error_12  = [ 9.952153e+01 ; 8.311406e+01 ; 1.028868e+02 ];

%-- Image #13:
omc_13 = [ -2.056657e+00 ; -2.047176e+00 ; -2.491330e-01 ];
Tc_13  = [ -9.409385e+02 ; -7.592801e+02 ; 3.971190e+03 ];
omc_error_13 = [ 1.784267e-02 ; 1.592650e-02 ; 3.152856e-02 ];
Tc_error_13  = [ 9.523956e+01 ; 7.816951e+01 ; 9.452860e+01 ];

%-- Image #14:
omc_14 = [ -1.404927e+00 ; -2.303106e+00 ; -5.348716e-01 ];
Tc_14  = [ -7.166751e+02 ; -1.269679e+03 ; 3.821454e+03 ];
omc_error_14 = [ 1.178250e-02 ; 1.949939e-02 ; 2.524492e-02 ];
Tc_error_14  = [ 9.328007e+01 ; 7.579846e+01 ; 9.655592e+01 ];

%-- Image #15:
omc_15 = [ -2.011507e+00 ; -1.755472e+00 ; -6.332191e-01 ];
Tc_15  = [ -1.177208e+03 ; -1.226223e+03 ; 3.720626e+03 ];
omc_error_15 = [ 1.605159e-02 ; 1.537500e-02 ; 2.761942e-02 ];
Tc_error_15  = [ 9.133602e+01 ; 7.564291e+01 ; 9.710149e+01 ];

%-- Image #16:
omc_16 = [ -2.056507e+00 ; -1.714836e+00 ; -6.468689e-01 ];
Tc_16  = [ -1.149793e+03 ; -8.464859e+02 ; 3.623309e+03 ];
omc_error_16 = [ 1.671417e-02 ; 1.640574e-02 ; 2.899280e-02 ];
Tc_error_16  = [ 8.789298e+01 ; 7.246896e+01 ; 9.046401e+01 ];

%-- Image #17:
omc_17 = [ -1.031737e+00 ; -2.513549e+00 ; -9.902272e-02 ];
Tc_17  = [ -6.767069e+02 ; -8.926752e+02 ; 4.143191e+03 ];
omc_error_17 = [ 1.235683e-02 ; 2.057017e-02 ; 2.649022e-02 ];
Tc_error_17  = [ 9.952871e+01 ; 8.099296e+01 ; 1.003206e+02 ];

%-- Image #18:
omc_18 = [ -5.386282e-01 ; -3.076174e+00 ; -2.606486e-01 ];
Tc_18  = [ -8.121464e+02 ; -8.378221e+02 ; 4.996447e+03 ];
omc_error_18 = [ 7.162887e-03 ; 2.595071e-02 ; 4.252173e-02 ];
Tc_error_18  = [ 1.195161e+02 ; 9.763405e+01 ; 1.167887e+02 ];

%-- Image #19:
omc_19 = [ -2.615386e+00 ; -1.049859e+00 ; -3.653836e-01 ];
Tc_19  = [ -1.331029e+03 ; -8.422278e+02 ; 5.070702e+03 ];
omc_error_19 = [ 2.396261e-02 ; 9.709910e-03 ; 3.474512e-02 ];
Tc_error_19  = [ 1.220022e+02 ; 1.001844e+02 ; 1.192380e+02 ];

%-- Image #20:
omc_20 = [ -2.626584e+00 ; -1.026546e+00 ; -3.601996e-01 ];
Tc_20  = [ -1.702023e+03 ; -1.255224e+03 ; 4.981790e+03 ];
omc_error_20 = [ 2.601756e-02 ; 7.337636e-03 ; 3.835432e-02 ];
Tc_error_20  = [ 1.205279e+02 ; 1.003327e+02 ; 1.281075e+02 ];

%-- Image #21:
omc_21 = [ 1.768588e+00 ; 2.562747e+00 ; 4.900137e-03 ];
Tc_21  = [ -7.921225e+02 ; -1.371662e+03 ; 4.870279e+03 ];
omc_error_21 = [ 1.115178e-02 ; 2.601851e-02 ; 3.981628e-02 ];
Tc_error_21  = [ 1.187909e+02 ; 9.475261e+01 ; 1.217042e+02 ];

%-- Image #22:
omc_22 = [ 1.553780e+00 ; 1.942036e+00 ; -4.379356e-01 ];
Tc_22  = [ -6.879630e+02 ; -7.664052e+02 ; 5.021750e+03 ];
omc_error_22 = [ 1.184326e-02 ; 2.340908e-02 ; 2.603938e-02 ];
Tc_error_22  = [ 1.204651e+02 ; 9.738957e+01 ; 1.136189e+02 ];

%-- Image #23:
omc_23 = [ 2.416271e+00 ; 1.792354e+00 ; -7.063995e-01 ];
Tc_23  = [ -1.304668e+03 ; -6.221802e+02 ; 4.884004e+03 ];
omc_error_23 = [ 1.043424e-02 ; 2.156796e-02 ; 3.447428e-02 ];
Tc_error_23  = [ 1.173292e+02 ; 9.644839e+01 ; 1.165188e+02 ];

%-- Image #24:
omc_24 = [ -2.733562e+00 ; -1.307228e+00 ; -2.202491e-01 ];
Tc_24  = [ -1.611652e+03 ; -6.991670e+02 ; 4.734724e+03 ];
omc_error_24 = [ 2.681586e-02 ; 7.927994e-03 ; 4.499215e-02 ];
Tc_error_24  = [ 1.138215e+02 ; 9.456564e+01 ; 1.181598e+02 ];

%-- Image #25:
omc_25 = [ -5.060959e-01 ; -2.529163e+00 ; -3.324829e-01 ];
Tc_25  = [ -6.012670e+02 ; -9.183080e+02 ; 4.547762e+03 ];
omc_error_25 = [ 9.671615e-03 ; 2.296984e-02 ; 2.488077e-02 ];
Tc_error_25  = [ 1.091412e+02 ; 8.868123e+01 ; 1.090979e+02 ];

%-- Image #26:
omc_26 = [ -5.949459e-02 ; -2.545995e+00 ; -2.635329e-01 ];
Tc_26  = [ -9.071103e+02 ; -1.007383e+03 ; 4.497913e+03 ];
omc_error_26 = [ 1.073165e-02 ; 2.392734e-02 ; 2.481904e-02 ];
Tc_error_26  = [ 1.085025e+02 ; 8.806201e+01 ; 1.122777e+02 ];

%-- Image #27:
omc_27 = [ 3.313343e-01 ; 2.982204e+00 ; 3.618931e-01 ];
Tc_27  = [ -1.284387e+03 ; -1.145632e+03 ; 4.448086e+03 ];
omc_error_27 = [ 7.544449e-03 ; 3.320131e-02 ; 4.550520e-02 ];
Tc_error_27  = [ 1.079740e+02 ; 8.802933e+01 ; 1.144151e+02 ];

