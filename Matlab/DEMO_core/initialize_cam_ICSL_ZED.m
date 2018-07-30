function [cam] = initialize_cam_ICSL_ZED(maxPyramidLevel)
% camera calibration parameters from ICSL ZED dataset


% intrinsic camera calibration parameters
cam.K = eye(3);
cam.K(1,1) = 349.877;
cam.K(2,2) = 349.877;
cam.K(1,3) = 339.84;
cam.K(2,3) = 182.562;
cam.k1 = -0.170099;
cam.k2 = 0.0257892;


% intrinsic camera calibration K parameters for multi-level pyramid
K_pyramid = zeros(3,3,maxPyramidLevel);
K_pyramid(:,:,1) = cam.K;

% perform pyramid reduction with average values
for k = 2:maxPyramidLevel
    K_pyramid(:,:,k) = downsampleKmatrix(K_pyramid(:,:,k-1));
end
cam.K_pyramid = K_pyramid;


% image size parameters
cam.nRows = 376;
cam.nCols = 672;


% scale factor to recover depth image
cam.scaleFactor = 1000;


end