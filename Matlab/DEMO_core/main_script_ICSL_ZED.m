clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;


%% basic setup for DEMO

% choose the experiment case
% ICSL RGBD dataset (1~XX)
expCase = 1;

% are figures drawn?
% 1 : yes, draw figures to see current status
% 0 : no, just run DEMO
toVisualize = 1;

% are data results saved?
% 1 : yes, save the variables and results
% 0 : no, just run DEMO
toSave = 1;


setupParams_ICSL_ZED;


% load ICSL RGBD dataset data
ICSLZEDdataset = rawICSLRGBDdataset_load(datasetPath, imInit, M);


% camera calibration parameters
cam = initialize_cam_ICSL_ZED(3);


%% main DEMO part


% 1. feature tracking pre-defined variables
systemInited_ft = false;

imageHeight = 376;
imageWidth = 672;
imagePixelNum = imageHeight * imageWidth;
imgSize = [imageWidth imageHeight];

imageCur = uint8(zeros(imgSize(2), imgSize(1)));
imageLast = uint8(zeros(imgSize(2), imgSize(1)));

showCount = 0;
showSkipNum = 2;
showDSRate = 2;
showSize = [imageWidth/showDSRate imageHeight/showDSRate];

imageShow = uint8(zeros(showSize(2), showSize(1)));
harrisLast = uint8(zeros(showSize(2), showSize(1)));

maxFeatureNumPerSubregion = 2;
xSubregionNum = 16;
ySubregionNum = 8;
totalSubregionNum = xSubregionNum * ySubregionNum;
MAXFEATURENUM = maxFeatureNumPerSubregion * totalSubregionNum;

xBoundary = 16;
yBoundary = 8;
subregionWidth = (imageWidth - 2 * xBoundary) / xSubregionNum;
subregionHeight = (imageHeight - 2 * yBoundary) / ySubregionNum;

maxTrackDis = 100;
winSize = 15;

featuresCur = cell(1, 2 * MAXFEATURENUM);
featuresLast = cell(1, 2 * MAXFEATURENUM);
featuresFound = zeros(1, 2 * MAXFEATURENUM);
featuresError = zeros(1, 2 * MAXFEATURENUM);

featuresIndFromStart = 0;
featuresInd = zeros(1, 2 * MAXFEATURENUM);

totalFeatureNum = 0;
subregionFeatureNum = zeros(1, 2 * totalSubregionNum);

imagePointsCur = cell(0);
imagePointsLast = cell(0);


% 2. frame to frame motion estimation pre-defined variables
systemInited_vo = false;

T_gc_current = eye(4);
T_gc_DEMO = cell(1, M);
T_gc_DEMO{1} = T_gc_current;


% 3. make figures to visualize current status
if (toVisualize)
    % create figure
    h = figure(10);
    set(h,'Color',[1 1 1]);
    set(h,'Units','pixels','Position',[150 400 1700 600]);
    ha1 = axes('Position',[0.025,0.1 , 0.3,0.8]);
    axis off;
    ha2 = axes('Position',[0.350,0.1 , 0.3,0.8]);
    axis off;
    ha3 = axes('Position',[0.675,0.1 , 0.3,0.8]);
    axis equal; grid on; hold on;
end


% do DEMO
for imgIdx = 1:M
    %% 1. feature tracking : re-assign variables for next ieration
    
    % image
    imageLast = imageCur;
    imageCur = getImgInTUMRGBDdataset(datasetPath, ICSLZEDdataset, cam, imgIdx, 'gray');
    
    imageShow = imresize(imageLast, (1/showDSRate));
    harrisLast = cv.cornerHarris(imageShow, 'BlockSize', 3);
    
    % features
    featuresTemp = featuresLast;
    featuresLast = featuresCur;
    featuresCur = featuresTemp;
    
    % image points
    imagePointsTemp = imagePointsLast;
    imagePointsLast = imagePointsCur;
    imagePointsCur = imagePointsTemp;
    imagePointsCur = cell(0);
    
    % for the first time in this loop
    if (~systemInited_ft)
        systemInited_ft = true;
    elseif (systemInited_ft)
        %% 1. feature tracking : feature detection with goodFeaturesToTrack
        
        recordFeatureNum = totalFeatureNum;
        for i = 1:ySubregionNum
            for j = 1:xSubregionNum
                ind = xSubregionNum * (i-1) + j;
                numToFind = maxFeatureNumPerSubregion - subregionFeatureNum(ind);
                
                if (numToFind>0)
                    
                    subregionLeft = xBoundary + (subregionWidth * (j-1)) + 1;
                    subregionTop = yBoundary + (subregionHeight * (i-1)) + 1;
                    
                    uStart = subregionLeft;
                    uEnd = subregionLeft + subregionWidth - 1;
                    vStart = subregionTop;
                    vEnd = subregionTop + subregionHeight - 1;
                    subregionTemp = imageLast(vStart:vEnd, uStart:uEnd);
                    
                    featuresTemp = cv.goodFeaturesToTrack(subregionTemp, 'QualityLevel', 0.1, 'MinDistance', 5.0, 'BlockSize', 3, 'UseHarrisDetector', true, 'K', 0.04);
                    for k=1:size(featuresTemp, 2)
                        featuresLast(totalFeatureNum+k) = featuresTemp(k);
                    end
                    
                    numFound = 0;
                    for k=1:size(featuresTemp, 2)
                        featuresLast{totalFeatureNum+k}(1) = featuresLast{totalFeatureNum+k}(1) + (subregionLeft-1);
                        featuresLast{totalFeatureNum+k}(2) = featuresLast{totalFeatureNum+k}(2) + (subregionTop-1);
                        
                        xInd = round( (featuresLast{totalFeatureNum+k}(1) + 0.5) / showDSRate );
                        yInd = round( (featuresLast{totalFeatureNum+k}(2) + 0.5) / showDSRate );
                        
                        if (harrisLast(yInd, xInd) >= 1e-6)
                            numFound = numFound + 1;
                            featuresIndFromStart = featuresIndFromStart +1;
                            
                            featuresLast{totalFeatureNum+numFound}(1) = featuresLast{totalFeatureNum+k}(1);
                            featuresLast{totalFeatureNum+numFound}(2) = featuresLast{totalFeatureNum+k}(2);
                            featuresInd(totalFeatureNum+numFound) = featuresIndFromStart;
                        end
                    end
                    
                    totalFeatureNum = totalFeatureNum + numFound;
                    subregionFeatureNum(ind) = subregionFeatureNum(ind) + numFound;
                end
            end
        end
        
        
        %% 1. feature tracking : feature tracking with KLT tracker
        
        [featuresCur, featuresFound, featuresError]  = cv.calcOpticalFlowPyrLK(imageLast, imageCur, featuresLast(1:totalFeatureNum), 'WinSize', [winSize winSize], 'MaxLevel', 3);
        
        for i=1:totalSubregionNum
            subregionFeatureNum(i) = 0;
        end
        
        featureCount = 0;
        for i=1:totalFeatureNum
            trackDis = sqrt( (featuresLast{i}(1) - featuresCur{i}(1))*(featuresLast{i}(1) - featuresCur{i}(1)) + (featuresLast{i}(2) - featuresCur{i}(2))*(featuresLast{i}(2) - featuresCur{i}(2)) );
            
            if (~(trackDis > maxTrackDis || featuresCur{i}(1) < xBoundary || featuresCur{i}(1) > (imageWidth - xBoundary) ...
                    || featuresCur{i}(2) < yBoundary || featuresCur{i}(2) > (imageHeight - yBoundary)))
                
                xInd = floor( (featuresLast{i}(1) - xBoundary) / subregionWidth ) + 1;
                yInd = floor( (featuresLast{i}(2) - yBoundary) / subregionHeight ) + 1;
                ind = xSubregionNum * (yInd-1) + xInd;
                
                if (subregionFeatureNum(ind) < maxFeatureNumPerSubregion)
                    featureCount = featureCount + 1;
                    subregionFeatureNum(ind) = subregionFeatureNum(ind) + 1;
                    
                    featuresCur{featureCount}(1) = featuresCur{i}(1);
                    featuresCur{featureCount}(2) = featuresCur{i}(2);
                    featuresLast{featureCount}(1) = featuresLast{i}(1);
                    featuresLast{featureCount}(2) = featuresLast{i}(2);
                    featuresInd(featureCount) = featuresInd(i);
                    
                    point.u = (featuresCur{featureCount}(1) - cam.K(1,3)) / cam.K(1,1);
                    point.v = (featuresCur{featureCount}(2) - cam.K(2,3)) / cam.K(2,2);
                    point.ind = featuresInd(featureCount);
                    imagePointsCur{end+1} = point;
                    
                    if (i > recordFeatureNum)
                        point.u = (featuresLast{featureCount}(1) - cam.K(1,3)) / cam.K(1,1);
                        point.v = (featuresLast{featureCount}(2) - cam.K(2,3)) / cam.K(2,2);
                        imagePointsLast{end+1} = point;
                    end
                end
            end
        end
        totalFeatureNum = featureCount;
    end
    
    
    %% 2. frame to frame motion estimation : re-assign variables for next ieration
    
    % for the first time in this loop
    if (~systemInited_vo)
        systemInited_vo = true;
    elseif  (systemInited_vo)
        %% 2. frame to frame motion estimation : find relationship between points
        
        depthImageTemp = getImgInTUMRGBDdataset(datasetPath, ICSLZEDdataset, cam, imgIdx-1, 'depth');
        ipRelations = findPointTrackingResult(imagePointsLast, imagePointsCur, depthImageTemp, cam);
        
        
        %% 2. frame to frame motion estimation : estimate 6 DoF motion
        
        % estimate the rotational motion [R] & translational motion [t]
        optsMWO.iterNum = 150;
        [xi_21, ipRelations] = estimatePoseAllRANSAC(ipRelations, cam, optsMWO);
        
        % re-assign 6 DoF camera motion
        tx = xi_21(1);
        ty = xi_21(2);
        tz = xi_21(3);
        phi = xi_21(4);
        theta = xi_21(5);
        psi = xi_21(6);
        
        
    end
    
    
    %% 3. update 6 DoF camera pose and visualization
    
    if (imgIdx >= 2)
        % update current camera pose
        t_21 = [tx; ty; tz];
        R_21 = angle2rotmtx([phi; theta; psi]);
        T_21 = [R_21, t_21;
            0, 0, 0, 1];
        
        T_gc_current = T_gc_current * inv(T_21);
        T_gc_DEMO{imgIdx} = T_gc_current;
        
        
        % visualize current status
        plots_ICSL_RGBD;
    end
    
    
end

% convert camera pose representation
stateEsti_DEMO = zeros(6, M);
R_gc_DEMO = zeros(3,3,M);
for k = 1:M
    R_gc_DEMO(:,:,k) = T_gc_DEMO{k}(1:3,1:3);
    stateEsti_DEMO(1:3,k) = T_gc_DEMO{k}(1:3,4);
    [yaw, pitch, roll] = dcm2angle(R_gc_DEMO(:,:,k));
    stateEsti_DEMO(4:6,k) = [roll; pitch; yaw];
end


%% plot error metric value (RPE, ATE)


% 1) DEMO motion estimation trajectory results
figure;
plot3(stateEsti_DEMO(1,:),stateEsti_DEMO(2,:),stateEsti_DEMO(3,:),'r','LineWidth',2); grid on;
legend('DEMO Matlab'); plot_inertial_frame(0.5); axis equal; view(-158, 38);
xlabel('x [m]','fontsize',10); ylabel('y [m]','fontsize',10); zlabel('z [m]','fontsize',10); hold off;


% 2) DEMO motion estimation trajectory results
figure;
subplot(3,2,1);
plot(stateEsti_DEMO(1,:),'r','LineWidth',2); grid on; axis tight; ylabel('x (m)');
legend('DEMO Matlab');
subplot(3,2,3);
plot(stateEsti_DEMO(2,:),'r','LineWidth',2); grid on; axis tight; ylabel('y (m)');
subplot(3,2,5);
plot(stateEsti_DEMO(3,:),'r','LineWidth',2); grid on; axis tight; ylabel('z (m)');
subplot(3,2,2);
plot(stateEsti_DEMO(4,:),'r','LineWidth',2); grid on; axis tight; ylabel('roll (rad)');
subplot(3,2,4);
plot(stateEsti_DEMO(5,:),'r','LineWidth',2); grid on; axis tight; ylabel('pitch (rad)');
subplot(3,2,6);
plot(stateEsti_DEMO(6,:),'r','LineWidth',2); grid on; axis tight; ylabel('yaw (rad)');


%% save the experiment data for ICRA 2018

if (toSave)
    save([SaveDir '/DEMO.mat']);
end


