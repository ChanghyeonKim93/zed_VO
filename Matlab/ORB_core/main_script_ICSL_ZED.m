clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;


%% basic setup for ORB SLAM

% choose the experiment case
% ICSL RGBD dataset (1~XX)
expCase = 1;

% are data results saved?
% 1 : yes, save the variables and results
% 0 : no, just run ORB
toSave = 1;


setupParams_ICSL_ZED;


%% load ORB SLAM estimation result


% import ORB SLAM result
SaveDir = [datasetPath '/forSamsung'];
textData_ORB = importdata([SaveDir '/ORB/CameraTrajectory.txt'], ' ');
textData_ORB = textData_ORB(imInit:(imInit+M-1),:);
rawORBSLAMtextdata.p_gc = [textData_ORB(:,2).'; textData_ORB(:,3).'; textData_ORB(:,4).'];
rawORBSLAMtextdata.q_gc = [textData_ORB(:,8).'; textData_ORB(:,5).'; textData_ORB(:,6).'; textData_ORB(:,7).'];


% transform from ORB inertial frame to true inertial frame
R_gORBc1 = q2r(rawORBSLAMtextdata.q_gc(:,1));
p_gORBc1 = rawORBSLAMtextdata.p_gc(:,1);
T_gORBc1 = [R_gORBc1, p_gORBc1;
    zeros(1,3), 1];
T_gc_true{1} = eye(4);
T_g_gORB = T_gc_true{1} * inv(T_gORBc1);


% extract ORB SLAM pose w.r.t. true inertial frame
T_gc_ORB = cell(1,M);
for k = 1:M
    % camera body frame in ORB
    R_gc_ORB_temp = q2r(rawORBSLAMtextdata.q_gc(:,k));
    p_gc_ORB_temp = rawORBSLAMtextdata.p_gc(:,k);
    T_gc_ORB_temp = [R_gc_ORB_temp, p_gc_ORB_temp;
        zeros(1,3), 1];
    
    % camera body frame in ground truth
    T_gc_ORB{k} = T_g_gORB * T_gc_ORB_temp;
end


% convert camera pose representation
stateEsti_ORB = zeros(6,M);
R_gc_ORB = zeros(3,3,M);
for k = 1:M
    R_gc_ORB(:,:,k) = T_gc_ORB{k}(1:3,1:3);
    stateEsti_ORB(1:3,k) = T_gc_ORB{k}(1:3,4);
    [yaw, pitch, roll] = dcm2angle(R_gc_ORB(:,:,k));
    stateEsti_ORB(4:6,k) = [roll; pitch; yaw];
end


%% plot error metric value (RPE, ATE)


% 1) ORB motion estimation trajectory results
figure;
plot3(stateEsti_ORB(1,:),stateEsti_ORB(2,:),stateEsti_ORB(3,:),'r','LineWidth',2); grid on;
legend('ORB Matlab'); plot_inertial_frame(0.5); axis equal; view(-158, 38);
xlabel('x [m]','fontsize',10); ylabel('y [m]','fontsize',10); zlabel('z [m]','fontsize',10); hold off;


% 2) ORB motion estimation trajectory results
figure;
subplot(3,2,1);
plot(stateEsti_ORB(1,:),'r','LineWidth',2); grid on; axis tight; ylabel('x (m)');
legend('ORB Matlab');
subplot(3,2,3);
plot(stateEsti_ORB(2,:),'r','LineWidth',2); grid on; axis tight; ylabel('y (m)');
subplot(3,2,5);
plot(stateEsti_ORB(3,:),'r','LineWidth',2); grid on; axis tight; ylabel('z (m)');
subplot(3,2,2);
plot(stateEsti_ORB(4,:),'r','LineWidth',2); grid on; axis tight; ylabel('roll (rad)');
subplot(3,2,4);
plot(stateEsti_ORB(5,:),'r','LineWidth',2); grid on; axis tight; ylabel('pitch (rad)');
subplot(3,2,6);
plot(stateEsti_ORB(6,:),'r','LineWidth',2); grid on; axis tight; ylabel('yaw (rad)');


% 3) final drift error of ORB
EPE_ORB = norm(stateEsti_ORB(1:3,1) - stateEsti_ORB(1:3,end));


%% save the experiment data for ICRA 2018

if (toSave)
    save([SaveDir '/ORB.mat']);
end



