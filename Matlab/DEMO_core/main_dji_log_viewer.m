clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;


%% basic setup

% choose the experiment case
% ICSL RGBD dataset (1~XX)
expCase = 1;
setupParams_ICSL_ZED;


% load data
logFile = [datasetPath '/dji_topics.txt'];
logData = importdata(logFile,'\t');


% data IDs
DJI_LOCAL_POSITION = 0;
DJI_GLOBAL_POSITION = 10;
DJI_ODOMETRY = 20;
DJI_VELOCITY = 30;
DJI_ACCELERATION = 40;
DJI_ATTITUDE = 50;
ICSL_VO = 100;


% raw data
dji_local_pos = logData( logData(:,2) == DJI_LOCAL_POSITION, [1,(1:3)+2]);
dji_global_pos = logData( logData(:,2) == DJI_GLOBAL_POSITION, [1,(1:5)+2]);
dji_odom = logData( logData(:,2) == DJI_ODOMETRY, [1,(1:12)+2]);
dji_vel = logData( logData(:,2) == DJI_VELOCITY, [1,(1:4)+2]);
dji_acc = logData( logData(:,2) == DJI_ACCELERATION, [1,(1:3)+2]);
dji_att = logData( logData(:,2) == DJI_ATTITUDE, [1,(1:6)+2]);
icsl_vo = logData( logData(:,2) == ICSL_VO, [1,(1:4)+2]);

euler_name = {'\phi [deg]', '\theta [deg]', '\psi [deg]'};
euler_rate_name = {'d\phi [deg/s]', 'd\theta [deg/s]', 'd\psi [deg/s]'};
rot_vel_name = {'\omega_x [deg/s]', '\omega_y [deg/s]', '\omega_z [deg/s]'};
pos_name = {'X [m]', 'Y [m]', 'Z [m]'};
pos_rate_name = {'dX [m/s]', 'dY [m/s]', 'dZ [m/s]'};
acc_name = {'a_x [m/s^2]', 'a_y [m/s^2]', 'a_z [m/s^2]'};
gps_name = {'lat [deg]', 'lon [deg]', 'alt [m]', 'status'};
lin_vel_name = {'v_x [m/s]', 'v_y [m/s]', 'v_z [m/s]'};


%%
%%%%% 3D view
figure(123)
cla; hold on; grid on;
plot3(icsl_vo(:,2), icsl_vo(:,3), icsl_vo(:,4), 'b-')
plot3(dji_local_pos(:,2), dji_local_pos(:,3), dji_local_pos(:,4), 'gx--')
plot3(dji_local_pos(1,2), dji_local_pos(1,3), dji_local_pos(1,4),...
    'bo', 'markerfacecolor', 'b', 'markersize', 10)
plot3(dji_local_pos(end,2), dji_local_pos(end,3), dji_local_pos(end,4),...
    'mo', 'markerfacecolor', 'm', 'markersize', 10)
axis equal
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
view([60,21])
axis tight
title('3D view')


%% for expCase = 1

% GPS re-package & origin shift
p_GPS = [dji_local_pos(:,2).'; dji_local_pos(:,3).'; dji_local_pos(:,4).'];
p_GPS = p_GPS - (p_GPS(:,1) * ones(1,size(p_GPS,2)));

% transform to new inertial frame for good view angle
xyzRotAngle = [0.0*(pi/180), 0.0*(pi/180), -75.0*(pi/180)];
R_gnew_g = angle2rotmtx(xyzRotAngle).';
p_GPS_forview = R_gnew_g * p_GPS(:,1:3000);


% VO re-package & origin shift
p_vo = stateEsti_DEMO(1:3,:);
p_vo = p_vo - (p_vo(:,1) * ones(1,size(p_vo,2)));

% transform to new inertial frame for good view angle
xyzRotAngle = [-155.0*(pi/180), -40.0*(pi/180), 22.0*(pi/180)];
R_gnew_g = angle2rotmtx(xyzRotAngle).';
p_vo_forview = R_gnew_g * p_vo(:,1:1500);


% plot
figure(123)
h_GPS = plot3(p_GPS_forview(1,:), p_GPS_forview(2,:), p_GPS_forview(3,:), 'k', 'Linewidth', 3); hold on; grid on; axis equal;
plot3(p_GPS_forview(1,1), p_GPS_forview(2,1), p_GPS_forview(3,1),'bo', 'markerfacecolor', 'b', 'markersize', 10);
plot3(p_GPS_forview(1,end), p_GPS_forview(2,end), p_GPS_forview(3,end),'ro', 'markerfacecolor', 'r', 'markersize', 10);
h_VO = plot3(p_vo_forview(1,:), p_vo_forview(2,:), p_vo_forview(3,:), 'm', 'Linewidth', 3);
h_start = plot3(p_vo_forview(1,1), p_vo_forview(2,1), p_vo_forview(3,1),'bo', 'markerfacecolor', 'b', 'markersize', 10);
h_end = plot3(p_vo_forview(1,end), p_vo_forview(2,end), p_vo_forview(3,end),'ro', 'markerfacecolor', 'r', 'markersize', 10);
set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',16);
title('top view','FontSize',15,'FontName','Times New Roman');
xlabel('X [m]','FontSize',15,'FontName','Times New Roman');
ylabel('Y [m]','FontSize',15,'FontName','Times New Roman');
zlabel('Z [m]','FontSize',15,'FontName','Times New Roman');
axis tight; view([0,90]);
legend([h_GPS h_VO h_start h_end],{'GPS','VO','Start','End'},'Orientation','vertical','FontSize',13,'FontName','Times New Roman');


%% for expCase = 2

% GPS re-package & origin shift
p_GPS = [dji_local_pos(:,2).'; dji_local_pos(:,3).'; dji_local_pos(:,4).'];
p_GPS = p_GPS - (p_GPS(:,1) * ones(1,size(p_GPS,2)));

% transform to new inertial frame for good view angle
xyzRotAngle = [0.0*(pi/180), 180.0*(pi/180), -16.0*(pi/180)];
R_gnew_g = angle2rotmtx(xyzRotAngle).';
p_GPS_forview = R_gnew_g * p_GPS;


% VO re-package & origin shift
p_vo = stateEsti_DEMO(1:3,:);
p_vo = p_vo - (p_vo(:,1) * ones(1,size(p_vo,2)));

% transform to new inertial frame for good view angle
xyzRotAngle = [-145.0*(pi/180), 10.0*(pi/180), 5.0*(pi/180)];
R_gnew_g = angle2rotmtx(xyzRotAngle).';
p_vo_forview = R_gnew_g * p_vo;


% plot
figure(123)
h_GPS = plot3(p_GPS_forview(1,:), p_GPS_forview(2,:), p_GPS_forview(3,:), 'k', 'Linewidth', 3); hold on; grid on; axis equal;
plot3(p_GPS_forview(1,1), p_GPS_forview(2,1), p_GPS_forview(3,1),'bo', 'markerfacecolor', 'b', 'markersize', 10);
plot3(p_GPS_forview(1,end), p_GPS_forview(2,end), p_GPS_forview(3,end),'ro', 'markerfacecolor', 'r', 'markersize', 10);
h_VO = plot3(p_vo_forview(1,:), p_vo_forview(2,:), p_vo_forview(3,:), 'm', 'Linewidth', 3);
h_start = plot3(p_vo_forview(1,1), p_vo_forview(2,1), p_vo_forview(3,1),'bo', 'markerfacecolor', 'b', 'markersize', 10);
h_end = plot3(p_vo_forview(1,end), p_vo_forview(2,end), p_vo_forview(3,end),'ro', 'markerfacecolor', 'r', 'markersize', 10);
set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',16);
title('top view','FontSize',15,'FontName','Times New Roman');
xlabel('X [m]','FontSize',15,'FontName','Times New Roman');
ylabel('Y [m]','FontSize',15,'FontName','Times New Roman');
zlabel('Z [m]','FontSize',15,'FontName','Times New Roman');
axis tight; view([0,90]);
legend([h_GPS h_VO h_start h_end],{'GPS','VO','Start','End'},'Orientation','vertical','FontSize',13,'FontName','Times New Roman');


%%
%%%%% top-down view
[dji_gps_y, dji_gps_x] = deg2utm(dji_global_pos(:,2), dji_global_pos(:,3));

figure(124)
cla; hold on; grid on;
plot(dji_local_pos(:,2) - dji_local_pos(1,2), dji_local_pos(:,3) - dji_local_pos(1,3), 'gx--')
% plot(dji_gps_x - dji_gps_x(1), dji_gps_y - dji_gps_y(1), 'b')
axis equal
axis tight
xlabel('X [m]')
ylabel('Y [m]')
title('top-down view')

%%
%%%%% local position
figure(111)
for i = 1:3
    subplot(3,1,i)
    cla; hold on; grid on;
    plot(dji_local_pos(:,1), dji_local_pos(:,i+1), 'gx--')
    plot(dji_odom(:,1), dji_odom(:,i+1), 'b')
    axis tight
    ylabel( pos_name{i} )
    if i == 1
        title('local position')
    end
    if i == 3
        xlabel('time [s]')
    end
end

%%
%%%%% euler angle
figure(112)
for i = 1:3
    subplot(3,1,i)
    cla; hold on; grid on;
    plot(dji_att(:,1), (180/pi)*dji_att(:,i+1), 'gx--')
    plot(dji_odom(:,1), (180/pi)*dji_odom(:,i+4), 'b')
    axis tight %dji_log_1
    ylabel( euler_name{i} )
    if i == 1
        title('euler angles')
    end
    if i == 3
        xlabel('time [s]')
    end
end

%%
%%%%% local position rate
figure(113)
for i = 1:3
    subplot(3,1,i)
    cla; hold on; grid on;
    plot(dji_vel(:,1), dji_vel(:,i+1), 'gx--')
    plot(dji_odom(:,1), dji_odom(:,i+7), 'b')
    axis tight
    ylabel( pos_rate_name{i} )
    if i == 1
        title('position rate')
    end
    if i == 3
        xlabel('time [s]')
    end
end

%%
%%%%% global position
figure(116)
for i = 1:4
    subplot(4,1,i)
    cla; hold on; grid on;
    if i < 4
        plot(dji_global_pos(:,1), dji_global_pos(:,i+1), 'gx--')
    else
        plot(dji_global_pos(:,1), dji_global_pos(:,i+2), 'gx--')
    end
    axis tight
    ylabel( gps_name{i} )
    if i == 1
        title('GPS')
    end
    if i == 4
        xlabel('time [s]')
    end
end

%%
%%%%% GPS parameter
figure(117)
subplot(3,1,1);
plot(dji_global_pos(1:3400,1), dji_global_pos(1:3400,2), 'k', 'Linewidth', 1.5); grid on; axis tight;
set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',14);
ylabel('Latitude [deg]','FontSize',14,'FontName','Times New Roman');

subplot(3,1,2);
plot(dji_global_pos(1:3400,1), dji_global_pos(1:3400,3), 'k', 'Linewidth', 1.5); grid on; axis tight;
set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',14);
ylabel('Longitude [deg]','FontSize',14,'FontName','Times New Roman');

subplot(3,1,3);
plot(dji_global_pos(1:3400,1), dji_global_pos(1:3400,6), 'k', 'Linewidth', 1.5); grid on; axis tight;
set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',14);
xlabel('Time [s]','FontSize',14,'FontName','Times New Roman');
ylabel('GPS status','FontSize',14,'FontName','Times New Roman');



