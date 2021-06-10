%ARS4 mini-project
clear all;
close all;
clc;

% Calibration data
x_std_f = 1.0550132608651832;
y_std_f = 0.8851236902556164;
yaw_std_f = 0.0398325634250031;

yaw_rate_offset_f = 0.003264;
speed_scale_f = 1.0124;
lever_arm_f = 0.4847;

speed_std_f = 0.10344498814394902;
yaw_rate_std_f = 0.04471455284474848;

x_std_l = 1.0156331264014704;
y_std_l = 0.9009600934652383;
yaw_std_l = 0.03621906547404191;

yaw_rate_offset_l = 0.003325;
speed_scale_l = 1.0157;
lever_arm_l = 0.5284;

speed_std_l = 0.10585673297965029;
yaw_rate_std_l = 0.04346252687730287;

% Datasets
data_lidar_f = readtable('../csv_sync/follower_plicp.csv');
data_gnss_f = readtable('../csv_sync/sync_follower_gnss.csv');
data_kinetics_f = readtable('../csv_sync/sync_follower_kinetics.csv');
data_gnss_ref_f = readtable('../csv_sync/sync_follower_gnss_ref.csv');

data_gnss_l = readtable('../csv_sync/sync_leader_gnss.csv');
data_kinetics_l = readtable('../csv_sync/sync_leader_kinetics.csv');
data_gnss_ref_l = readtable('../csv_sync/sync_leader_gnss_ref.csv');

% Map
data_map = readtable('../csv_sync/map_seville.csv');

zoom = 20;
sleeping_time = 0.1;
nb_steps=size(data_lidar_f,1);
skip_steps=5;

for step = 1:skip_steps:nb_steps
    % Pose LiDAR
    pos_lidar_f = [data_lidar_f.x(step); data_lidar_f.y(step); data_lidar_f.yaw(step)];
    cov_lidar_f = [data_lidar_f.cov_x(step)        data_lidar_f.cov_xy(step)     data_lidar_f.cov_xyaw(step); 
                          data_lidar_f.cov_xy(step)      data_lidar_f.cov_y(step)       data_lidar_f.cov_yyaw(step);
                          data_lidar_f.cov_xyaw(step)  data_lidar_f.cov_yyaw(step) data_lidar_f.cov_yaw(step)];
    
    % Pose GNSS for the follower                
    pos_gnss_f = [data_gnss_f.x(step)+data_gnss_f.bias_x(step); 
                            data_gnss_f.y(step)+data_gnss_f.bias_y(step);
                            data_gnss_f.yaw(step)];
%     cov_gnss_f = [(data_gnss_f.h_acc(step)/1000.0)^2  0.0  0.0; 
%                             0.0  (data_gnss_f.h_acc(step)/1000.0)^2  0.0;
%                             0.0  0.0  (data_gnss_f.yaw_acc(step))^2];
    cov_gnss_f = [x_std_f^2  0.0  0.0; 
                            0.0  y_std_f^2  0.0;
                            0.0  0.0  yaw_std_f^2];
    
    % Speed and yaw rate for the follower
    yaw_rate =  data_kinetics_f.yaw_rate(step) + yaw_rate_offset_f;
    speed = speed_scale_f*data_kinetics_f.lon_vel(step) - lever_arm_f*abs(yaw_rate);
    vel_f = [speed; yaw_rate];
    cov_vel_f = [speed_std_f^2  0.0;
                        0.0 yaw_rate_std_f^2];
                        
    % Ground truth for the follower      
    pos_ref_f = [data_gnss_ref_f.x(step); data_gnss_ref_f.y(step); data_gnss_ref_f.yaw(step)];
    cov_ref_f = [data_gnss_ref_f.x_std(step)^2  0.0 0.0; 
                        0.0  data_gnss_ref_f.y_std(step)^2  0.0;
                        0.0  0.0  data_gnss_ref_f.yaw_std(step)^2];
    
    % Pose GNSS for the leader                
    pos_gnss_l = [data_gnss_l.x(step)+data_gnss_l.bias_x(step); 
                           data_gnss_l.y(step)+data_gnss_l.bias_y(step); 
                           data_gnss_l.yaw(step)];
%     cov_gnss_l = [(data_gnss_l.h_acc(step)/1000.0)^2  0.0  0.0; 
%                             0.0  (data_gnss_l.h_acc(step)/1000.0)^2  0.0;
%                             0.0  0.0  (data_gnss_l.yaw_acc(step))^2];
    cov_gnss_l = [x_std_l^2  0.0  0.0; 
                            0.0  y_std_l^2  0.0;
                            0.0  0.0  yaw_std_l^2];
    
    % Speed and yaw rate for the leader
    yaw_rate =  data_kinetics_l.yaw_rate(step) + yaw_rate_offset_l;
    speed = speed_scale_l*data_kinetics_l.lon_vel(step) - lever_arm_l*abs(yaw_rate);
    vel_l = [speed; yaw_rate];
    cov_vel_l = [speed_std_l^2  0.0;
                        0.0 yaw_rate_std_l^2];
                        
    % Ground truth for the leader      
    pos_ref_l = [data_gnss_ref_l.x(step); data_gnss_ref_l.y(step); data_gnss_ref_l.yaw(step)];
    cov_ref_l = [data_gnss_ref_l.x_std(step)^2  0.0 0.0; 
                        0.0  data_gnss_ref_l.y_std(step)^2  0.0;
                        0.0  0.0  data_gnss_ref_l.yaw_std(step)^2];
                         
    figure(1); clf;
    % Map display
    plot(data_map.x, data_map.y, '-k'); hold on;
    % Display of the ground truth of the leader Vehicle
    displayPos(pos_ref_l, vel_l(1), 'k');
    displayCov(pos_ref_l, cov_ref_l, 0.95, 'k');
%     % Display of the Ublox receiver computed position of the leader Vehicle
%     displayPos(pos_gnss_l, 1, 'b');
%     displayCov(pos_gnss_l, cov_gnss_l, 0.95, 'b');
    % Display of the ground truth of the follower Vehicle
    displayPos(pos_ref_f, vel_f(1), 'k');
    displayCov(pos_ref_f, cov_ref_f, 0.95, 'k');
%     % Display of the Ublox receiver computed position of the follower Vehicle
%     displayPos(pos_gnss_f, 1, 'b');
%     displayCov(pos_gnss_f, cov_gnss_f, 0.95, 'b');
    
    if zoom ~= 0 %zoom in view
        xlim([pos_ref_f(1)-zoom pos_ref_f(1)+zoom]);
        ylim([pos_ref_f(2)-zoom pos_ref_f(2)+zoom]);
    end
    xlabel('m');ylabel('m');
    title(['The two cars on SEVILLE (',num2str(round(100*step/nb_steps)),'% of the test)']);
    
    if data_lidar_f.usable(step)
        color = 'b';
    else
        color = 'r';
    end
%     figure(2); clf;
%     plot(0, 0, '> k'); hold on;
%     displayPos(pos_lidar_f, 1, color);
%     if(data_lidar_f.plicp_det(step) > 10^(-5))
%         displayCov(pos_lidar_f, cov_lidar_f, 0.95, color);
%     end
%     xlim([0 20]);
%     ylim([-5 5]);
%     grid on;
%     xlabel('m');
%     ylabel('m');
%     title('Relative pose of the leader in the frame of the follower');
    
    pause(sleeping_time);
end
