clear
close all
clc

%----------READ DATA-------------------------------------------------------
% calibration data
load calibration_data.mat;
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
%--------------------------------------------------------------------------

%---------Filter parameter initialization----------------------------------
% Evolution model (Take v and w as input)
syms v w psi Te x y f(x,y,psi,v,w,Te) A(x,y,psi,v,w,Te)
f(x,y,psi,v,w,Te) = [x+v*Te*cos(psi);
                     y+v*Te*sin(psi);
                     psi+w*Te;
                     v;
                     w];
A(x,y,psi,v,w,Te) = jacobian(f, [x,y,psi,v,w]);

% Initial states
X_f = [data_gnss_ref_f.x(1); 
    data_gnss_ref_f.y(1); 
    data_gnss_ref_f.yaw(1);
    0;
    0];                         %X=[x,y,psi,v,w]
X_l = [data_gnss_ref_l.x(1); 
    data_gnss_ref_l.y(1); 
    data_gnss_ref_l.yaw(1);
    0;
    0];
% Model error
Q_f = zeros(5);
Q_f(1,1) = 1/2;Q_f(2,2) = 1/2;Q_f(3,3) = .1;
Q_f(4,4) = speed_std_f^2;Q_f(5,5) = yaw_rate_std_f^2;
Q_f = Q_f*0.1;
Q_l = zeros(5);
Q_l(1,1) = 1/2;Q_l(2,2) = 1;Q_l(3,3) = .1;
Q_l(4,4) = speed_std_l^2;Q_l(5,5) = yaw_rate_std_l^2;
Q_l = Q_l*0.1;
% Observation error
R_f = eye(5);
R_f(1,1) = 1.2*x_std_f^2; R_f(2,2) = y_std_f^2; R_f(3,3) = yaw_std_f^2;
R_f(4,4) = speed_std_f^2; R_f(5,5) = yaw_rate_std_f^2;
R_f = R_f*4;
R_l = eye(5);
R_l(1,1) = 1.15*x_std_l^2; R_l(2,2) = y_std_l^2; R_l(3,3) = yaw_std_l^2;
R_l(4,4) = speed_std_l^2; R_l(5,5) = yaw_rate_std_l^2;
R_l = R_l*4;
% Initial covariance matrix
P_f = 10*eye(5);
P_f(3,3) = 0.1;P_f(4,4) = 1;P_f(5,5) = 0.1;
P_l = 10*eye(5);
P_l(3,3) = 0.1;P_l(4,4) = 1;P_l(5,5) = 0.1;


%--------------------------------------------------------------------------

%----------Simulation Setting----------------------------------------------
sleeping_time = 0.001;
nb_steps=round(size(data_lidar_f,1)/5);
skip_steps=5;
steps = 1:skip_steps:nb_steps;
zoom = 20;
T0 = data_gnss_f.time(1);
T = data_gnss_f.time(1)-skip_steps*0.08;
%--------------------------------------------------------------------------

%----------Result Saving---------------------------------------------------
nepoch = length(steps);
result_f = struct();
result_f.timestamp = zeros(1,nepoch);
result_f.uselidar = zeros(1,nepoch);
result_f.X = zeros(5,nepoch);
result_f.P = zeros(5,nepoch);
result_f.err = zeros(3,nepoch);
result_l = struct();
result_l.timestamp = zeros(1,nepoch);
result_l.uselidar = zeros(1,nepoch);
result_l.X = zeros(5,nepoch);
result_l.P = zeros(5,nepoch);
result_l.err = zeros(3,nepoch);
non_consis_f = 0;
non_consis_l = 0;
%--------------------------------------------------------------------------

for epoch = 1:nepoch
    step = steps(epoch);
    dt = data_gnss_f.time(step)-T;
    T = data_gnss_f.time(step);
    %----------------------------------------------------------------------
    % get measure for the follower    
    pos_gnss_f = [data_gnss_f.x(step)+data_gnss_f.bias_x(step); 
                  data_gnss_f.y(step)+data_gnss_f.bias_y(step);
                  data_gnss_f.yaw(step)];
    yaw_rate =  data_kinetics_f.yaw_rate(step) + yaw_rate_offset_f;
    speed = speed_scale_f*data_kinetics_f.lon_vel(step) - lever_arm_f*abs(yaw_rate);
    vel_f = [speed; yaw_rate];
   
    % get measure for the leader
    pos_gnss_l = [data_gnss_l.x(step)+data_gnss_l.bias_x(step); 
                   data_gnss_l.y(step)+data_gnss_l.bias_y(step); 
                   data_gnss_l.yaw(step)];
    yaw_rate =  data_kinetics_l.yaw_rate(step) + yaw_rate_offset_l;
    speed = speed_scale_l*data_kinetics_l.lon_vel(step) - lever_arm_l*abs(yaw_rate);
    vel_l = [speed; yaw_rate];    
    %----------------------------------------------------------------------
    % Measurement LiDAR
    pos_lidar_f = [data_lidar_f.x(step); data_lidar_f.y(step); data_lidar_f.yaw(step)];
    cov_lidar_f = [data_lidar_f.cov_x(step)        data_lidar_f.cov_xy(step)     data_lidar_f.cov_xyaw(step); 
                  data_lidar_f.cov_xy(step)      data_lidar_f.cov_y(step)       data_lidar_f.cov_yyaw(step);
                  data_lidar_f.cov_xyaw(step)  data_lidar_f.cov_yyaw(step) data_lidar_f.cov_yaw(step)];       
    %----------------------------------------------------------------------
    % Ground truth for the follower      
    pos_ref_f = [data_gnss_ref_f.x(step); data_gnss_ref_f.y(step); data_gnss_ref_f.yaw(step)];
    cov_ref_f = [data_gnss_ref_f.x_std(step)^2  0.0 0.0; 
                0.0  data_gnss_ref_f.y_std(step)^2  0.0;
                0.0  0.0  data_gnss_ref_f.yaw_std(step)^2];
    % Ground truth for the leader      
    pos_ref_l = [data_gnss_ref_l.x(step); data_gnss_ref_l.y(step); data_gnss_ref_l.yaw(step)];
    cov_ref_l = [data_gnss_ref_l.x_std(step)^2  0.0 0.0; 
                0.0  data_gnss_ref_l.y_std(step)^2  0.0;
                0.0  0.0  data_gnss_ref_l.yaw_std(step)^2]; 
    %----------------------------------------------------------------------
    % EKF Update
    % Follower
    y_f = [pos_gnss_f; vel_f];
    C_f = eye(5);
    [X_f, P_f] = KF_update(X_f, P_f, y_f, R_f, C_f);
    % Leader
    y_l = [pos_gnss_l; vel_l];
    C_l = eye(5);
    [X_l, P_l] = KF_update(X_l, P_l, y_l, R_l, C_l);
    
    % Communication Leader->Follower
    pos_l_in_ref_com = X_l(1:3);
    cov_l_in_ref_com = P_l(1:3,1:3);
    
    % Communication Follower->Leader
    pos_f_in_ref_com = X_f(1:3);
    cov_f_in_ref_com = P_f(1:3,1:3);
    
    % Follower lidar update
    use_lidar_f = data_lidar_f.usable(step);
    use_lidar_l = data_lidar_f.usable(step);
    % Frame transformation
    if use_lidar_f
        [pos_f_in_l, cov_f_in_l] = ominus_UT(pos_lidar_f, cov_lidar_f);
        [pos_f_in_ref, cov_f_in_ref] = oplus_UT(pos_l_in_ref_com, pos_f_in_l, cov_l_in_ref_com, cov_f_in_l);
        y_f_lidar = pos_f_in_ref;
        % Outlier Rejection
        C_f_lidar = eye(3);
        R_f_lidar = cov_f_in_ref*1;R_f_lidar(2,2) = R_f_lidar(2,2)*1.2;
        D = sqrt((y_f_lidar-C_f_lidar*X_f(1:3))'/(C_f_lidar*P_f(1:3,1:3)*C_f_lidar'+R_f_lidar)*(y_f_lidar-C_f_lidar*X_f(1:3)));
        if D > chi2inv(0.95,3)
            use_lidar_f = 0;
        end
    end
    if use_lidar_l
        [pos_l_in_ref, cov_l_in_ref] = oplus_UT(pos_f_in_ref_com, pos_lidar_f, cov_f_in_ref_com, cov_lidar_f);
        y_l_lidar = pos_l_in_ref;
        % Outlier Rejection
        C_l_lidar = eye(3);
        R_l_lidar = cov_l_in_ref*1;
        D = sqrt((y_l_lidar-C_l_lidar*X_l(1:3))'/(C_l_lidar*P_l(1:3,1:3)*C_l_lidar'+R_l_lidar)*(y_l_lidar-C_l_lidar*X_l(1:3)));
        if D > chi2inv(0.95,3)
            use_lidar_l = 0;
        end
    end
    if use_lidar_f
        result_f.uselidar(epoch) = 1;
        [X_f(1:3), P_f(1:3,1:3)] = CI_update(X_f(1:3), P_f(1:3,1:3), y_f_lidar, R_f_lidar, C_f_lidar);
    end
    if use_lidar_l
        result_l.uselidar(epoch) = 1;
        [X_l(1:3), P_l(1:3,1:3)] = CI_update(X_l(1:3), P_l(1:3,1:3), y_l_lidar, R_l_lidar, C_l_lidar);
    end
    %----------------------------------------------------------------------
    % Save result
    % Follower
    result_f.X(:,epoch) = X_f;
    result_f.P(:,epoch) = diag(P_f);
    result_f.timestamp(epoch) = T - T0;
    result_f.err(:,epoch) = X_f(1:3)-pos_ref_f;
    if result_f.err(3,epoch)<-pi || result_f.err(3,epoch)>pi
        result_f.err(3,epoch) = result_f.err(3,epoch) - sign(result_f.err(3,epoch))*2*pi;
    end
    % Leader
    result_l.X(:,epoch) = X_l;
    result_l.P(:,epoch) = diag(P_l);
    result_l.timestamp(epoch) = T - T0;
    result_l.err(:,epoch) = X_l(1:3)-pos_ref_l;
    if result_l.err(3,epoch)<-pi || result_l.err(3,epoch)>pi
        result_l.err(3,epoch) = result_l.err(3,epoch) - sign(result_l.err(3,epoch))*2*pi;
    end
    %----------------------------------------------------------------------
    % Consistency check
    thresh = chi2inv(0.95,3);
    d_f = result_f.err(:,epoch)'/(P_f(1:3,1:3)+cov_ref_f)*result_f.err(:,epoch);
    d_l = result_l.err(:,epoch)'/(P_l(1:3,1:3)+cov_ref_l)*result_l.err(:,epoch);
    if d_f > thresh
        non_consis_f = non_consis_f+1;
    end
    if d_l > thresh
        non_consis_l = non_consis_l+1;
    end
    %----------------------------------------------------------------------
    % Plot result
    figure(1); clf;
    % Map display
    plot(data_map.x, data_map.y, '-k'); hold on;
            
    % Display of the ground truth of the leader Vehicle
    displayPos(pos_ref_l, vel_l(1), 'k');
    displayCov(pos_ref_l, cov_ref_l, 0.95, 'k');
    % Display of the estimated of the leader Vehicle
    if use_lidar_f
        displayPos(X_l, 1, 'b');
        displayCov(X_l, P_l, 0.95, 'b');
    else
        displayPos(X_l, 1, 'r');
        displayCov(X_l, P_l, 0.95, 'r');
    end
    % Display of the ground truth of the follower Vehicle
    displayPos(pos_ref_f, vel_f(1), 'k');
    displayCov(pos_ref_f, cov_ref_f, 0.95, 'k');
    % Display of the estimated of the follower Vehicle
    if use_lidar_f
        displayPos(X_f, 1, 'b');
        displayCov(X_f, P_f, 0.95, 'b');
    else
        displayPos(X_f, 1, 'r');
        displayCov(X_f, P_f, 0.95, 'r');
    end
    if zoom ~= 0 %zoom in view
        xlim([pos_ref_f(1)-zoom pos_ref_f(1)+zoom]);
        ylim([pos_ref_f(2)-zoom pos_ref_f(2)+zoom]);
    end
    xlabel('m');ylabel('m');
    title(['The two cars on SEVILLE (',num2str(round(100*step/nb_steps)),'% of the test)']);
    pause(sleeping_time)
    %----------------------------------------------------------------------
    % Prediction
    % Follower
    Ak_f = double(A(X_f(1),X_f(2),X_f(3),X_f(4),X_f(5),dt));
    X_f = double(f(X_f(1),X_f(2),X_f(3),X_f(4),X_f(5),dt));
    P_f = Ak_f*P_f*Ak_f'+Q_f;
    if X_f(3)>pi || X_f(3)<-pi
        X_f(3) = X_f(3)- sign(X_f(3))*2*pi;
    end
    % Leader
    Ak_l = double(A(X_l(1),X_l(2),X_l(3),X_l(4),X_l(5),dt));
    X_l = double(f(X_l(1),X_l(2),X_l(3),X_l(4),X_l(5),dt));
    P_l = Ak_l*P_l*Ak_l'+Q_l;
    if X_l(3)>pi || X_l(3)<-pi
        X_l(3) = X_l(3)- sign(X_l(3))*2*pi;
    end
    %----------------------------------------------------------------------
    clc; disp([num2str(round(100*epoch/nepoch,0)),'% Completed']);
end
%--------------------------------------------------------------------------
% Compute Accuracy using MSE
Acc_f_x = mean(result_f.err(1,:).^2);Acc_f_y = mean(result_f.err(2,:).^2);
Acc_f_yaw = mean(result_f.err(3,:).^2);
Acc_l_x = mean(result_l.err(1,:).^2);Acc_l_y = mean(result_l.err(2,:).^2);
Acc_l_yaw = mean(result_l.err(3,:).^2);
% Compute consistency
consis_f = 1- non_consis_f/nepoch;
consis_l = 1- non_consis_l/nepoch;
% Display accuracy and consistency
disp(['Accuracy of the estimate on the follower : x-', num2str(Acc_f_x),' | y-', num2str(Acc_f_y),' | yaw-', num2str(Acc_f_yaw)])
disp(['Consistency of the estimate on the follower: ', num2str(100*consis_f),'%'])
disp(['Accuracy of the estimate on the leader : x-', num2str(Acc_l_x),' | y-', num2str(Acc_l_y),' | yaw-', num2str(Acc_l_yaw)])
disp(['Consistency of the estimate on the leader: ', num2str(100*consis_l),'%'])
%--------------------------------------------------------------------------
% Plot result
figure('Name','EKF on Follower')
subplot(3,2,1)
plot(data_gnss_ref_f.time(1:nb_steps)-data_gnss_f.time(1), data_gnss_ref_f.x(1:nb_steps))
hold on; grid on
plot(result_f.timestamp,result_f.X(1,:),'r')
legend('Ground truth','Estimate');xlabel('t[s]');ylabel('X[m]')
title('Estimate and ground truth for X')
subplot(3,2,3)
plot(data_gnss_ref_f.time(1:nb_steps)-data_gnss_f.time(1), data_gnss_ref_f.y(1:nb_steps))
hold on; grid on
plot(result_f.timestamp,result_f.X(2,:),'r')
legend('Ground truth','Estimate');xlabel('t[s]');ylabel('Y[m]')
title('Estimate and ground truth for Y')
subplot(3,2,5)
plot(data_gnss_ref_f.time(1:nb_steps)-data_gnss_f.time(1), data_gnss_ref_f.yaw(1:nb_steps))
hold on; grid on
plot(result_f.timestamp,result_f.X(3,:),'r')
legend('Ground truth','Estimate');xlabel('t[s]');ylabel('\psi[rad]')
title('Estimate and ground truth for \psi')

subplot(3,2,2)
plot(result_f.timestamp,result_f.uselidar*20+10,'--','Color',[0.8 0 0 0.2])
hold on; grid on
plot(result_f.timestamp,result_f.err(1,:),'Color',[0 0.6 1])
plot(result_f.timestamp, 2*sqrt(result_f.P(1,:)),'r')
plot(result_f.timestamp,-2*sqrt(result_f.P(1,:)),'r')
ylim([-4 4]);xlabel('t[s]');ylabel('e_x[m]')
title('Estimation error of x with +/- 2 \sigma bounds')

subplot(3,2,4)
plot(result_f.timestamp,result_f.uselidar*20+10,'--','Color',[0.8 0 0 0.2])
hold on; grid on
plot(result_f.timestamp,result_f.err(2,:),'Color',[0 0.6 1])
plot(result_f.timestamp, 2*sqrt(result_f.P(2,:)),'r')
plot(result_f.timestamp,-2*sqrt(result_f.P(2,:)),'r')
ylim([-4 4]);xlabel('t[s]');ylabel('e_y[m]')
title('Estimation error of y with +/- 2 \sigma bounds')

subplot(3,2,6)
plot(result_f.timestamp,result_f.uselidar*20+10,'--','Color',[0.8 0 0 0.2])
hold on; grid on
plot(result_f.timestamp,result_f.err(3,:),'Color',[0 0.6 1])
plot(result_f.timestamp, 2*sqrt(result_f.P(3,:)),'r')
plot(result_f.timestamp,-2*sqrt(result_f.P(3,:)),'r')
ylim([-0.4 0.4]);xlabel('t[s]');ylabel('e_\psi[rad]')
title('Estimation error of \psi with +/- 2 \sigma bounds')


figure('Name','EKF on Leader')
subplot(3,2,1)
plot(data_gnss_ref_l.time(1:nb_steps)-data_gnss_l.time(1), data_gnss_ref_l.x(1:nb_steps))
hold on; grid on
plot(result_l.timestamp,result_l.X(1,:),'r')
legend('Ground truth','Estimate');xlabel('t[s]');ylabel('X[m]')
title('Estimate and ground truth for X')
subplot(3,2,3)
plot(data_gnss_ref_l.time(1:nb_steps)-data_gnss_l.time(1), data_gnss_ref_l.y(1:nb_steps))
hold on; grid on
plot(result_l.timestamp,result_l.X(2,:),'r')
legend('Ground truth','Estimate');xlabel('t[s]');ylabel('Y[m]')
title('Estimate and ground truth for Y')
subplot(3,2,5)
plot(data_gnss_ref_l.time(1:nb_steps)-data_gnss_l.time(1), data_gnss_ref_l.yaw(1:nb_steps))
hold on; grid on
plot(result_l.timestamp,result_l.X(3,:),'r')
legend('Ground truth','Estimate');xlabel('t[s]');ylabel('\psi[rad]')
title('Estimate and ground truth for \psi')

subplot(3,2,2)
plot(result_l.timestamp,result_l.err(1,:))
hold on; grid on
plot(result_l.timestamp, 2*sqrt(result_l.P(1,:)),'r')
plot(result_l.timestamp,-2*sqrt(result_l.P(1,:)),'r')
ylim([-4 4]);xlabel('t[s]');ylabel('e_x[m]')
title('Estimation error of x with +/- 2 \sigma bounds')

subplot(3,2,4)
plot(result_l.timestamp,result_l.err(2,:))
hold on; grid on
plot(result_l.timestamp, 2*sqrt(result_l.P(2,:)),'r')
plot(result_l.timestamp,-2*sqrt(result_l.P(2,:)),'r')
ylim([-4 4]);xlabel('t[s]');ylabel('e_y[m]')
title('Estimation error of y with +/- 2 \sigma bounds')

subplot(3,2,6)
plot(result_l.timestamp,result_l.err(3,:))
hold on; grid on
plot(result_l.timestamp, 2*sqrt(result_l.P(3,:)),'r')
plot(result_l.timestamp,-2*sqrt(result_l.P(3,:)),'r')
ylim([-0.4 0.4]);xlabel('t[s]');ylabel('e_\psi[rad]')
title('Estimation error of \psi with +/- 2 \sigma bounds')
%--------------------------------------------------------------------------