% Vertigo
% Kalman Filtering in 3 dimensions
%
% This file is part of the VertigoIMU project
%
% Jon Sowman 2017
% jon+vertigo@jonsowman.com
% Luke Gonsalves

% The start and end times in the data to process
window_start = 0; % Rocket Default 55 Seconds
window_end =  133;% Rocket Default 85 Seconds

% Extract the bit of data we want to look at
tstartidx = find(imudata(:,1) > window_start, 1);
tendidx = find(imudata(:,1) > window_end, 1);
tstartidx_gps = find(gpsdata(:,1) > window_start, 1);
tendidx_gps = find(gpsdata(:,1) > window_end, 1);

% Find the accel
data_time = imudata(tstartidx:tendidx, 1);
data_accel_down = (accel_ned(tstartidx:tendidx, 3) - 1) * 9.81;
data_accel_north = (accel_ned(tstartidx:tendidx, 1) ) * 9.81;
data_accel_east = (accel_ned(tstartidx:tendidx, 2) ) * 9.81;


% Find the gps in UTM 
data_time_gps = gpsdata(tstartidx_gps:tendidx_gps, 1);
[x,y,zone] = ll2utm(gpsdata(tstartidx_gps:tendidx_gps,4),gpsdata(tstartidx_gps:tendidx_gps,3));
%[x,y,zone] = ll2utm(lat,lon); % do the job!
%gpsdata(:,1) = (gpsdata(:,1) - gpsdata(1,1)) / 1000;
North_utm_position = (x(:,1)- x(1,1));
East_utm_position = (y(:,1)- y(1,1));
%plot (North_utm_position, East_utm_position);


position_north_gps = North_utm_position 
position_north_gps = position_north_gps - position_north_gps(1);
position_east_gps = East_utm_position 
position_east_gps = position_east_gps - position_east_gps(1);
data_alt_gps = gpsdata(tstartidx_gps:tendidx_gps, 5);
data_alt_gps = data_alt_gps - data_alt_gps(1);

% Combined IMU and GPS
data_merged = sortrows([data_time 1*ones(length(data_time), 1) [1:length(data_time)]'; data_time_gps, 2*ones(length(data_time_gps), 1), [1:length(data_time_gps)]']);

% Integrate accel
data_vel = cumtrapz(data_time, data_accel_down);
data_pos = cumtrapz(data_time, data_vel);

data_vel_north = cumtrapz(data_time, data_accel_north); % North
data_pos_north = cumtrapz(data_time, data_vel_north);

data_vel_east = cumtrapz(data_time, data_accel_east); % East
data_pos_east = cumtrapz(data_time, data_vel_east);

% Euler: x+ = x + dx * dt
% Model: Fx + Gu (no inputs, random jerk)
% s+ = s + dt * v
% v+ = v + dt * a
Hg = [1 0 0 0]; % measure gps
Ha = [0 0 1 1]; % measure acc

% Initial state and covariance
x = [0 0 0 0]';
xn = [0 0 0 0]';
xe = [0 0 0 0]';
P = zeros(4, 4); P(3,3) = 1; P(4,4) = 1e0;

% Process noise
Q = diag([0 0 1e-4 1e-9]);
% Measurement noise
Ra = 1;
Rg = 0.01;

% Store Kalman
kal_x_stor = zeros(length(data_time), 4);
kal_xn_stor = zeros(length(data_time), 4);
kal_xe_stor = zeros(length(data_time), 4);
% Computation time
tc = data_time(1);

% Run Kalman
for i = 1:length(data_merged)
    % Find dt since samples can possibly be dropped
    dt = data_merged(i, 1) - tc;
    tc = data_merged(i, 1);
    
    % Find DT model
    F = [1 dt 0 0; 0 1 dt 0; 0 0 1 -1; 0 0 0 1];
    
    % Predict
    xp = F * x; % no inputs
    xpn = F * xn;
    xpe = F * xe;
    Pp = F * P * F' + Q;
    
    % Update for accel
    if data_merged(i, 2) == 1
        y = data_accel_down(data_merged(i, 3)) - Ha * x;
       
        yn = data_accel_north(data_merged(i, 3)) - Ha * xn; % North
        ye = data_accel_east(data_merged(i, 3)) - Ha * xe; % East

        S = Ha * P * Ha' + Ra;
        K = Pp * Ha' * 1/S;
        x = xp + K * y;
        
        xn = xpn + K * yn; % North
        xe = xpe + K * ye; % East

        P = (eye(4) - K * Ha) * Pp;
    elseif data_merged(i, 2) == 2
    % Update for GPS
        y = data_alt_gps(data_merged(i, 3)) - Hg * x;
        
        yn = position_north_gps(data_merged(i, 3)) - Hg * xn; % North
        ye = position_east_gps(data_merged(i, 3)) - Hg * xe; % East
             
        S = Hg * P * Hg' + Rg;
        K = Pp * Hg' * 1/S;
        x = xp + K * y;
        
        xn = xpn + K * yn; % North
        xe = xpe + K * ye; % East

        P = (eye(4) - K * Hg) * Pp;
    end
    
    % Store
    kal_x_stor(i, :) = x;
    kal_xn_stor(i, :) = xn;
    kal_xe_stor(i, :) = xe;
end

% Plot
clf;
subplot(3,1,1);
hold on;
stairs(data_merged(:,1), kal_x_stor(:,1));
stairs(data_time, data_pos);
scatter(data_time_gps, data_alt_gps);
line([min(data_time) max(data_time)], [0 0], 'LineStyle', '--', 'Color', 'black');
legend('Fusion Est. Altitude', 'IMU Only', 'GPS Samples', 'Launch Alt');
xlabel('Time (s)');
ylabel('Altitude (m)');

subplot(3,1,2);
hold on;
stairs(data_merged(:,1), kal_xn_stor(:,1));
stairs(data_time, data_pos_north);
scatter(data_time_gps, position_north_gps);
legend('Fusion Est. North', 'IMU Only', 'GPS Samples');
xlabel('Time (s)');
ylabel('North (m)');

subplot(3,1,3);
hold on;
stairs(data_merged(:,1), kal_xe_stor(:,1));
stairs(data_time, data_pos_east);
scatter(data_time_gps, position_east_gps);
legend('Fusion Est. East', 'IMU Only', 'GPS Samples');
xlabel('Time (s)');
ylabel('East (m)');
%subplot(2,1,2);
%hold on;
%stairs(rock_merged(:,1), kal_x_stor(:,4));

figure
%subplot(2,1,1);
hold on;
plot(kal_xe_stor(:,1), kal_xn_stor(:,1), position_east_gps, position_north_gps);
legend('Fusion Position', 'GPS');
xlabel('East (m)');
ylabel('North (m)');
axis equal;

%subplot(2,1,2);
%hold on;

%plot (East_utm_position, North_utm_position);
%legend('GPS UTM Position');
%xlabel('East (m)');
%ylabel('North (m)');
%axis equal;

figure
plot3(kal_xe_stor(:,1), kal_xn_stor(:,1), kal_x_stor(:,1), position_east_gps, position_north_gps, data_alt_gps);
legend('Fusion Position', 'GPS');
xlabel('East (m)');
ylabel('North (m)');
zlabel('Altitude (m)');
axis equal;

figure
plot (kal_xe_stor(:,1), kal_xn_stor(:,1));
legend('Fusion Position');
xlabel('East (m)');
ylabel('North (m)');

smooth_east_position = smooth(kal_xe_stor(:,1));
smooth_north_position = smooth (kal_xn_stor(:,1));
figure
plot (smooth_east_position,smooth_north_position);
legend('smoothFusion Position');
xlabel('smooth East (m)');
ylabel('smooth North (m)');

% Plot quiver plot showing accelerations for each position
figure;
%north_accel_elements =  data_accel_north(1:10:end,:);
%east_accel_elements =  data_accel_east(1:10:end,:);
north_accel_elements = smooth(kal_xn_stor(:,3));
east_accel_elements = smooth(kal_xe_stor(:,3));
positionsE = smooth_east_position(:,:);
positionsN = smooth_north_position(:,:);
quiv_ds_rate = 50; % downsample rate
quiver(decimate(positionsE, quiv_ds_rate), ...
    decimate(positionsN, quiv_ds_rate), ...
    decimate(east_accel_elements, quiv_ds_rate), ...
    decimate(north_accel_elements, quiv_ds_rate));


