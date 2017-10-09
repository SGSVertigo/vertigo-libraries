% Vertigo
% Kalman Filtering in 3 dimensions
%
% This file is part of the VertigoIMU project
%
% Jon Sowman 2017
% jon+vertigo@jonsowman.com
% Luke Gonsalves

% The start and end times in the data to process
window_start = 55; % Rocket Default 55 Seconds
window_end = 85; % Rocket Default 85 Seconds

% Extract the bit of data we want to look at
tstartidx = find(imudata(:,1) > window_start, 1);
tendidx = find(imudata(:,1) > window_end, 1);
tstartidx_gps = find(gpsdata(:,1) > window_start, 1);
tendidx_gps = find(gpsdata(:,1) > window_end, 1);

% Find the accel
rock_time = imudata(tstartidx:tendidx, 1);
rock_accel = (accel_ned(tstartidx:tendidx, 3) - 1) * 9.81;

rock_accel_north = (accel_ned(tstartidx:tendidx, 1) ) * 9.81;
rock_accel_east = (accel_ned(tstartidx:tendidx, 2) ) * 9.81;

% Find the gps
rock_time_gps = gpsdata(tstartidx_gps:tendidx_gps, 1);

rock_north_gps = North_utm_position(tstartidx_gps:tendidx_gps, 1); % North 
rock_north_gps = rock_north_gps - rock_north_gps(1);
rock_east_gps = East_utm_position(tstartidx_gps:tendidx_gps, 1); % East 
rock_east_gps = rock_east_gps - rock_east_gps(1);

rock_alt_gps = gpsdata(tstartidx_gps:tendidx_gps, 5);
rock_alt_gps = rock_alt_gps - rock_alt_gps(1);

% Combined IMU and GPS
rock_merged = sortrows([rock_time 1*ones(length(rock_time), 1) [1:length(rock_time)]'; rock_time_gps, 2*ones(length(rock_time_gps), 1), [1:length(rock_time_gps)]']);

% Integrate accel
rock_vel = cumtrapz(rock_time, rock_accel);
rock_pos = cumtrapz(rock_time, rock_vel);

rock_vel_north = cumtrapz(rock_time, rock_accel_north); % North
rock_pos_north = cumtrapz(rock_time, rock_vel_north);

rock_vel_east = cumtrapz(rock_time, rock_accel_east); % East
rock_pos_east = cumtrapz(rock_time, rock_vel_east);

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
kal_x_stor = zeros(length(rock_time), 4);
kal_xn_stor = zeros(length(rock_time), 4);
kal_xe_stor = zeros(length(rock_time), 4);
% Computation time
tc = rock_time(1);

% Run Kalman
for i = 1:length(rock_merged)
    % Find dt since samples can possibly be dropped
    dt = rock_merged(i, 1) - tc;
    tc = rock_merged(i, 1);
    
    % Find DT model
    F = [1 dt 0 0; 0 1 dt 0; 0 0 1 -1; 0 0 0 1];
    
    % Predict
    xp = F * x; % no inputs
    xpn = F * xn;
    xpe = F * xe;
    Pp = F * P * F' + Q;
    
    % Update for accel
    if rock_merged(i, 2) == 1
        y = rock_accel(rock_merged(i, 3)) - Ha * x;
       
        yn = rock_accel_north(rock_merged(i, 3)) - Ha * xn; % North
        ye = rock_accel_east(rock_merged(i, 3)) - Ha * xe; % East

        S = Ha * P * Ha' + Ra;
        K = Pp * Ha' * 1/S;
        x = xp + K * y;
        
        xn = xpn + K * yn; % North
        xe = xpe + K * ye; % East

        P = (eye(4) - K * Ha) * Pp;
    elseif rock_merged(i, 2) == 2
    % Update for GPS
        y = rock_alt_gps(rock_merged(i, 3)) - Hg * x;
        
        yn = rock_north_gps(rock_merged(i, 3)) - Hg * xn; % North
        ye = rock_east_gps(rock_merged(i, 3)) - Hg * xe; % East
             
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
stairs(rock_merged(:,1), kal_x_stor(:,1));
stairs(rock_time, rock_pos);
scatter(rock_time_gps, rock_alt_gps);
line([min(rock_time) max(rock_time)], [0 0], 'LineStyle', '--', 'Color', 'black');
legend('Fusion Est. Altitude', 'IMU Only', 'GPS Samples', 'Launch Alt');
xlabel('Time (s)');
ylabel('Altitude (m)');

subplot(3,1,2);
hold on;
stairs(rock_merged(:,1), kal_xn_stor(:,1));
stairs(rock_time, rock_pos_north);
scatter(rock_time_gps, rock_north_gps);
legend('Fusion Est. North', 'IMU Only', 'GPS Samples');
xlabel('Time (s)');
ylabel('North (m)');

subplot(3,1,3);
hold on;
stairs(rock_merged(:,1), kal_xe_stor(:,1));
stairs(rock_time, rock_pos_east);
scatter(rock_time_gps, rock_east_gps);
legend('Fusion Est. East', 'IMU Only', 'GPS Samples');
xlabel('Time (s)');
ylabel('East (m)');
%subplot(2,1,2);
%hold on;
%stairs(rock_merged(:,1), kal_x_stor(:,4));

figure
subplot(2,1,1);
hold on;
plot(kal_xe_stor(:,1), kal_xn_stor(:,1), rock_pos_east, rock_pos_north, rock_east_gps, rock_north_gps);
legend('Fusion Position', 'Acceleration', 'GPS');
xlabel('East (m)');
ylabel('North (m)');
axis equal;

subplot(2,1,2);
hold on;

plot (East_utm_position, North_utm_position);
legend('GPS UTM Position');
xlabel('East (m)');
ylabel('North (m)');
axis equal;

figure
plot3(kal_xe_stor(:,1), kal_xn_stor(:,1), kal_x_stor(:,1), rock_east_gps, rock_north_gps, rock_alt_gps);
legend('Fusion Position', 'GPS');
xlabel('East (m)');
ylabel('North (m)');
zlabel('Altitude (m)');
axis equal;