% Vertigo
% Kalman Filtering in 3 dimensions
% Roll, pitch Yaw graph plotting
% This file is part of the VertigoIMU project
% Yasith Senanayake 8/10
% Jon Sowman 2017
% jon+vertigo@jonsowman.com
% jcostello@suttonmail.org



% The start and end times in the data to process
prompt = 'What time do you wish to start analysis from? ';
window_start = input(prompt);
prompt = 'What time do you wish to end the analysis? ';
window_end = input(prompt);

% window_start = 0;%sets start time to 0
% window_end = (size(t)-30)*0.01; %sets end time to end of data

all_data = [];
% Extract the bit of data we want to look at
tstartidx = find(imudata(:,1) > window_start, 1);
tendidx = find(imudata(:,1) > window_end, 1);
tstartidx_gps = find(gpsdata(:,1) > window_start, 1);
tendidx_gps = find(gpsdata(:,1) > window_end, 1);


% Find the accel
data_time = imudata(tstartidx:tendidx, 1);
euldata_window = euldata (tstartidx:tendidx, :);
data_accel_down = (accel_ned(tstartidx:tendidx, 3) - 1) * 9.81;
data_accel_north = (accel_ned(tstartidx:tendidx, 1) ) * 9.81;
data_accel_east = (accel_ned(tstartidx:tendidx, 2) ) * 9.81;


% Find the gps in UTM 
data_time_gps = gpsdata(tstartidx_gps:tendidx_gps, 1);
[x,y,zone] = utl_ll2utm(gpsdata(tstartidx_gps:tendidx_gps,4),gpsdata(tstartidx_gps:tendidx_gps,3));
%[x,y,zone] = ll2utm(lat,lon); % do the job!
%gpsdata(:,1) = (gpsdata(:,1) - gpsdata(1,1)) / 1000;
North_utm_position = (x(:,1)- x(1,1));
East_utm_position = (y(:,1)- y(1,1));
%plot (North_utm_position, East_utm_position);


position_north_gps = North_utm_position ;
position_north_gps = position_north_gps - position_north_gps(1);
position_east_gps = East_utm_position ;
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

%smoothing the data
smooth_down_position = smooth(kal_x_stor(:,1));
smooth_east_position = smooth(kal_xe_stor(:,1));
smooth_north_position = smooth (kal_xn_stor(:,1));

%Total time samples
total_time = [ data_time ; data_time_gps];
%sort in time chronological order
stotal_time = sort(total_time);
%all smoothed position data from kalman filter NED with Time - NEDT
NEDT = [smooth_north_position smooth_east_position smooth_down_position stotal_time];

% Convert polar coords to cartesian.  given polar length = quiver_length
quiver_length = 5;
[pitchx,pitchy] = pol2cart(euldata_window(:,1)*2*pi/360,quiver_length);
[rollx,rolly] = pol2cart(euldata_window(:,2)*2*pi/360,quiver_length);
[yawx,yawy] = pol2cart(euldata_window(:,3)*2*pi/360,quiver_length);

%Cartesian vectors for roll, pitch and yaw with time
roll_pitch_yaw_t = [rollx rolly pitchx pitchy yawx yawy data_time];

%joining all data and equalising matrix sizes
for z = 1 :length(NEDT)    
    for  i = 1: length(data_time)    
        if roll_pitch_yaw_t(i,7) == NEDT(z,4)
            all_data (i,:) = horzcat(NEDT (z,:), roll_pitch_yaw_t(i,:));
        end
    end
end

%all_data holds [N E D T Rx Ry Px Py Yx Yy T]


%arrow_size = as
as = 0.4;

%decimate_rate  = dr
dr = 50;
dec_data=utl_decimatrix(all_data,dr);


%quiver plot yaw
figure;
quiver (decimate(all_data (:,2),dr),decimate(all_data (:,1),dr), ...
    decimate(all_data (:,9),dr),decimate(all_data (:,10),dr),as);
hold on
plot (decimate(all_data (:,2),dr),decimate(all_data (:,1),dr));
legend('Yaw at position');
xlabel('East Position(m)');
ylabel('North Position(m)');
hold off


%quiver plot roll
figure;
quiver (decimate(all_data (:,2),dr),decimate(all_data (:,3),dr), ...
    decimate(all_data (:,5),dr),decimate(all_data (:,6),dr),as);
hold on
plot (decimate(all_data (:,2),dr),decimate(all_data (:,3),dr));
legend('Roll at position');
xlabel('East Position(m)');
ylabel('Down Position(m)');
hold off

%quiver plot pitch
figure;
quiver (decimate(all_data (:,1),dr),decimate(all_data (:,3),dr), ...
    decimate(all_data (:,7),dr),decimate(all_data (:,8),dr),as);
hold on
plot(decimate(all_data (:,1),dr),decimate(all_data (:,3),dr));
legend('Pitch at position');
xlabel('North Position(m)');
ylabel('Down Position(m)');
hold off

figure;
plot(data_time, euldata_window(:,1:3));
xlabel('Time (s)');
ylabel('Orientation (deg)');
legend('roll', 'pitch', 'yaw');

figure('Name','3D Orientation vs. Position');
%lines if my decimate worked


%all_data holds [N E D T Rx Ry Px Py Yx Yy T ]
%all_data holds [1 2 3 4 5  6  7  8  9  10 11]
quiver3(dec_data(:,1),dec_data(:,2),dec_data(:,3),dec_data(:,5),dec_data(:,7),dec_data(:,9),as)
hold on
plot3(dec_data(:,1),dec_data(:,2),dec_data(:,3));
legend('Orientation at Position');
xlabel('North Displacement /m | Roll');
ylabel('East Displacement /m | Pitch');
zlabel('Vertical Displacement /m | Yaw');
daspect([1 1 1])
hold off

