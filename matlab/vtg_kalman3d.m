%
% This file is part of the Vertigo project
%
% Jon Sowman 2017
% jon+vertigo@jonsowman.com
%

% Convert GPS data into metres from starting position
gpstime = gpsdata(:,1);
[east, north] = ll2utm(gpsdata(:,4), gpsdata(:,3));
dalt = -(gpsdata(:,5) - gpsdata(1,5)); % Down is positive
deast = east - east(1);
dnorth = north - north(1);

% Interpolate GPS data onto the IMU data
time = imudata(:,1);
deast = interp1(gpstime, deast, time, 'linear', 'extrap');
dnorth = interp1(gpstime, dnorth, time, 'linear', 'extrap');
dalt = interp1(gpstime, dalt, time, 'linear', 'extrap');

% Run Kalman filter
Nstep = length(time);

% State vector: [s_n, s_e, s_d, v_n, v_e, v_d, a_n, a_e, a_d]
x = zeros(9, 1);
P = eye(9);

dt = time(2) - time(1); % This is still a really bad plan

% Process noise
Q = diag([1e-9 1e-9 1e-9 1e-9 1e-9 1e-9 1e-3 1e-3 1e-3]);
% Measurement noise
gps_var = 1e-7;
imu_var = 1e-5;
R = diag([gps_var gps_var gps_var*1e3 imu_var imu_var imu_var]);

% DT model
F = [1 0 0 dt 0  0  0 0 0; ...
     0 1 0 0  dt 0  0 0 0; ...
     0 0 1 0  0  dt 0 0 0; ...
     0 0 0 1  0  0  0 0 0; ...
     0 0 0 0  1  0  0 0 0; ...
     0 0 0 0  0  1  0 0 0; ...
     0 0 0 0  0  0  0 0 0; ...
     0 0 0 0  0  0  0 0 0; ...
     0 0 0 0  0  0  0 0 0];
 
H = [1 0 0 0 0 0 0 0 0; ...
     0 1 0 0 0 0 0 0 0; ...
     0 0 1 0 0 0 0 0 0; ...
     0 0 0 0 0 0 1 0 0; ...
     0 0 0 0 0 0 0 1 0; ...
     0 0 0 0 0 0 0 0 1];

% Sensors
sensors = @(i) [dnorth(i);
                deast(i);
                dalt(i);
                accel_ned(i, :)'];
                
kal_x_stor = zeros(9, Nstep);

h = waitbar(0, 'Running Kalman filter...');
for i = 1:Nstep
    % Predict
    xp = F * x; % no inputs
    Pp = F * P * F' + Q;
    
    % Update
    y = sensors(i) - H * x;
    S = H * P * H' + R;
    K = Pp * H' * inv(S);
    x = xp + K * y;
    P = (eye(9) - K * H) * Pp;
    
    % Store
    kal_x_stor(:, i) = x;
    
    % Waitbar
    if mod(i/Nstep, 0.1) == 0
        waitbar(i/Nstep, h);
    end
end
close(h);

% Plot results
clf;
subplot(2,1,1);
plot3(kal_x_stor(2, :), kal_x_stor(1, :), -kal_x_stor(3, :));
hold on;
plot3(deast, dnorth, -dalt);
xlabel('North (m)');
ylabel('East (m)');

subplot(2,1,2);
plot(time, -kal_x_stor(3, :));
hold on
plot(time, -dalt);
xlabel('Time (s)');
ylabel('Altitude (m)')