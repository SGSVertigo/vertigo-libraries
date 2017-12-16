function vtg_kalman3d()
%
% This file is part of the Vertigo project
%
% Jon Sowman 2017
% jon+vertigo@jonsowman.com
%

% Acquire data from workspace
gpsdata = evalin('base', 'gpsdata');
imudata = evalin('base', 'imudata');
accel_ned = evalin('base', 'accel_ned');

% Convert GPS data into metres from starting position
gpstime = gpsdata(:,1);
[east, north] = ll2utm(gpsdata(:,4), gpsdata(:,3));
dalt = -(gpsdata(:,5) - gpsdata(1,5)); % Down is positive
deast = east - east(1);
dnorth = north - north(1);

% Interpolate GPS data onto the IMU data
time = imudata(:,1);
deast = interp1(gpstime, deast, time, 'pchip', 'extrap');
dnorth = interp1(gpstime, dnorth, time, 'pchip', 'extrap');
dalt = interp1(gpstime, dalt, time, 'pchip', 'extrap');

% Run Kalman filter
Nstep = length(time);

% State vector: [s_n, s_e, s_d, v_n, v_e, v_d, a_n, a_e, a_d]
x = zeros(9, 1);
P = 1e-3 * eye(9);

dt = time(2) - time(1); % This is still a really bad plan

% Process noise
Q = diag([1e-9 1e-9 1e-9 1e-9 1e-9 1e-9 1e-3 1e-3 1e-3]);
% Measurement noise
gps_var = 1e-6;
imu_var = 1e-3;
R = diag([gps_var gps_var gps_var*10 imu_var imu_var imu_var]);

% Discrete time model
% s_n(k+1) = s_n(k) + v_n(k) * dt
% v_n(k+1) = v_n(k) + a_n(k) * dt
% a_n(k+1) = a_n(k)
F = [1 0 0 dt 0  0  0  0  0; ...
     0 1 0 0  dt 0  0  0  0; ...
     0 0 1 0  0  dt 0  0  0; ...
     0 0 0 1  0  0  dt 0  0; ...
     0 0 0 0  1  0  0  dt 0; ...
     0 0 0 0  0  1  0  0  dt; ...
     0 0 0 0  0  0  1  0  0; ...
     0 0 0 0  0  0  0  1  0; ...
     0 0 0 0  0  0  0  0  1];
 
H = [1 0 0 0 0 0 0 0 0; ...
     0 1 0 0 0 0 0 0 0; ...
     0 0 1 0 0 0 0 0 0; ...
     0 0 0 0 0 0 1 0 0; ...
     0 0 0 0 0 0 0 1 0; ...
     0 0 0 0 0 0 0 0 1];

% Remove gravity from NED accel, leaving linear accels in NED frame
accel_ned(:,3) = accel_ned(:,3) - 1;
accel_ned = accel_ned .* 9.81;

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
view(90,90);

subplot(2,1,2);
plot(time, -kal_x_stor(3, :));
hold on
plot(time, -dalt);
xlabel('Time (s)');
ylabel('Altitude (m)')

% Comet plot
figure;
comet(kal_x_stor(2, :), kal_x_stor(1, :));

end % function