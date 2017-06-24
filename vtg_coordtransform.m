% Jon Sowman 2017
% jon+vertigo@jonsowman.com
%
% This file is part of the Vertigo project
%
% Rotate the IMU data into the NED frame using the AHRS data

accel_ned = zeros(length(imudata), 3);
gyro_ned  = zeros(length(imudata), 3);

for i = 1:length(imudata)
    quat_int = interp1(quatdata(:,1), quatdata(:, 3:6), imudata(i,1));
    a = [0 imudata(i, 3:5)];
    g = [0 imudata(i, 6:8)];
    aa = vtg_quatmultiply(vtg_quatmultiply(quat_int, a), quatconj(quat_int));
    accel_ned(i, :) = aa(2:4);
    gg= vtg_quatmultiply(vtg_quatmultiply(quat_int, g), quatconj(quat_int));
    gyro_ned(i, :) = gg(2:4);
end

% Plot
subplot(4,1,1);
plot(imudata(:,1), imudata(:, 3:5));
xlabel('Time (s)');
ylabel('Raw accel (g)');
legend('x', 'y', 'z');

subplot(4,1,2);
plot(imudata(:,1), accel_ned);
xlabel('Time (s)');
ylabel('NED accel (g)');
legend('N', 'E', 'D');

subplot(4,1,3);
plot(imudata(:,1), imudata(:, 6:8));
xlabel('Time (s)');
ylabel('Raw gyro (deg/s)');
legend('x', 'y', 'z');

subplot(4,1,4);
plot(imudata(:,1), gyro_ned);
xlabel('Time (s)');
ylabel('NED gyro (deg/s)');
legend('N', 'E', 'D');