% Vertigo
%
% Jon Sowman 2017
% jon+vertigo@jonsowman.com

% Get raw data
[csvfile, csvpath] = uigetfile('*.csv');
csvdata = csvread([csvpath csvfile]);

% Split into GPS and IMU data
gpsidx = find(csvdata(:,2) == 1);
imuidx = find(csvdata(:,2) == 2);
quatidx = find(csvdata(:,2) == 3);
gpsdata = csvdata(gpsidx, :);
imudata = csvdata(imuidx, :);
quatdata = csvdata(quatidx, :);

% Adjust all times
gpsdata(:,1) = (gpsdata(:,1) - gpsdata(1,1)) / 1000;
imudata(:,1) = (imudata(:,1) - imudata(1,1)) / 1000;
quatdata(:,1) = (quatdata(:,1) - quatdata(1,1)) / 1000;

% Do quaternion->Euler conversion
euldata = zeros(length(quatdata), 3);
for i = 1:length(quatdata)
    euldata(i,:) = vtg_quat2eul(quatdata(i,3:6));
end
%euldata = vtg_quat2eul(quatdata);

% Plot raw imu data
% Accelerations
subplot(3,1,1);
plot(imudata(:,1), imudata(:,3:5));
xlabel('Time (s)');
ylabel('Acceleration (g)');
legend('x', 'y', 'z');

% Rate gyros
subplot(3,1,2);
plot(imudata(:,1), imudata(:,6:8));
xlabel('Time (s)');
ylabel('Gyro (deg/s)');
legend('x', 'y', 'z');

% Plot DMP data
subplot(3,1,3);
plot(quatdata(:,1), euldata);
xlabel('Time (s)');
ylabel('Orientation (deg)');
legend('roll', 'pitch', 'yaw');