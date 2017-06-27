% Output Graph module
% Luke Gonsalves 2017
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

euldatarad = euldata*(pi/180); %As rad as it sounds

%TOM CODING (data taken every 0.01s)
int = 600:3000; %time interval for polyfit zero error removal
rotm = eul2rotm(euldatarad); %done in the order zyx (idk if that matters)
imudatasplit = permute(imudata(:,3:5),[2 3 1]); %instead of n by m its 1 by m by n

aworld = zeros(3,length(quatdata));
for i = 1:length(quatdata)
aworld(:,i) = rotm(:,:,i) * imudatasplit(1:3,:,i); %rotation matrix applied to each data point
end
aworld = aworld'; %flips it the right way up (wide to tall)

t = imudata(:,1); %Time Variable

smoothax = smooth(imudata(:,3), 0.3, 'lowess');
smoothay = smooth(imudata(:,4), 0.3, 'lowess');
smoothaz = smooth(imudata(:,5), 0.3, 'lowess');

figure
subplot(4,1,4);
plot(t, smoothax, t, smoothay, t, smoothaz);
xlabel('Time (s)');
ylabel('Cleaned Acceleration (g)');
legend('x', 'y', 'z');

% Plot raw imu data
% Accelerations

subplot(4,1,1);
plot(imudata(:,1), imudata(:,3:5));
xlabel('Time (s)');
ylabel('Acceleration (g)');
legend('x', 'y', 'z');

% Rate gyros
subplot(4,1,2);
plot(imudata(:,1), imudata(:,6:8));
xlabel('Time (s)');
ylabel('Gyro (deg/s)');
legend('x', 'y', 'z');

% Plot DMP data
subplot(4,1,3);
plot(quatdata(:,1), euldata);
xlabel('Time (s)');
ylabel('Orientation (deg)');
legend('roll', 'pitch', 'yaw');