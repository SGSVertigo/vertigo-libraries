% Vertigo
%
% Jon Sowman 2017
% jon+vertigo@jonsowman.com
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




t = imudata(:,1); %Time Variable
p0degree = 5; %Polyfit Degree of X
% Acceleration Polyfit
px = polyfit(imudata(:,1), imudata(:,3), p0degree);
ppx = polyval(px, t);

py = polyfit(imudata(:,1), imudata(:,4), p0degree);
ppy = polyval(py, t);

pz = polyfit(imudata(:,1), imudata(:,5), p0degree);
ppz = polyval(pz, t);

% 1st Pass Accelerations var
xclean = imudata(:,3)-ppx;
yclean = imudata(:,4)-ppy;
zclean = imudata(:,5)-ppz;

% 2nd Pass Accelerations var
xclean2 = conv(xclean,ones(5,1)/5,'same');
yclean2 = conv(yclean,ones(5,1)/5,'same');
zclean2 = conv(zclean,ones(5,1)/5,'same');

% 1st Integrals
p1degree = 5;
ix = polyfit(imudata(:,1), xclean2, p1degree);
qx = polyint(ix);
qqx = polyval(ix, t);
iy = polyfit(imudata(:,1), yclean2, p1degree);
qy = polyint(iy);
qqy = polyval(iy, t);
iz = polyfit(imudata(:,1), zclean2, p1degree);
qz = polyint(iz);
qqz = polyval(iz, t);
subplot(5,1,2);
plot(t, qqx, t, qqy, t, qqz);
xlabel('Time (s)');
ylabel('Velocity (ms^-1)');
legend('x', 'y', 'z');  

% 2nd Integrals
rx = polyint(qx);
rrx = polyval(rx, t);
ry = polyint(qy);
rry = polyval(ry, t);
rz = polyint(qz);
rrz = polyval(rz, t);
subplot(5,1,1);
plot(t, rrx, t, rry, t, rrz);
xlabel('Time (s)');
ylabel('Displacement (m)');
legend('x', 'y', 'z');  

% 2nd Pass Accelerations
subplot(5,1,3);
plot(t, xclean2, t, yclean2, t, zclean2);
xlabel('Time (s)');
ylabel('2nd Pass Acceleration (m)');
legend('x', 'y', 'z');  

% 1st Pass Accelerations
subplot(5,1,4);
plot(t, xclean, t, yclean, t, zclean);
xlabel('Time (s)');
ylabel('1st Pass Acceleration (m)');
legend('x', 'y', 'z');  

% Accelerations
subplot(5,1,5);
plot(imudata(:,1), imudata(:,3:5));
% Plot Acceleration Regression Line
hold on;
plot(t, ppx, '--', t, ppy, '--', t, ppz, '--');
xlabel('Time (s)');
ylabel('Acceleration (g)');
legend('x', 'y', 'z');

% Plot raw imu data
% Accelerations
figure
subplot(3,1,1);
plot(imudata(:,1), imudata(:,3:5));
% Plot Acceleration Regression Line
hold on;
plot(t, ppx, '--', t, ppy, '--', t, ppz, '--');
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

% Plot XYZ Displacement
figure 
plot3(rrx, rry, rrz);
grid on
pbaspect([1 1 1])
xlabel('X Displacement (m)');
ylabel('Y Displacement (m)');
zlabel('Z Displacement (m)');

