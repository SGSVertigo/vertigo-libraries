% Vertigo
%
% Jon Sowman 2017
% Tom Ronayne 2017
% Luke Gonsalves 2017
% David Rudolph 2017
% jon+vertigo@jonsowman.com

% Ignoring gps for now (btw imu is acceleration data)

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

ax = aworld(:,1);
ay = aworld(:,2);
az = aworld(:,3);
% Smooth acceleration data using s-golay filter
smoothax = smooth(ax, 0.3, 'sgolay');
smoothay = smooth(ay, 0.3, 'sgolay');
smoothaz = smooth(az, 0.3, 'sgolay');

apolyx = polyfit(quatdata(int,1),ax(int),1);
apolyy = polyfit(quatdata(int,1),ay(int),1);
apolyz = polyfit(quatdata(int,1),az(int),1);
for i = 1:length(quatdata)
    axc(i,1) = smoothax(i) - apolyx(1)*quatdata(i,1) - apolyx(2);
    ayc(i,1) = smoothay(i) - apolyy(1)*quatdata(i,1) - apolyy(2);
    azc(i,1) = smoothaz(i) - apolyz(1)*quatdata(i,1) - apolyz(2);
end
for i = 1:600 %remove acceleration at the begining to remove velocity drift
    axc(i) = 0;
    ayc(i) = 0;
    azc(i) = 0;
end
    vx = cumsum(axc);
    vy = cumsum(ayc);
    vz = cumsum(azc);
    vpolyx = polyfit(quatdata(int,1),vx(int),2);
    vpolyy = polyfit(quatdata(int,1),vy(int),2);
    vpolyz = polyfit(quatdata(int,1),vz(int),2);
for i = 1:length(quatdata)
    vxc(i,1) = vx(i) - vpolyx(1)*(quatdata(i,1))^2 - vpolyx(2)*quatdata(i,1) - vpolyx(3);
    vyc(i,1) = vy(i) - vpolyy(1)*(quatdata(i,1))^2 - vpolyy(2)*quatdata(i,1) - vpolyy(3);
    vzc(i,1) = vz(i) - vpolyz(1)*(quatdata(i,1))^2 - vpolyz(2)*quatdata(i,1) - vpolyz(3);
end
for i = 1:600 %remove velocity
    vxc(i) = 0;
    vyc(i) = 0;
    vzc(i) = 0;
end
    x = cumsum(vxc);
    y = cumsum(vyc);
    z = cumsum(vzc)
    t = imudata(1:length(quatdata),1);

% Plot raw imu data
% Accelerations
    subplot(3,2,1);
    hold on;
    plot(t, axc);
    plot(t, ayc);
    plot(t, azc);
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    legend('x', 'y', 'z');

% Plot Accelerations Against Time

% Velocity TOM MADE TIHS
    subplot(3,2,2);
    hold on;
    plot(t, vxc);
    plot(t, vyc);
    plot(t, vzc);
    xlabel('Time (s)');
    ylabel('Velocity (idek)');
    legend('x', 'y', 'z');

    subplot(3,2,3);
    hold on;
    plot(t, x);
    plot(t, y);
    plot(t, z);
    xlabel('Time (s)');
    ylabel('Displacement (idek)');
    legend('x', 'y', 'z');

% Rate gyros
% Experimental Filter
    gyrox = smooth(imudata(:,6), 0.3, 'sgolay');
    gyroy = smooth(imudata(:,7), 0.3, 'sgolay');
    gyroz = smooth(imudata(:,8), 0.3, 'sgolay');
    subplot(3,2,4);
    hold on;
    plot(imudata(:,1), imudata(:,6:8), '-');
    hold on;
    plot(imudata(:,1), gyrox);
    plot(imudata(:,1), gyroy);
    plot(imudata(:,1), gyroz);
    xlabel('Time (s)');
    ylabel('Gyro (Rad/s)');
    legend('x', 'y', 'z');

% Plot DMP data
    subplot(3,2,5);
    plot(quatdata(:,1), euldata);
    xlabel('Time (s)');
    ylabel('Orientation (deg)');
    legend('roll', 'pitch', 'yaw');

%animated plot
    figure
    plot3(x, y, z);
    xlabel('X displacement');
    ylabel('Y displacement');
    zlabel('Z displacement');


