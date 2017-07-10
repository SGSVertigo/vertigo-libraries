% Jon Sowman 2017
% Jamie Costello
% jon+vertigo@jonsowman.com
%
% This file is part of the Vertigo project
%
% Rotate the IMU data into the NED frame using the AHRS data
accel_ned = zeros(length(imudata), 3);
gyro_ned  = zeros(length(imudata), 3);
t = imudata(:,1);
for i = 1:length(imudata)
    quat_int = interp1(quatdata(:,1), quatdata(:, 3:6), imudata(i,1));
    a = [0 imudata(i, 3:5)];
    g = [0 imudata(i, 6:8)];
    aa = quatmultiply(quatmultiply(quat_int, a), quatconj(quat_int));
    accel_ned(i, :) = aa(2:4);
    gg= quatmultiply(quatmultiply(quat_int, g), quatconj(quat_int));
    gyro_ned(i, :) = gg(2:4);
end

subplot (3,1,1);
time = [diff(imudata(:,1)); 0.01];
North = cumtrapz(imudata(:,1), 9.81 * accel_ned(:,1));
North = cumtrapz(imudata(:,1), North);
East = cumtrapz(imudata(:,1), 9.81 * accel_ned(:,2));
East = cumtrapz(imudata(:,1), East);
Down = cumtrapz(imudata(:,1), 9.81 * (accel_ned(:,3) - 0.981));
Down = cumtrapz(imudata(:,1), Down);
%plot raw positions
plot (t, North, t, East, t, Down);

xlabel('Time (s)');
ylabel('NED Position(m) ');


%find polyfit coefficient Values for each of North, East Down (PCN, PCE,
%PCD) with polynomial of order 10- which may well be excessive.
% Create a polyfit values (PVN, PVE, PVD) for all times (imudata (:,1) -all times from
% column 1)
pcn = polyfit(imudata(:,1),North,1);
pvn = polyval(pcn,imudata(:,1));
pce = polyfit(imudata(:,1),East,1);
pve = polyval(pce,imudata(:,1));
pcd = polyfit(imudata(:,1),Down(:),1);
pvd = polyval(pcd,imudata(:,1));
subplot (3,1,2);

plot (t, North - pvn, t, East - pve,t , Down(:)- pvd);
xlabel('Time (s)');
ylabel('NED Position(m) ');


%choose times to look at in detail - I think between 5 and 20 seconds so:

Corrected_North = North (500:2000)- pvn(500:2000);
Corrected_East = East (500:2000)-pve(500:2000);
Corrected_Down = Down (500:2000)-pvd(500:2000);
Time = imudata(500:2000)- imudata(500);
subplot (3,1,3);
plot ( Time, Corrected_North);
%Time, Corrected_North, Time, Corrected_East,
%Not bad.... but there's more to do in vertigo_wheel_analysis2
xlabel('Time (s)');
ylabel('N Position(m) ');

%figure
%plot (Corrected_North, Corrected_East)
