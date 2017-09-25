% rotm is far superior as it doesn't use any non vanilla functions
% Luke Gonsaves 2017, superior leader of vertigo. Bless this mess.
% Tom Ronayne 2017, super boffin and rotm enthusiast
% vtg_raw_data.m => rotmftw.m
% In the messiest way possible, uses Euler angles to rotate imu into the
% world frame, then combines imu with gps data to create a graph of
% accelerations

euldatarad = euldata*(pi/180); % As rad as it sounds
rotm = eul2rotm(euldatarad); % Done in the order zyx (idk if that matters)
imudatasplit = permute(imudata(:,3:5),[2 3 1]); % instead of n by m its 1 by m by n
aworld = zeros(3,length(quatdata));
for i = 1:length(quatdata)
    aworld(:,i) = rotm(:,:,i) * imudatasplit(1:3,:,i); % Rotation matrix applied to each data point
end

aworld = aworld'; % Flips it the right way up (wide to tall)
ax = aworld(:,1);
ay = aworld(:,2);
az = aworld(:,3);

for i = 1:10:(length(quatdata)-40) % -40 is to sort out the matrix dimensions
    inew = i+9; % The reason for the 10 is because there's 10* more imu data than gps
   if mod(inew,200) == 0 % The second variable in mod affects the density of arrows
       axg(inew/10) = ax(i);
       ayg(inew/10) = ay(i);
       azg(inew/10) = az(i);
   else
       axg(inew/10) = 0;
       ayg(inew/10) = 0;
       azg(inew/10) = 0;
   end
end

axgr = axg';
aygr = ayg';
azgr = azg';

plot (East_utm_position,North_utm_position);
axis equal;
xlabel('East (m)');
ylabel('North (m)');
hold on
% quiver(East_utm_position,North_utm_position,axgr,aygr,3); % Last number is scale
quiver3(East_utm_position,North_utm_position,gpsdata(:,5),axgr,aygr,azgr,3); % Last number is scale