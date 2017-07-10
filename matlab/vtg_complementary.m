% Vertigo
%
% Jon Sowman 2017
% jon+vertigo@jonsowman.com

% Column indices in data
xa = 3;
ya = 4;
za = 5;
xg = 6;
yg = 7;
zg = 8;

% Timestep
dt = 1/100;

% Data storage for computed angles
xangle = zeros(length(imudata), 1);
yangle = zeros(length(imudata), 1);
zangle = zeros(length(imudata), 1);

% Constants for comp filter
kgyr = 0.98;
kacc = 0.02;

% Run complementary filter
for i = 1:length(imudata)-1
    xangle(i+1) = kgyr * (xangle(i) + imudata(i, xg) * dt) + kacc * xa;
    yangle(i+1) = kgyr * (yangle(i) + imudata(i, yg) * dt) + kacc * ya;
    zangle(i+1) = kgyr * (zangle(i) + imudata(i, zg) * dt) + kacc * za;
end

% Plot
figure;
hold on;
plot((imudata(:,1) - imudata(1,1))/1000, xangle);
plot((imudata(:,1) - imudata(1,1))/1000, yangle);
plot((imudata(:,1) - imudata(1,1))/1000, zangle);
legend('x', 'y', 'z');
xlabel('Time (s)');
ylabel('Angle (deg)');