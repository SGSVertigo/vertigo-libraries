% Plot each corrected displacement vs time to find cursor data.
figure
subplot(3,1,1)
plot (Time, Corrected_North)
ylabel('N')
subplot(3,1,2)
plot (Time, Corrected_East)
ylabel('E')
subplot(3,1,3)
plot (Time, Corrected_Down)
ylabel('D')

% REDUNDANT as already collected tooltip data
%Save the cursor data in a matrix for each plane.
%A = [wheelcursorn.Position];
%B = [wheelcursore.Position];
%C = [wheelcursord.Position];
%All of the times are X coordinates and they are in the odd positions in the columns so:
%new matrix T holds all of the time values 
%ET = B([15 13 11 9 7 5 3 1]);
%CT = C([19 17 15 13 11 9 7 5 3 1]);
%Do the same for displacement
%E = B ([16 14 12 10 8 6 4 2]);
%D = C ([20 18 16 14 12 10 8 6 4 2]);
%figure
%subplot(3,1,1)
%plot (T, N);
%subplot(3,1,2)
%plot (BT, E);
%subplot(3,1,3)
%plot (CT, D);

%Plot existing cursor data
figure
subplot(3,1,1)
plot (cursorn.Time, cursorn.Position)
subplot (3,1,2)
plot (cursore.Time, cursore.Position)
subplot (3,1,3)
plot (cursord.Time, cursord.Position)

% now find a polyfit coefficients for these graphs
pcN = polyfit(cursorn.Time, cursorn.Position,5);
pcE = polyfit(cursore.Time, cursore.Position,5);
pcD = polyfit(cursord.Time, cursord.Position,5);

%pcE = polyfit (BT,E,5);
%pcD = polyfit (CT,D,5);

%find values in Time - remember time is 5- 20 seconds from the imudata

pvN = polyval(pcN,Time);
pvE = polyval(pcE,Time);
pvD = polyval(pcD,Time);

figure
%Plot the polyvalues. Mainly for checking
subplot (3,1,1)
plot (Time, pvN);
subplot(3,1,2)
plot (Time, pvE);
subplot(3,1,3)
plot (Time, pvD);


figure
%Plot each adjusted displacement vs time
subplot(3,1,1)
plot (Time, (Corrected_North - pvN'));
subplot(3,1,2)
plot (Time, (Corrected_East - pvE'));
subplot(3,1,3)
plot (Time, (Corrected_Down - pvE'));

figure
%Plot the displacements vs each other
plot3 ((smooth(Corrected_East - pvE', 'moving')),(smooth(Corrected_North - pvN', 'moving')),(smooth(Corrected_Down - pvD', 'moving')))
xlabel('East Displacement');
ylabel('North Displacement');
zlabel('Down Displacement');







