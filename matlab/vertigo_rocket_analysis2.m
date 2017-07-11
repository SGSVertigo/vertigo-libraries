plot (Time, Corrected_North);
% once plotted pick the cursor data tool on the figure screen.  Mark a
% point at the top of the first blue peak.  Then right click, add new
% cursor data point and point out the second blue peak.  You may have to
% move the information square.  Do this for all of the blue peaks- IN
% ORDER!

%finally, right click and export data to workspace
%Create a matrix for this new data call it A

A = [cursor_info.Position];


%All of my times are X coordintaes and they area in the odd positions in the columns so:
%My new matrix T holds all of the time values 
T = A([1 3 5 7 9 11 13]);

%Similarly my North positions are all in even slots so:
N = A ([2 4 6 8 10 12 14 ])

figure
%plot (T, N);
% now find a polyfit coefficients for this new graph
pcN = polyfit(T,N,5);

%find values in Time - remember time is 5- 20 seconds from the imudata
pvN = polyval(pcN,Time);

plot (Time, pvN);
% and finally we plot the most corrected graph - not pvN' that means I have
% swapped Columns into rows- that's all.
plot (Time, (Corrected_North - pvN'));
%plot (Time, (Corrected_East - pvE'));
% you need to go through this process for the East direction.
% To check progress you can always remove the % sign and plot one of the
% graphs - play around with the power of the polyfit curve to get a better
% fit.





