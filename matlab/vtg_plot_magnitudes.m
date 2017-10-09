%Ben Kaye

run('vtg_plot_pos_vel_accel.m');
accelmag=(abs(kal_xn_stor(:,3))/9.81);
accelt=stotal_time;
% pfit=polyfit(accelt,accelmag,7);

figure('Name', 'Linear Acceleration')
% plot(pfit);
plot(accelt,accelmag);
xlabel('Time/s');
ylabel('Acceleration/g');