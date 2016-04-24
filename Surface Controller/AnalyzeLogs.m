close all
time = 0:.1:
subplot(4,1,1) %depth
plot(receivingLog(:,5)/100);
hold on
plot(receivingLog(:,10)/100);
title('Depth vs Depth Set');
ylabel('Depth [m]');

subplot(4,1,2)
plot(receivingLog(:,2)/100 + 180);
hold on
plot(receivingLog(:,9)/10);
title('Yaw vs Yaw Set');
ylabel('Yaw [deg]');

subplot(4,1,3) %pitch
plot(receivingLog(:,4)/100)
title('Pitch');
ylabel('Pitch [deg]');

subplot(4,1,4) %roll
plot(receivingLog(:,3)/100);
title('Roll');
ylabel('Roll [deg]');

%%
figure
time = 0:.1:1000;
plot(time(1:length(receivingLog)), receivingLog(:,5)/100-.3);
hold on
plot(time(1:length(receivingLog)),receivingLog(:,10)/100);
title('Depth vs Depth Set', 'FontSize',18);
ylabel('Depth [m]', 'FontSize', 14);
xlabel('time (sec)', 'FontSize', 14)
legend('Vehicle Depth', 'User Setpoint', 'FontSize', 14);
grid on

%%
figure
plot(time(1:length(receivingLog)),receivingLog(:,2)/100 + 180);
hold on
plot(time(1:length(receivingLog)),receivingLog(:,9)/10);
title('Heading vs Heading Setpoint', 'FontSize',18);
ylabel('Heading [deg]', 'FontSize', 14);
xlabel('time (sec)', 'FontSize', 14)
legend('Vehicle Heading', 'User Setpoint', 'FontSize', 14);
grid on

