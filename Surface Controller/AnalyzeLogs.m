close all
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