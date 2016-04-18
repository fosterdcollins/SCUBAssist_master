close all

%4-1_20-52
figures = 1;
subplots = 0;
tstart = 0;
tend = 500;

looptime = 0.1; %s

t = 1:length(receivingLog);
t = t' * looptime;

if figures
    figure
elseif subplots
    subplot(4,1,1) %depth
end
plot(t,receivingLog(:,5)/100);
hold on
plot(t,receivingLog(:,10)/100 + 0.25);
title('Closed Loop Depth Profile Tracking');
ylabel('Depth [ft]');
xlabel('Time [sec]');
xlim([tstart tend]);
grid on
box off
legend('Depth','Depth Set Point','Location','NorthWest');

if figures
    figure
elseif subplots
    subplot(4,1,2) %yaw
end
plot(t,receivingLog(:,2)/100 + 180);
hold on
plot(t,receivingLog(:,9)/10);%-1);
title('Yaw vs Yaw Set');
ylabel('Yaw [deg]');
xlabel('Time [sec]');
xlim([tstart tend]);
ylim([0 360]);
grid on
box off
legend('Heading','Heading Set Point','Location','NorthWest');

if figures
    figure
elseif subplots
    subplot(4,1,3) %pitch
end
plot(t,receivingLog(:,4)/100)
title('Pitch');
ylabel('Pitch [deg]');
xlim([tstart tend]);
ylim([-50,50]);
grid on
box off

if figures
    figure;
elseif subplots
    subplot(4,1,4) %roll
end
plot(t,receivingLog(:,3)/100);
title('Roll');
xlabel('Time [sec]');
ylabel('Roll [deg]');
xlim([tstart tend]);
ylim([-80,80]);
grid on;
box off;