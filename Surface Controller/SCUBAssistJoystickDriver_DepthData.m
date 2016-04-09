close all; clear all;
%fclose(arduino);
ArduinoPresent = 1;
CompassEnabled = 1;
PromptSaving = 1;
mode = 2; % 0=debug, 1=open, 2=closed, 3=auto

%%%%%%%%%%%%%%%%%%%%  Gains  %%%%%%%%%%%%%%%%%%%%%%%%%%%%
Kp_Heading = .85; %.7
Kd_Heading = .5; %.36

Kp_Depth = 3;
Kd_Depth = 0;
Ki_Depth = 0;

Kp_Lateral = 1;
Kd_Lateral = 0;

Gains = round([Kp_Heading Kd_Heading Kp_Depth Kd_Depth Ki_Depth Kp_Lateral Kd_Lateral]*1000);

%%%%%%%%%%%%%%%%%%%%  Gains  %%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ArduinoPresent
    arduino = serial('COM54','BaudRate',115200); %change COM port as needed
    fopen(arduino)
%     s.BytesAvailableFcn = @ArduinoSerial;
%     arduino.BytesAvailableFcnMode = 'terminator';
%     arduino.Terminator = ';';
    set(arduino,'Timeout',.025);
    pause(2);
    dataStrGain = ['G' num2str(Gains(1)) ',' num2str(Gains(2)) ',' num2str(Gains(3)) ',' num2str(Gains(4)) ',' num2str(Gains(5)) ',' num2str(Gains(6)) ',' num2str(Gains(7)) ';'];
   
    for i = 1:2
        fprintf(arduino, dataStrGain);
    end
    receivingLog = zeros(1,10);  % Batt, Yaw, Roll, Pitch, Depth, Ax, Ay, Az, YawSet, DepthSet
    sendingLog = zeros(1,5);
end

h.joy = vrjoystick(1);

HG_Open = 2;
RG_Open = 1;
VG_Open = 6;

%closeedLoopGain
HG_Closed = 1.5;
RG_Closed = 5;
VG_Closed = 6;

SetPoint = [0,0];

%input data
COP = [0, 0 0];
T1_posX = -5.5; %FRONT RIGHT
T1_posY = 8.5;
T1_VectX = 1/2;
T1_VectY = sqrt(3)/2;

T2_posX = 5.5; %FRONT LEFT
T2_posY = 8.5;
T2_VectX = -1/2;
T2_VectY = sqrt(3)/2;

T3_posX = 0; %REAR
T3_posY = -12;
T3_VectX = 1;
T3_VectY = 0;

%%
T1_posX = 8.5; %FRONT RIGHT
T1_posY = 5.5;
T1_VectX = -sqrt(3)/2;
T1_VectY = 1/2;

T2_posX = 8.5; %FRONT LEFT
T2_posY = -5.5;
T2_VectX = -sqrt(3)/2;
T2_VectY = -1/2;

T3_posX = -12; %REAR
T3_posY = 0;
T3_VectX = 0;
T3_VectY = 1;

R1 = cross([T1_posX T1_posY 0]-COP, [T1_VectX, T1_VectY 0]);
R2 = cross([T2_posX T2_posY 0]-COP, [T2_VectX, T2_VectY 0]);
R3 = cross([T3_posX T3_posY 0]-COP, [T3_VectX, T3_VectY 0]);

A = [T1_VectX, T2_VectX, T3_VectX;
    T1_VectY, T2_VectY, T3_VectY;
    R1(3), R2(3), R3(3)];

Ainv = inv(A);

PowerFit = [0.9216, 7.0716, 3.6057]; %[a0, a1, a2]
rad2deg = 180/pi;
accel_0g = [140 -64 -750];
accel_1g = [16900 16400 16700]; %1g in x,y,z
accel_scale = accel_1g - accel_0g;

%ploting
figure;
hold on
axis equal
axis([-8 8 -13 13])
plot([T1_posX, T1_posX+T1_VectX], [T1_posY, T1_posY+T1_VectY], 'b-')
plot([T2_posX, T2_posX+T2_VectX], [T2_posY, T2_posY+T2_VectY], 'b-')
plot([T3_posX, T3_posX+T3_VectX], [T3_posY, T3_posY+T3_VectY], 'b-')
plot([T1_posX], [T1_posY], 'b.')
plot([T2_posX], [T2_posY], 'b.')
plot([T3_posX], [T3_posY], 'b.')

M1 = plot([T1_posX, T1_posX], [T1_posY, T1_posY], 'r-');
M2 = plot([T2_posX, T2_posX], [T2_posY, T2_posY], 'r-');
M3 = plot([T3_posX, T3_posX], [T3_posY, T3_posY], 'r-');
[axes, buttons, povs] = read(h.joy);

loop = 0;
lastswitch = 0;
%joystick parameters
xythresh = .05;
zthresh = .4;
torquethresh = .1;

LoopTime = .1;
tic

time = 2;
iter = time/LoopTime;

a = [zeros(1, iter*3), 2*ones(1, iter), zeros(1, iter*3), -2*ones(1, iter), zeros(1, iter*3), 4*ones(1, iter/2), zeros(1, iter*3), -4*ones(1, iter/2), zeros(1, iter*7)];

for i = a
    time = toc;
    ZForceRequested = i;
    TorqueRequested = 0;
    ForceRequested = [0 0];
    
    %Force
    Fdesired = [ForceRequested, TorqueRequested]';
    
    %% send to arduino
    SendingData = round([Fdesired; ZForceRequested]*1000);
    dataStr = ['C' num2str(mode) ',' num2str(SendingData(1)) ',' num2str(SendingData(2)) ',' num2str(SendingData(3)) ',' num2str(SendingData(4)) ';\n']
   
    if(ArduinoPresent)
        fprintf(arduino, dataStr);
        sendingLog(end+1,:) = [mode, SendingData'];
    end
    dataStr = [];
    
    %% Reciving Compass
    if(ArduinoPresent && CompassEnabled)
        if(arduino.BytesAvailable ~= 0)
            RecivedDataStr = fgetl(arduino);
            Telem = 0;
            %
            RecivedDataStr
            [Telem, count] = sscanf(RecivedDataStr,'%i,%i,%i,%i,%i,%i,%i,%i,%i,%i;');
            % Batt, Yaw, Roll, Pitch, Depth, Ax, Ay, Az, YawSet, DepthSet
                        
            if length(Telem) >= 10 && Telem(1) ~= -1
                if PromptSaving
                    receivingLog(end+1,:) = Telem';
                end
                if(mode == 2)
                    disp(sprintf(['\nBatt: ' num2str(Telem(1)) ...
                        '\nYaw = ' num2str(Telem(2)/100 + 180) ...
                        '\nYaw Setpoint = ' num2str(Telem(9)/10) ...
                        '\n\nDepth: ' num2str(Telem(5)/100)...
                        '\nDepth Setpoint: ' num2str(Telem(10)/100)...
                        '\n\nYaw = ' num2str(Telem(2)/100 +180) ...
                        '\nPitch = ' num2str(Telem(4)/100) ...
                        '\nRoll = ' num2str(Telem(3)/100) ...
                        '\n\nAx: ' num2str(Telem(6)/1000)...
                        '\nAy: ' num2str(Telem(7)/1000)...
                        '\nAz: ' num2str(Telem(8)/1000)...
                        ]));
                else
                   disp(sprintf(['\nBatt: ' num2str(Telem(1)) ...
                        '\nYaw = ' num2str(Telem(2)/100 + 180) ...
                        '\nPitch = ' num2str(Telem(4)/100) ...
                        '\nRoll = ' num2str(Telem(3)/100) ...
                        '\nDepth: ' num2str(Telem(5)/100)...
                        '\n\nAx: ' num2str(Telem(6)/1000)...
                        '\nAy: ' num2str(Telem(7)/1000)...
                        '\nAz: ' num2str(Telem(8)/1000)...
                        ]));
                end
                
                if Telem(1) < 870 %830
                    disp('LOW BATTERY');
                end
            end
        end

    end
    

    while toc-time < LoopTime
        pause(.0001);
    end
    
    loop = loop + 1;
end

close(h.joy);
disp('Kill');
if ArduinoPresent
    fprintf(arduino,'C1,0,0,0,0;');
    fclose(arduino);
    if PromptSaving == 1
        YesSave = questdlg('Save Logs?');
        if (length(YesSave) == 3)
        clock = clock; 
        save(['Log_' num2str(clock(2)) '-' num2str(clock(3)) '_' num2str(clock(4))...
        '-' num2str(clock(5)) '-' num2str(clock(6)) '.mat']);
        end
    end
  
end

close all;