%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal
 
%C:\Users\Delas\Documents\MATLAB\Shimmer
%C:\Users\Delas\Desktop\Martynas\matlab\Oscillatory-Motion-Tracking-With-x-IMU-master

%% Read from file
fileData1 = tdfread('C:\Users\Delas\Desktop\Martynas\matlab\testu_data\20220120\testas1.dat','\t');
%fileData2 = tdfread('C:\Users\Delas\Desktop\Martynas\matlab\testu_data\20220116\testP008.dat','\t');

%% Import data

    acc2 =    [fileData1.Accelerometer_X fileData1.Accelerometer_Y fileData1.Accelerometer_Z];
    gyr2 =    [fileData1.Gyroscope_X fileData1.Gyroscope_Y fileData1.Gyroscope_Z];
    %acc_dd = [fileData2.Accelerometer_X fileData2.Accelerometer_Y fileData2.Accelerometer_Z];
    %gyr_dd = [fileData2.Gyroscope_X fileData2.Gyroscope_Y fileData2.Gyroscope_Z];
%%
samplePeriod = 1/256;
%if length(fileData1.Accelerometer_X) > length(fileData2.Accelerometer_X)
%    acc2 = acc2(1:length(fileData2.Accelerometer_X),:);
%    gyr2 = gyr2(1:length(fileData2.Accelerometer_X),:);
%else 
%    acc_dd = acc2(1:length(fileData1.Accelerometer_X),:);
%    gyr_dd = gyr2(1:length(fileData1.Accelerometer_X),:);
%end   
% Plot
figure('NumberTitle', 'off', 'Name', 'Gyroscope');
hold on;
plot(gyr2(:,1), 'r');
plot(gyr2(:,2), 'g');
plot(gyr2(:,3), 'b');
xlabel('sample');
ylabel('dps');
title('Gyroscope');
legend('X', 'Y', 'Z');

figure('NumberTitle', 'off', 'Name', 'Accelerometer');
hold on;
plot(acc2(:,1), 'r');
plot(acc2(:,2), 'g');
plot(acc2(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Accelerometer');
legend('X', 'Y', 'Z');

%% Process data through AHRS algorithm (calcualte orientation)
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = zeros(3,3,length(gyr2));     % rotation matrix describing sensor relative to Earth

ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);

for i = 1:length(gyr2)
    ahrs.UpdateIMU(gyr2(i,:) * (pi/180), acc2(i,:));	% gyroscope units must be radians
    R(:,:,i) = quatern2rotMat(ahrs.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
end

%% Calculate 'tilt-compensated' accelerometer

tcAcc = zeros(size(acc2));  % accelerometer in Earth frame

for i = 1:length(acc2)
    tcAcc(i,:) = R(:,:,i) * acc2(i,:)';
end

% Plot
figure('NumberTitle', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
hold on;
plot(tcAcc(:,1), 'r');
plot(tcAcc(:,2), 'g');
plot(tcAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('''Tilt-compensated'' accelerometer');
legend('X', 'Y', 'Z');

%% Calculate linear acceleration in Earth frame (subtracting gravity)

linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1)];
linAcc = linAcc * 9.81;     % convert from 'g' to m/s/s

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Acceleration');
hold on;
plot(linAcc(:,1), 'r');
plot(linAcc(:,2), 'g');
plot(linAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration');
legend('X', 'Y', 'Z');

%% Calculate linear velocity (integrate acceleartion)

linVel = zeros(size(linAcc));

for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
end

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Velocity');
hold on;
plot(linVel(:,1), 'r');
plot(linVel(:,2), 'g');
plot(linVel(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear velocity');
legend('X', 'Y', 'Z');

%% High-pass filter linear velocity to remove drift

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);

% Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Velocity');
hold on;
plot(linVelHP(:,1), 'r');
plot(linVelHP(:,2), 'g');
plot(linVelHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear velocity');
legend('X', 'Y', 'Z');

%% Calculate linear position (integrate velocity)

linPos = zeros(size(linVelHP));

for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Position');
hold on;
plot(linPos(:,1), 'r');
plot(linPos(:,2), 'g');
plot(linPos(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear position');
legend('X', 'Y', 'Z');

%% High-pass filter linear position to remove drift

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

% Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Position');
hold on;
plot(linPosHP(:,1), 'r');
plot(linPosHP(:,2), 'g');
plot(linPosHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear position');
legend('X', 'Y', 'Z');

%% 3D pozicija erdveje
figure('NumberTitle', 'off', 'Name', '3D position')
plot3(linPosHP(:,1),linPosHP(:,2),linPosHP(:,3))
hold on
plot3(linPosHP_dd(:,1),linPosHP_dd(:,2),linPosHP_dd(:,3),'r')
%legend('i≈?orinis', 'vidinis');
%title('tiesine pozicija erdveje')
%% Play animation

SamplePlotFreq = 8;

SixDOFanimation(linPosHP, R, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));            
 
%% End of script