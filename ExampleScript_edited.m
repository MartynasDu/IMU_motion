% ExampleScript.m
%
% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with example data. ExampleData.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches ±90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected
%% Start of script

addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%pilnas1.dat

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% Read from file
fileData1 = tdfread(['D:\VGTU_magistrantura\2022\MBD3\matlab\testu_data\20220307_51Hz\3sh180dilb1.dat'],'\t');%1sh180dilb1
%fileData2 = tdfread('C:\Users\Delas\Desktop\Martynas\matlab\testu_data\20220116\testP008.dat','\t');

% Import data


    Accelerometer = [fileData1.Accelerometer_X fileData1.Accelerometer_Y fileData1.Accelerometer_Z];
    Gyroscope     = [fileData1.Gyroscope_X fileData1.Gyroscope_Y fileData1.Gyroscope_Z];
    Magnetometer  = [fileData1.Magnetometer_X fileData1.Magnetometer_Y fileData1.Magnetometer_Z];

    %Accelerometer = Accelerometer* -1;
    %Gyroscope = Gyroscope* -1;
    %Magnetometer = Magnetometer* -1;
    Accelerometer = Accelerometer/(9.81);%9.81; %%m/s2 to g
        time = 1:length(Accelerometer);
    samplePeriod = 1/51.2;

    % Plot acc
    figure('NumberTitle', 'off', 'Name', 'Accelerometer');
    hold on;
    plot(Accelerometer(:,1), 'r');
    plot(Accelerometer(:,2), 'g');
    plot(Accelerometer(:,3), 'b');
    xlabel('sample');
    ylabel('g');
    title('Accelerometer');
    legend('X', 'Y', 'Z');
% Import and plot sensor data

%load('ExampleData.mat');

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

% Process sensor data through algorithm

AHRS = MadgwickAHRS('SamplePeriod', samplePeriod, 'Beta', 0.1);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

R = zeros(3,3,length(time));     % rotation matrix describing sensor relative to Earth
quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
    R(:,:,t) = quatern2rotMat(AHRS.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
end

% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('X roll \phi', 'Y pitch \theta', 'Z yaw \psi');
hold off;

%% Flexion/Extension   Internal/External Rotation   Abduction/Adduction
arrfrom  = [896
1199
1512
];

arrto  = [1198;
1511
1862
];

for var = 1:length(arrfrom)
    from = arrfrom(var);
    to = arrto(var);

    MM = [((-min(euler(:,1))) + max(euler(:,1))) ((-min(euler(:,3))) + max(euler(:,3))) ((-min(euler(:,2))) + max(euler(:,2)))];
end
%% Calculate 'tilt-compensated' accelerometer

tcAcc = zeros(size(Accelerometer));  % accelerometer in Earth frame

for i = 1:length(Accelerometer)
    tcAcc(i,:) = R(:,:,i) * Accelerometer(i,:)';
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

% Calculate linear acceleration in Earth frame (subtracting gravity)

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

% Calculate linear velocity (integrate acceleartion)

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

% High-pass filter linear velocity to remove drift

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

% Calculate linear position (integrate velocity)

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

% High-pass filter linear position to remove drift

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

% Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Position');
hold on;
%
%timeaxis = linspace(0,40,2071);

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
%plot3(linPosHP_dd(:,1),linPosHP_dd(:,2),linPosHP_dd(:,3),'r')
%legend('i≈?orinis', 'vidinis');
%title('tiesine pozicija erdveje')

%% Duomenu karpymas i judesius
from = 868; to = 1095; %Dilbis7, 1 judesys
	
x = linPosHP(from:to,1)/5; y = linPosHP(from:to,2)/5; z = linPosHP(from:to,3)/5;
%x = linPosHP(from:to,1); y = linPosHP(from:to,2); z = linPosHP(from:to,3);

%x = linPosHP(980:length(linPosHP),1);
%y = linPosHP(980:length(linPosHP),2);
%z = linPosHP(980:length(linPosHP),3);


%% Play animation
% from = 4818;
% to = length(linPosHP);
linPosHP2 = linPosHP(from:to,:)/5; %dalyba nes algoritmas rodo 5x didesnius duomenis pagal samplerate santyki 256/51.2

%apsukamos asys teisingam judesio atvaizdavimui
linPosHP3 = zeros(size(linPosHP2));
linPosHP3(:,1) = linPosHP2(:,3);
linPosHP3(:,2) = linPosHP2(:,2);
linPosHP3(:,3) = linPosHP2(:,1);
SamplePlotFreq = 8;

SixDOFanimation(linPosHP3, R, ...
               'SamplePlotFreq', SamplePlotFreq, 'Trail', 'DotsOnly', ...
               'Position', [9 39 1280 720], ...
               'AxisLength', 0.02, 'ShowArrowHead', false, ...
               'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Pilnas judesys',...
               'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));            

%% call point data P(x,y,z)coordinate from file "ha.txt"
P = [x y z];
figure()
plot3(P(:,1),P(:,2),P(:,3),'r.');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('180∞ judesys');
grid on; hold on;
%% compute the normal and a center point that belongs to the plane N
[n,~,P0] = affine_fit(P);%Assuming the approximate plane N contain all points by total least square method.
% PLOT PLANE N
%plot the two adjusted planes
xx = linspace(min(P(:,1)),max(P(:,1)),3);
yy = linspace(min(P(:,2)),max(P(:,2)),3);
%zz = linspace(min(P(:,3)),max(P(:,3)),3);
[X,Y] = meshgrid(xx,yy);
%first plane
figure()
Psurf = surf(X,Y, - (n(1)/n(3)*X+n(2)/n(3)*Y-dot(n,P0)/n(3)),'facecolor','red','facealpha',0.5);
grid on; hold on;
%plot the  points P0
%plot3(P0(1),P0(2),P0(3),'ro','markersize',10,'markerfacecolor','red');
%% Projected a point P onto a plane N
dist=(P-P0)*n;% distance from point X to plane.%This is (k,1) matrix
Q=P-dist.*n'; % projected of set data P onto plane N
%figure()
plot3(Q(:,1),Q(:,2),Q(:,3),'b.');%plot all points Q
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('180∞ judesio projekcija');
grid on;
%hold on;
%% Zalias pavirsius
% xx = linspace(min(Q(:,1)),max(Q(:,1)),3);
% yy = linspace(min(Q(:,2)),max(Q(:,2)),3);
% zz = linspace(min(Q(:,3)),max(Q(:,3)),3);
% [X,Y] = meshgrid(xx,yy);
% Qsurf = surf(X,Y, - (n(1)/n(3)*X+n(2)/n(3)*Y-dot(n,P0)/n(3)),'facecolor','green','facealpha',0.6);
 hold off;


%% min/max values
MM = [min(Q(:,1)) max(Q(:,1)) min(Q(:,2)) max(Q(:,2)) min(Q(:,3)) max(Q(:,3))];

[mValue , vIndex] = min(Q(:,3));
minX = mValue;
minY = Q(vIndex,2);
minZ = Q(vIndex,3);

[maxValue , vaxIndex] = max(Q(:,3));
maxX = maxValue;
maxY = Q(vaxIndex,2);
maxZ = Q(vaxIndex,3);

%atstumas, kur x reiksme didziausia
distance = sqrt(sum(cat(3,((maxX - minX).^ 2), ((maxY - minY).^ 2), ((maxZ - minZ).^ 2)),3));
%% atstumas tarp pirmos ir paskutines judesio reiksmes
firstX = mean(Q(1:5,1));
firstY = mean(Q(1:5,2));
firstZ = mean(Q(1:5,3));

lastX = mean(Q(length(Q)-5:length(Q),1));
lastY = mean(Q(length(Q)-5:length(Q),2));
lastZ = mean(Q(length(Q)-5:length(Q),3));

distance2 = sqrt(sum(cat(3,((lastX - firstX).^ 2), ((lastY - firstY).^ 2), ((lastZ - firstZ).^ 2)),3));
%% Plot thermometer
% miny = min(yy);
% maxy =  max(yy);
% disty = sqrt(min(yy)^2) + sqrt(max(yy)^2);
% disty20proc = disty * 0.2;
% 
% %vecy = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1];
% vecy = (0.1+zeros(52,1))';
% 
% thermometer([ vecy ], disty + disty20proc, { '10%'},...
%     { disty 'Pilnas judesys'})
% %%
% %% Plot thermometer
% miny = min(yy);
% maxy =  max(yy);
% disty = sqrt(min(yy)^2) + sqrt(max(yy)^2);
% disty20proc = disty * 0.2;
% 
% %vecy = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1];
% vecy = (0.1+zeros(580))';
% 
% thermometer([ vecy ], 101, { '10%'},...
%     { 100 '    Pilnas judesys'})
%% End of script
