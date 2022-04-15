%% Start of script

addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
two_files = 1; %1 = two IMUs; 0 = one IMU;

% Read from file
fileData1 = tdfread('testu_data\20220412_MD\7sh180dilb1_alkune_virsun.dat','\t');%3sh180dilb1_sonas
%fileData1 = tdfread('D:\VGTU_magistrantura\2022\MBD3\matlab\testu_data\20220405_3_kryptys\6sh180dilb1_sonu.dat','\t');
    Accelerometer = [fileData1.Accelerometer_X fileData1.Accelerometer_Y fileData1.Accelerometer_Z];
    Gyroscope     = [fileData1.Gyroscope_X fileData1.Gyroscope_Y fileData1.Gyroscope_Z];
    Magnetometer  = [fileData1.Magnetometer_X fileData1.Magnetometer_Y fileData1.Magnetometer_Z];

    
    % Import ArduinoUNO data
    %fileData2 = tdfread(['D:\VGTU_magistrantura\2022\MBD3\matlab\testDilbis1.dat'],',');%1sh180dilb1
    % Import data
    %Accelerometer = [fileData2.accReadings_1 fileData2.accReadings_2 fileData2.accReadings_3];
    %Gyroscope     = [fileData2.gyrReadings_1*50 fileData2.gyrReadings_2*20 fileData2.gyrReadings_3*20];
    %Magnetometer  = [fileData2.magReadings_2/15 -fileData2.magReadings_1 fileData2.magReadings_3]/32;

    %Accelerometer = Accelerometer* -1;
    %Gyroscope = Gyroscope* -1;
    %Magnetometer = Magnetometer* -1;
    Accelerometer = Accelerometer/(9.81);%9.81; %%m/s2 to g
    samplePeriod = 1/256;






if two_files == 1
    fileData2 = tdfread('testu_data\20220412_MD\6sh180zast1_alkune_virsun.dat','\t');%3sh180dilb1_sonas
    Accelerometer2 = [fileData2.Accelerometer_X fileData2.Accelerometer_Y fileData2.Accelerometer_Z];
    Gyroscope2     = [fileData2.Gyroscope_X fileData2.Gyroscope_Y fileData2.Gyroscope_Z];
    Magnetometer2  = [fileData2.Magnetometer_X fileData2.Magnetometer_Y fileData2.Magnetometer_Z];


    Accelerometer2 = Accelerometer2/(9.81);%9.81; %%m/s2 to g
    time2 = 1:length(Accelerometer2);


    %sulyginam pirmojo ir antrojo IMU duomenu ilgi
    Accelerometer = Accelerometer(7:10285,:);
    Gyroscope = Gyroscope(7:10285,:);
    Magnetometer = Magnetometer(7:10285,:);
end

    time = 1:length(Accelerometer);


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

    % Plot gyr
    figure('NumberTitle', 'off', 'Name', 'Gyroscope');
    hold on;
    plot(Gyroscope(:,1), 'r');
    plot(Gyroscope(:,2), 'g');
    plot(Gyroscope(:,3), 'b');
    xlabel('sample');
    ylabel('dps');
    title('Gyroscope');
    legend('X', 'Y', 'Z');


% Plot sensor data
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

InitialPos = [1 0 0 0];
% Process sensor data through algorithm

AHRS = MadgwickAHRS('SamplePeriod', samplePeriod, 'Quaternion', InitialPos,  'Beta', 0.3);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

R = zeros(3,3,length(time));     % rotation matrix describing sensor relative to Earth
quaternio = zeros(length(time), 4);
%quaternion(:,2) = 1;

for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternio(t, :) = AHRS.Quaternion;
    R(:,:,t) = quatern2rotMat(AHRS.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
end

% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternio)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

%nustatoma orientacija antrojo imu
if two_files == 1
    AHRS = MadgwickAHRS('SamplePeriod', samplePeriod, 'Quaternion', InitialPos,  'Beta', 0.3);
    % AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);
    
    R2 = zeros(3,3,length(time2));     % rotation matrix describing sensor relative to Earth
    quaternio2 = zeros(length(time2), 4);
    %quaternion(:,2) = 1;
    
    for t = 1:length(time2)
        AHRS.Update(Gyroscope2(t,:) * (pi/180), Accelerometer2(t,:), Magnetometer2(t,:));	% gyroscope units must be radians
        quaternio2(t, :) = AHRS.Quaternion;
        R2(:,:,t) = quatern2rotMat(AHRS.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
    end

    euler2 = quatern2euler(quaternConj(quaternio2)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
    
    
    figure('Name', 'Euler Angles 2nd IMU');
    hold on;
    plot(time2, euler2(:,1), 'r');
    plot(time2, euler2(:,2), 'g');
    plot(time2, euler2(:,3), 'b');
    title('Euler angles 2 IMU');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    legend('X roll \phi', 'Y pitch \theta', 'Z yaw \psi');
    hold off;
end


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

%%
%nustatomas vidurkis ramybes busenoje
%q_start = mean(quaternio(1:(inv(samplePeriod)*5),:));
q_start = mean(quaternio(1800:2000,:));%1024:153
%normalizuojamos reiksmes
q_start = quatnormalize(q_start);

%fiksuota pozicija pagal kuria atsukame jutiklius is ECI orientacijos
q_start_op = [0.967049105922898,0.036397648284591,-0.063846117754861,-0.243751740877285];
%q_start3 =  quatmultiply(quatinv(q_start),q_start2);

%signalas atsukamas pagal zemes orientacija
Q_eci = quatmultiply(quaternio,quatinv(q_start));

%signalas atsukamas pagal orientacini IMU
Q_rot = quatmultiply(Q_eci,q_start_op);

%atvaizdavimui 
euler_rot = quatern2euler(quaternConj(Q_rot)) * (180/pi);




%% Antro IMU kalibravimas
%nustatomas vidurkis ramybes busenoje
if two_files == 1
    q_start2 = mean(quaternio2(1800:2000,:));%1024:153
    % normalizuojamos reiksmes
    q_start2 = quatnormalize(q_start2);
        
    %signalas atsukamas pagal zemes orientacija
    Q_eci2 = quatmultiply(quaternio2,quatinv(q_start2));
    
    %signalas atsukamas pagal orientacini IMU
    Q_rot2 = quatmultiply(Q_eci2,q_start_op);
    
    %atvaizdavimui 
    euler_rot2 = quatern2euler(quaternConj(Q_rot2)) * (180/pi);
        
    % kampas tarp pirmojo in antrojo imu
    q12_eci = quatmultiply(Q_eci,quatinv(Q_eci2));
    q12_rot = quatmultiply(Q_rot,quatinv(Q_rot2));

    %pakeiciamas duomenu tipas paprastesniam atvaizdavimui
    quat2 = quaternion(quaternio2);
    Q_eci2 = quaternion(Q_eci2);
    Q_rot2 = quaternion(Q_rot2);
    q12_eci = quaternion(q12_eci);
    q12_rot = quaternion(q12_rot);
end

%PIRMAS IMU pakeiciamas duomenu tipas paprastesniam atvaizdavimui
quat = quaternion(quaternio);
Q_eci = quaternion(Q_eci);
Q_rot = quaternion(Q_rot);

%atvaizdavimas
figure
subplot(3, 1, 1)
plot(time, eulerd(quat, 'ZYX', 'frame'), '--')
title('Logged Euler Angles')
ylabel('\circ') % Degrees symbol.
legend('z-axis', 'y-axis', 'x-axis')
subplot(3, 1, 2)
plot(time, eulerd(Q_eci, 'ZYX', 'frame'))
title('ECI Euler Angles')
ylabel('\circ') % Degrees symbol.
legend('z-axis', 'y-axis', 'x-axis')
subplot(3, 1, 3)
plot(time, eulerd(Q_rot, 'ZYX', 'frame'))
title('Aligned Euler Angles')
ylabel('\circ') % Degrees symbol.
legend('z-axis', 'y-axis', 'x-axis')


if two_files == 1

    %ANTRAS IMU pakeiciamas duomenu tipas paprastesniam atvaizdavimui
    quat = quaternion(quaternio);
    Q_eci = quaternion(Q_eci);
    Q_rot = quaternion(Q_rot);

    %atvaizdavimas pagal statini ECI
    figure
    subplot(3, 1, 1)
    plot(time, eulerd(Q_eci, 'ZYX', 'frame'), '--')
    title('First IMU ECI')
    ylabel('\circ') % Degrees symbol.
    legend('z-axis', 'y-axis', 'x-axis')
    subplot(3, 1, 2)
    plot(time, eulerd(Q_eci2, 'ZYX', 'frame'))
    title('Second IMU ECI')
    ylabel('\circ') % Degrees symbol.
    legend('z-axis', 'y-axis', 'x-axis')
    subplot(3, 1, 3)
    plot(time, eulerd(q12_eci, 'ZYX', 'frame'))
    title('Angle between IMU ECI')
    ylabel('\circ') % Degrees symbol.
    legend('z-axis', 'y-axis', 'x-axis')
    
    %atvaizdavimas pagal statini IMU
    figure
    subplot(3, 1, 1)
    plot(time, eulerd(Q_rot, 'ZYX', 'frame'), '--')
    title('First IMU')
    ylabel('\circ') % Degrees symbol.
    legend('z-axis', 'y-axis', 'x-axis')
    subplot(3, 1, 2)
    plot(time, eulerd(Q_rot2, 'ZYX', 'frame'))
    title('Second IMU')
    ylabel('\circ') % Degrees symbol.
    legend('z-axis', 'y-axis', 'x-axis')
    subplot(3, 1, 3)
    plot(time, eulerd(q12_rot, 'ZYX', 'frame'))
    title('Angle between IMU')
    ylabel('\circ') % Degrees symbol.
    legend('z-axis', 'y-axis', 'x-axis')
end


%% Signalo skaidymas i pavienius judesius
[pks,locs] = findpeaks(euler_rot(:,1),time,'MinPeakProminence',7.5);
[pks_n,locs_n] = findpeaks(-euler_rot(:,1),time,'MinPeakProminence',7.5);
pks_n = -1*pks_n;

figure()
hold on
plot(euler_rot(:,1))
plot(locs,pks,'vr',locs_n,pks_n,'^r')
hold off

%signalo atkapu masyvas
if length(locs) > length(locs_n)
    peaks_le = length(locs_n);
else
    peaks_le = length(locs);
end

arr_mov = zeros(peaks_le,2);

for i = 1:peaks_le
    if locs(1) > locs_n(1)
        arr_mov(i,1) = locs_n(i);
        arr_mov(i,2) = locs(i);
    else
        arr_mov(i,1) = locs(i);
        arr_mov(i,2) = locs_n(i);
    end
end


%% ATSTUMO SKAICIAVIMO ALGORITMAS
% Calculate 'tilt-compensated' accelerometer

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

%% Duomenu karpymas i judesius
arrfrom  = arr_mov(:,1);
arrto  = arr_mov(:,2);
arclen_arr = zeros(length(arr_mov),1);

for i = 1:length(arrfrom)
    from = arrfrom(i);
    to = arrto(i);

    % skaiciuojamas min max tarp eulerio kampu - LAIPSNIAI
    % Flexion/Extension   Internal/External Rotation   Abduction/Adduction
    euler_sample = euler_rot(from:to,:);
    MM(i,:) = [((-min(euler_sample(:,1))) + max(euler_sample(:,1))) ((-min(euler_sample(:,3))) +...
        max(euler_sample(:,3))) ((-min(euler_sample(:,2))) + max(euler_sample(:,2)))];
    
    % karopoma pozicija i judeius, judesio ilgiui apskaiciuoti
    %x = linPosHP(from:to,1)/5; y = linPosHP(from:to,2)/5; z = linPosHP(from:to,3)/5;
    x = linPosHP(from:to,1); y = linPosHP(from:to,2); z = linPosHP(from:to,3); 
    % Line lenght. Judesio ilgis
    [arclen,seglen] = arclength(x,y,z);      % a 3-d space curve
    arclen_arr(i) = arclen;
end




%% 3D pozicija erdveje

figure('NumberTitle', 'off', 'Name', '3D position')
plot3(x,y,z)
grid on;
title('tiesine pozicija erdveje')

%% Play animation
% from = 4818;
% to = length(linPosHP);
linPosHP2 = linPosHP(from:to,:); %dalyba nes algoritmas rodo 5x didesnius duomenis pagal samplerate santyki 256/51.2
    
%apsukamos asys teisingam judesio atvaizdavimui
% linPosHP3 = zeros(size(linPosHP2));
% linPosHP3(:,1) = linPosHP2(:,3);
% linPosHP3(:,2) = linPosHP2(:,2);
% linPosHP3(:,3) = linPosHP2(:,1);

SamplePlotFreq = 8;

SixDOFanimation(linPosHP2, R, ...
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
title('180° judesys');
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
title('180° judesio projekcija');
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
MM2 = [min(Q(:,1)) max(Q(:,1)) min(Q(:,2)) max(Q(:,2)) min(Q(:,3)) max(Q(:,3))];

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

%arclen atstumas
 [arclen2,seglen2] = arclength(Q(:,1),Q(:,2),Q(:,3));  
%% atstumas tarp pirmos ir paskutines judesio reiksmes
firstX = mean(Q(1:5,1));
firstY = mean(Q(1:5,2));
firstZ = mean(Q(1:5,3));

lastX = mean(Q(length(Q)-5:length(Q),1));
lastY = mean(Q(length(Q)-5:length(Q),2));
lastZ = mean(Q(length(Q)-5:length(Q),3));

distance2 = sqrt(sum(cat(3,((lastX - firstX).^ 2), ((lastY - firstY).^ 2), ((lastZ - firstZ).^ 2)),3));
%% Plot thermometer according to distance
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

%% Plot thermometer
miny = min(yy);
maxy =  max(yy);
disty = sqrt(min(yy)^2) + sqrt(max(yy)^2);
disty20proc = disty * 0.2;

%vecy = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1];
vecy = (1+zeros(round(arclen*100),1))';


thermometer([ vecy ], 160, { '10%'},...
    { 141.3 '    Pilnas judesys'})
%% End of script
