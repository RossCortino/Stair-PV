clearvars -except Streaming Normalized R01 rawR01;
close all
clc;

load '../Data/dataset_Riener.mat'

load ankleKineticsDescent.mat
load ankleKinematicsDescent.mat
tf = 1.41;

time = linspace(0,tf,1000)
gc = linspace(0,100,1000);

ankle_joint = smooth(interp1(ankleKinematicsDescent(:,1),ankleKinematicsDescent(:,2),gc));
ankle_torque = smooth(interp1(ankleKineticsDescent(:,1),ankleKineticsDescent(:,2),gc));


mode(diff(time))

filt_freq = 15 ; %Hz
samprate = 1/mode(diff(time)); % Hz
[B_f,A_f] = butter(2,filt_freq/(samprate/2)) ; 
filt_joint = filtfilt(B_f,A_f,ankle_joint);

figure
plot(gc,ankle_joint,'linewidth',3)
hold on
plot(gc,filt_joint,'linewidth',2)

filt_freq = 10; %Hz
samprate = 1/mode(diff(time)); % Hz
[B_f,A_f] = butter(2,filt_freq/(samprate/2)) ; 
filt_torque = filtfilt(B_f,A_f,ankle_torque);

figure
plot(gc,ankle_torque,'linewidth',3)
hold on
plot(gc,filt_torque,'linewidth',2)


riener.stairDescent.ankle.time = time';
riener.stairDescent.ankle.position = filt_joint;
riener.stairDescent.ankle.torque = filt_torque;
save ../Data/dataset_Riener.mat riener