clearvars -except Streaming Normalized R01 rawR01;
close all
clc;

load '../Data/dataset_Riener.mat'

load hipKinematicsAscent.mat
load hipKineticsAscent.mat



hipAngle= hipKinematics(:,2);
hipTorque = hipKinetics(:,2);


hipAngleAscent = interp1(1:length(hipAngle), hipAngle, 1:length(hipAngle)/1012:length(hipAngle))';
hipTorqueAscent = interp1(1:length(hipTorque), hipTorque, 1:length(hipTorque)/1008:length(hipTorque))';
time = riener.stairAscent.knee.time;


riener.stairAscent.hip.position = hipAngleAscent;
riener.stairAscent.hip.torque = hipTorqueAscent;
riener.stairAscent.hip.time = time;


load hipKinematicsDescent.mat
load hipKineticsDescent.mat

hipAngle= hipKinematicsDescent(:,2);
hipTorque = hipKineticsDescent(:,2);


hipAngleDescent = interp1(1:length(hipAngle), hipAngle, 1:length(hipAngle)/1010:length(hipAngle))';
hipTorqueDescent = interp1(1:length(hipTorque), hipTorque, 1:length(hipTorque)/1009:length(hipTorque))';

riener.stairDescent.hip.position = hipAngleDescent;
riener.stairDescent.hip.torque = hipTorqueDescent;
riener.stairDescent.hip.time = time;



load vGRFAscent.mat


vGRF = vGRFAscent(:,2);

vGRF= interp1(1:length(vGRF), vGRF, 1:length(vGRF)/639:length(vGRF));

vGRF = [vGRF vGRF(end)*ones(1,1000-length(vGRF))];
time = riener.stairAscent.knee.time;
figure
plot(vGRF)

riener.stairAscent.GRF.vertical = vGRF';
riener.stairAscent.GRF.time = time;

load vGRFDescent.mat


vGRF = vGRFDescent(:,2);

vGRF= interp1(1:length(vGRF), vGRF, 1:length(vGRF)/639:length(vGRF));

vGRF = [vGRF vGRF(end)*ones(1,1000-length(vGRF))];
time = riener.stairDescent.knee.time;
figure
plot(vGRF)

riener.stairDescent.GRF.vertical = vGRF';
riener.stairDescent.GRF.time = time;

save ../Data/dataset_Riener.mat riener