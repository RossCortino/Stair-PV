clearvars -except Streaming Normalized R01 rawR01;
close all
clc;


load ../Data/dataset_Riener.mat

mass = 76.7;

hipAngleAscent = riener.stairAscent.hip.position';
hipTorqueAscent = riener.stairAscent.hip.torque'*mass;

kneeAngleAscent = riener.stairAscent.knee.position';
kneeTorqueAscent = riener.stairAscent.knee.torque'*mass;

ankleAngleAscent = riener.stairAscent.ankle.position';
ankleTorqueAscent = riener.stairAscent.ankle.torque'*mass;

vGRF = riener.stairAscent.GRF.vertical'*mass;



gc = linspace(0,100,length(vGRF));

% figure
% 
% plot(gc,smooth(hipAngleAscent,100),'linewidth',2)
% hold on
% plot(gc,hipAngleAscent,'--','linewidth',2)

figure
title('Stair Ascent Kinematics')
plot(gc,hipAngleAscent)
hold on
plot(gc,kneeAngleAscent)
plot(gc,ankleAngleAscent)

legend('Hip','Knee','Ankle')
xlabel('Gait Cycle (%)')
ylabel('Joint Angle (^o)')


figure
title('Stair Ascent Kinetics')
plot(gc,hipTorqueAscent)
hold on
plot(gc,kneeTorqueAscent)
plot(gc,ankleTorqueAscent)

legend('Hip','Knee','Ankle')
xlabel('Gait Cycle (%)')
ylabel('Moment (Nm)')

figure
title('Stair Ascent GRF')
plot(gc,vGRF)
xlabel('Gait Cycle (%)')
ylabel('Moment (N)')
