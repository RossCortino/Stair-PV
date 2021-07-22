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

thigh = hipAngleAscent;
thighd = ddt(hipAngleAscent);
qh_min = min(thigh);
qh_max = max(thigh);

t = linspace(0,1,length(thigh));
gc = t*100;


c = t(find(thigh == qh_min));
prevState = 1;
prevPV = 0;
sm = .63;
qhm = qh_min;
mhf = 0;
FC_thresh = 10;
qpo = 14;
pv =[];
state = [];



for i = 1:length(hipAngleAscent)
    %function [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh, thighd, vGRF, qh_min, qh_max,qpo, c, prevState,prevPV, sm, qhm,mhf,FC_thresh)
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh(i), thighd(i), vGRF(i),qh_min, qh_max, qpo, c, prevState,prevPV, sm, qhm,mhf,FC_thresh);
    if currState == 3 || currState ==4
       if thigh(i) == qh_max
           mhf = 1;
       end
    end
    
    pv(i) = currPV;
    state(i) = currState;
    prevPV = currPV;
    prevState = currState;
end


figure
subplot(311)
plot(gc,thigh,'linewidth',2)
hold on
plot(gc(find(thigh == qh_max)),qh_max,'*')
ylabel('Thigh Angle (^o)')
legend('Measured','MHF')

subplot(312)
yyaxis left
plot(gc,pv,'linewidth',2)
ylabel('Phase')
hold on
yyaxis right
ylabel('State')
plot(gc,state,'linewidth',2)
legend('Phase Variable','State')


subplot(313)
plot(gc,vGRF,'linewidth',2)
hold on
plot(gc,FC_thresh*ones(1,length(gc)),'--','linewidth',2)
ylabel('Ground Reaction Force (N)')
xlabel('Gait Cycle (%)')
legend('Measured','FC Threshold')