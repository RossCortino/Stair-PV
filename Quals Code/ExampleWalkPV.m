clearvars -except Streaming Normalized stairAscentTrialData R01 rawR01 estimatedAngles;

close all
addpath("Utility Functions")

if ~exist('Normalized')
    load '../Data/Normalized.mat'
end
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s1'};
incline={'i0'};
[thigh_0, knee_0, ankle_0, thigh_0sd, knee_0sd, ankle_0sd] = averageJointKinematics_Walk(Normalized,sub,trial,incline);



t = linspace(0,1, length(thigh_0));
thigh_0d = ddt(thigh_0);

pv_swing_thresh = .65;
c = t(find(thigh_0 == min(thigh_0)));
q_po = -10;
prevState = 1;
prevPV = 0;

qh_max = thigh_0(1);
qh_min = min(thigh_0);
mhf = 0;
pv = [];
sm = 0;
qhm = 0;
gc = linspace(0,100,length(t));
for i = 1:length(t)
    
    thigh = thigh_0(i);
    thighd = thigh_0d(i);
    
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Walk(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf,pv_swing_thresh)
    pv(i) = currPV;
    
    if prevState == 1 && currState == 2
        s2 = gc(i);
    elseif prevState == 2 && currState == 3
        s3 = gc(i)
    elseif prevState == 3 && currState == 4
        s4 = gc(i)
    end
    prevState = currState;
    prevPV = currPV;
    
    
end

gc = linspace(0,100,length(t));
figure

plot(gc, thigh_0,'linewidth',2)
hold on
% xline(s2,'k--','linewidth',2)
xline(s3,'k--','linewidth',2)
% xline(s4,'k--','linewidth',2)
set(gca,'FontSize',15)
xlabel('Gait Cycle (%)')
ylabel('Thigh Angle (^o)')
grid on


figure

plot(gc, pv,'linewidth',2)
set(gca,'FontSize',15)
xlabel('Gait Cycle (%)')
ylabel('Phase')
grid on


figure
plot(gc, linspace(0,1, length(gc)),'linewidth',2)
set(gca,'FontSize',15)
xlabel('Gait Cycle (%)')
ylabel('Phase')
grid on
