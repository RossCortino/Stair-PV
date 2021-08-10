clearvars -except Streaming Normalized R01 rawR01;
close all
clc;

addpath('Utility Functions')
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


%peak detection parameters
%threshold and min distance
thresh = .000001;
minpeakh_max = 50;
minpeakh_min = -20;
minpeakd = 50;

xpeak = [];
ypeak = [];

xtrough = [];
ytrough = [];

temp_traj_MHF = [];
temp_traj_MHE = [];
p = 1;
mhf = 0;
mhe = 0;


for i = 1:length(hipAngleAscent)
    %function [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh, thighd, vGRF, qh_min, qh_max,qpo, c, prevState,prevPV, sm, qhm,mhf,FC_thresh)
    
    if prevState == 3 || prevState ==4
        if thigh(i) > 30 %&& i-p > minpeakd
            temp_traj_MHF = [temp_traj_MHF thigh(i)];
            if length(temp_traj_MHF) > 3
                temp_traj_MHF;
                [pks,locs] = findpeaks(temp_traj_MHF,'threshold', thresh,'MinPeakHeight',minpeakh_max);
                if ~isempty(pks)
                    p = i-1;
                    temp_traj_MHF = [];
                    ypeak = [ypeak pks];
                    xpeak = [xpeak p];
                    temp_max = qh_max;
                    qh_max = mean([ypeak max(thigh)]);
                    mhf = 1;
                    
                end
            end
            
        end
    end
    
    
        
        
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh(i), thighd(i), vGRF(i),qh_min, qh_max, qpo, c, prevState,prevPV, sm, qhm,mhf,FC_thresh);
    pv(i) = currPV;
    state(i) = currState;
    prevPV = currPV;
    prevState = currState;
end

c = t(find(thigh == qh_min));
prevState = 1;
prevPV = 0;
sm = .63;
qhm = qh_min;
mhf = 0;
FC_thresh = 10;
qpo = 14;
pv_mhe =[];
state_mhe = [];


%peak detection parameters
%threshold and min distance
thresh = .000001;
minpeakh_max = 50;
minpeakh_min = -20;
minpeakd = 50;

xpeak = [];
ypeak = [];

xtrough = [];
ytrough = [];
thigh_start_mhe = 40;
thigh_start_mhf = 30;

temp_traj_MHF = [];
temp_traj_MHE = [];
p = 1;
mhf = 0;
mhe = 0;

for i = 1:length(hipAngleAscent)
    if prevState == 2
        if -thigh(i) > -thigh_start_mhe
            temp_traj_MHE = [temp_traj_MHE -thigh(i)];
            if length(temp_traj_MHE) > 3
                temp_traj_MHF;
                [pks,locs] = findpeaks(temp_traj_MHE,'threshold', thresh,'MinPeakHeight',minpeakh_min);
                if ~isempty(pks)
                    temp_traj_MHE = [];
                    ytrough = [ytrough pks];
                    xtrough = [xtrough locs];
                    mhe = 1;
                  
                end
            end
            
        end
    end
    
    if prevState == 3 || prevState ==4
        if thigh(i) > thigh_start_mhf %&& i-p > minpeakd
            temp_traj_MHF = [temp_traj_MHF thigh(i)];
            if length(temp_traj_MHF) > 3
                temp_traj_MHF;
                [pks,locs] = findpeaks(temp_traj_MHF,'threshold', thresh,'MinPeakHeight',minpeakh_max);
                if ~isempty(pks)
                    p = i-1;
                    temp_traj_MHF = [];
                    ypeak = [ypeak pks];
                    xpeak = [xpeak p];
                    temp_max = qh_max;
                    qh_max = mean([ypeak max(thigh)]);
                    mhf = 1;
                    
                end
            end
            
        end
    end
    
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair_MHE(thigh(i), thighd(i), vGRF(i),qh_min, qh_max, qpo, c, prevState,prevPV, sm, qhm,mhf,mhe,FC_thresh);
    
    pv_mhe(i) = currPV;
    state_mhe(i) = currState;
    prevPV = currPV;
    prevState = currState;
end
    
    figure
    subplot(311)
    plot(gc,thigh,'linewidth',2)
    hold on
    plot(gc(find(thigh == qh_max)),qh_max,'*')
    plot(gc(find(thigh == -ytrough)),-ytrough,'*')    
    ylabel('Thigh Angle (^o)')
    legend('Measured','MHF','MHE')
    
    subplot(312)
    yyaxis left
    plot(gc,pv,'linewidth',2)
    hold on
    plot(gc,pv_mhe,'color','black','linewidth',2)
    ylabel('Phase')
    
    yyaxis right
    ylabel('State')
    plot(gc,state, 'linewidth',2)
    plot(gc, state_mhe,'color','black','LineWidth',2)
    legend('Original','w/MHE','State(Orig)','State(MHE)')
    
    
    subplot(313)
    plot(gc,vGRF,'linewidth',2)
    hold on
    plot(gc,FC_thresh*ones(1,length(gc)),'--','linewidth',2)
    ylabel('Ground Reaction Force (N)')
    xlabel('Gait Cycle (%)')
    legend('Measured','FC Threshold')