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


%peak detection parameters
%threshold and min distance
thresh = .000001;
minpeakh = 50;
minpeakd = 50;

xpeak = [];
ypeak = [];
temp_traj_MHF = [];
p = 1;
mhf = 0;


for i = 1:length(hipAngleAscent)
    %function [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh, thighd, vGRF, qh_min, qh_max,qpo, c, prevState,prevPV, sm, qhm,mhf,FC_thresh)
    
    if prevState == 3 || prevState == 4
       if thigh(i) > 20 %&& i-p > minpeakd
            temp_traj_MHF = [temp_traj_MHF thigh(i)];
            if length(temp_traj_MHF) > 3
                temp_traj_MHF;
                [pks,locs] = findpeaks(temp_traj_MHF,'threshold', thresh,'MinPeakHeight',minpeakh);
                if ~isempty(pks)
                    p = i-1;
                    temp_traj_MHF = [];
                    ypeak = [ypeak pks];
                    xpeak = [xpeak p];
                    temp_max = qh_max;
                    qh_max = mean([ypeak max(thigh)]);
                    mhf = 1;
                    
                    %                 plot(1:i-1,pv)
                    %                 ylabel('phasevariable')
                    %                 title('PV Streaming')
                    %                 hold on
                end
            end

        end
    end
%     if i == 512 || i == 513
%         keyboard
%     end
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_StairV2(thigh(i), thighd(i), vGRF(i),qh_min, qh_max, qpo, c, prevState,prevPV, sm, qhm,mhf,FC_thresh);
    
    if i == 512 || i == 513
        disp(currState)
        pvDescending = (qh_max-thigh(i))/(qh_max-qh_min)*c
        pvAscending = 1 + (1-sm)*(thigh(i)-qh_max)/(qh_max-qhm)
    end
    pv(i) = currPV;
    state(i) = currState;
    prevPV = currPV;
    prevState = currState;
end

[v, w] = unique( pv, 'stable' );
duplicate_indices = setdiff( 1:numel(pv), w )
figure
plot(gc,thigh,'linewidth',2)
hold on
plot(gc(duplicate_indices),thigh(duplicate_indices),'*')
% plot(gc(1000-find(hist(pv,unique(pv))>1)),thigh(1000-find(hist(pv,unique(pv))>1)),'*')
% pvDescending = (qh_max-thigh(find(hist(pv,unique(pv))>1)))/(qh_max-qh_min)*c
% pvAscending = 1 + (1-sm)*(thigh(find(hist(pv,unique(pv))>1))-qh_max)/(qh_max-qhm)
 ylabel('Thigh Angle (^o)')
 yyaxis right
ylabel('State')
plot(gc,state,'linewidth',2)
legend('Measured','Repeat Index','State')

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



% figure
% subplot(311)
% plot(gc,thigh,'linewidth',2)
% hold on
% plot(gc(find(thigh == qh_max)),qh_max,'*')
% ylabel('Thigh Angle (^o)')
% legend('Measured','MHF')
% 
% subplot(312)
% yyaxis left
% plot(gc,pv,'linewidth',2)
% ylabel('Phase')
% hold on
% yyaxis right
% ylabel('State')
% plot(gc,state,'linewidth',2)
% legend('Phase Variable','State')
% 
% subplot(313)
% plot(gc,ddt(pv),'linewidth',2)
% hold on
% plot(gc, zeros(length(gc)))
% ylabel('Phase Velocity')
% ylim([-.005, .005])
% xlim([0, 100]) 
% legend('Phase Vel','0 Vel thresh')


% [C,ia] = unique(pv,'stable');
% 
% ia
% pv(ia)

% find(hist(pv,unique(pv,'stable'))>1)
% find(hist(fliplr(pv),unique(fliplr(pv),'stable'))>1)


[v, w] = unique( pv, 'stable' );
duplicate_indices = setdiff( 1:numel(pv), w )
% figure
% find(hist(pv,unique(pv))>1)