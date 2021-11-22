clearvars -except Streaming Normalized stairAscentTrialData R01 rawR01 estimatedAngles;


close all
addpath("Utility Functions")

if ~exist('Normalized')
    load '../Data/Normalized.mat'
end

incline={'in30'};
trial={'s3'};
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
[thigh_d30, knee_d30, ankle_d30, thigh_d30sd, knee_d30sd, ankle_d30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

thigh_d30_shift = circshift(thigh_d30,-find(thigh_d30 == max(thigh_d30)));

gc = linspace(0,100,length(thigh_d30));
t = linspace(0,1,length(gc));

c_d30 = t(find(thigh_d30_shift == min(thigh_d30_shift)));

thigh_mean = thigh_d30;
knee_mean = knee_d30;
ankle_mean = ankle_d30;
current_incline = '-30^o';
q_po = 7;
pv_swing_thresh = .96;
c= c_d30;

thighd_mean = ddt(thigh_mean);
prevPV = 0;
prevState = 1;
sm = 0;
qhm = 0;
qh_max = max(thigh_mean);
qh_min = min(thigh_mean);
%     c = t(find(thigh_mean== qh_min))


%peak detection parameters
%threshold and min distance
thresh = .00001;
minpeakh = 15;
minpeakd = 50;
mhf = 0;
temp_traj = [];
ypeak = [];
xpeak = [];


%peak detection parameters
%threshold and min distance
thresh = .0000001;
minpeakh_mhe = 6;
minpeakd_mhe = 6;
thigh_start_mhe = 7;
mhe = 0;
temp_traj_MHE = [];
%     pv_swing_thresh = .96;
ytrough = [];
xtrough = [];


state =[]
for i = 1:length(t)
        
        thigh = thigh_mean(i);
        thighd = thighd_mean(i);
        
        
        
        if prevState == 2|| prevState == 3
            temp_traj_MHE = [temp_traj_MHE -thigh];
            if length(temp_traj_MHE) > 3
                [pks_MHE,locs_MHE] = findpeaks(temp_traj_MHE,'threshold', thresh,'MinPeakHeight',-minpeakh_mhe);
                if ~isempty(pks_MHE)
                    temp_traj_MHE = [];
                    ytrough = -[ytrough pks_MHE];
                    xtrough = [xtrough i-1];
                    mhe = 1;
                    
                end
            end
            
            
        end
        if prevState == 3 || prevState == 4
            if thigh > 15 %&& i-p > minpeakd
                temp_traj = [temp_traj thigh];
                if length(temp_traj) > 3
                    temp_traj;
                    [pks,locs] = findpeaks(temp_traj,'threshold', thresh,'MinPeakHeight',minpeakh);
                    if ~isempty(pks)
                        p = i-1;
                        temp_traj = [];
                        ypeak = [ypeak pks];
                        xpeak = [xpeak p];
                        temp_max = qh_max;
                        qh_max = mean([ypeak max(thigh_mean)]);
                        mhf = 1;
                    end
                end
                
            end
        end
        %
%         if strcmp(current_incline, '0^o')
%             [currPV,currState,sm,qhm] = calculatePhaseVariable_Walk(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
%         else
%             [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
%         end
        
        [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
        %
        
        
        pv(i) = currPV;
        prevState = currState;
        prevPV = currPV;
        state(i) = currState;
end


fs = 1/100;
figure
subplot(411)
plot(gc,thigh_mean)
hold on
plot(gc,knee_mean)
xlabel("Gait Cycle (%)")
ylabel("Angle (^o)")
grid on
subplot(412)
plot(gc,pv)
xlabel("Gait Cycle (%)")
ylabel("Phase Variable")
grid on
subplot(413)
plot(gc,state)
xlabel("Gait Cycle (%)")
ylabel("State")
grid on
subplot(414)
plot(gc,ddt(knee_d30,fs)./ddt(thigh_d30,fs),"linewidth",2)
ylabel("Knee_{vel}/Thigh_{vel}")
xlabel("Gait Cycle (%)")
xline(60,"k--","linewidth",2)
grid on

