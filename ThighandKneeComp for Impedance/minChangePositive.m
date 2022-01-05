clearvars -except Streaming Normalized R01 rawR01 X;
close all

addpath("../Utility Functions")
if ~exist('Normalized')
    load '../Data/Normalized.mat'
end

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};

trial={'s3'};


knownIncline = -30;

incline={'i20'};

[thigh_20, knee_20, ankle_20, thigh_20sd, knee_20sd, ankle_20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i25'};

[thigh_25, knee_25, ankle_25, thigh_25sd, knee_25sd, ankle_25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i30'};

[thigh_30, knee_30, ankle_30, thigh_30sd, knee_30sd, ankle_30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i35'};

[thigh_35, knee_35, ankle_35, thigh_35sd, knee_35sd, ankle_35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);


incline={'in20'};

[thigh_d20, knee_d20, ankle_d20, thigh_d20sd, knee_d20sd, ankle_d20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in25'};

[thigh_d25, knee_d25, ankle_d25, thigh_d25sd, knee_d25sd, ankle_d25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in30'};

[thigh_d30, knee_d30, ankle_d30, thigh_d30sd, knee_d30sd, ankle_d30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in35'};

[thigh_d35, knee_d35, ankle_d35, thigh_d35sd, knee_d35sd, ankle_d35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);



gc = linspace(0,100,150);
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s1'};
incline={'i0'};
[thigh_0, knee_0, ankle_0, thigh_0sd, knee_0sd, ankle_0sd] = averageJointKinematics_Walk(Normalized,sub,trial,incline);

thigh_d30_shift = circshift(thigh_d30,-find(thigh_d30 == max(thigh_d30)));
c_d30 = gc(find(thigh_d30_shift == min(thigh_d30_shift)))/100;

thigh_mean = thigh_d30;
knee_mean = knee_d30;
ankle_mean = ankle_d30;
current_incline = '-30^o';
q_po = 7;
pv_swing_thresh = .9;
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



    

for i = 1:length(gc)
        
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
            if thigh == qh_max 
                mhf = 1
            end
%             if thigh > 15 %&& i-p > minpeakd
%                 temp_traj = [temp_traj thigh];
%                 if length(temp_traj) > 3
%                     temp_traj;
%                     [pks,locs] = findpeaks(temp_traj,'threshold', thresh,'MinPeakHeight',minpeakh);
%                     if ~isempty(pks)
%                         p = i-1;
%                         temp_traj = [];
%                         ypeak = [ypeak pks];
%                         xpeak = [xpeak p];
%                         temp_max = qh_max;
%                         qh_max = qh_max; %mean([ypeak max(thigh_mean)]);
%                         mhf = 1;
%                     end
%                 end
%                 
%             end
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
        
end
    

thigh_mean = thigh_d30;
knee_mean = knee_d30;
ankle_mean = ankle_d30;
current_incline = '-30^o';
q_po = 7;
pv_swing_thresh = .9;
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


qh_min_5 = qh_min-5;
% Calculate PV Max +5
for i = 1:length(gc)
        
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
            if thigh == qh_max 
                mhf = 1
            end
%             if thigh > 15 %&& i-p > minpeakd
%                 temp_traj = [temp_traj thigh];
%                 if length(temp_traj) > 3
%                     temp_traj;
%                     [pks,locs] = findpeaks(temp_traj,'threshold', thresh,'MinPeakHeight',minpeakh);
%                     if ~isempty(pks)
%                         p = i-1;
%                         temp_traj = [];
%                         ypeak = [ypeak pks];
%                         xpeak = [xpeak p];
%                         temp_max = qh_max;
%                         qh_max_5 = qh_max_5; %mean([ypeak max(thigh_mean)]);
%                         mhf = 1;
%                     end
%                 end
%                 
%             end
        end
        %
%         if strcmp(current_incline, '0^o')
%             [currPV,currState,sm,qhm] = calculatePhaseVariable_Walk(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
%         else
%             [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
%         end
        
        [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min_5, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
        %
        
        
        pv_5(i) = currPV;
        prevState = currState;
        prevPV = currPV;
        
end
figure
plot(gc,pv)
hold on
plot(gc,pv_5)
t = linspace(0,1,length(gc));


knee_interp = interp1(pv,knee_mean,t,'linear','extrap');
ankle_interp = interp1(pv, ankle_mean, t,'linear','extrap');


% calculate knee
Y_k = fft(knee_interp);
L = length(Y_k);
pk_real = real(Y_k/(L/2));
pk_im = imag(Y_k/(L/2));

% calculate knee
Y_k_check = fft(knee_mean);
L = length(Y_k);
pk_real_check = real(Y_k_check/(L/2));
pk_im_check = imag(Y_k_check/(L/2));


syms sh
N_k = 8;
N_a = 14;
hk_interp = .5*pk_real(1)+.5*pk_real(N_k/2+1)*cos(pi*N_k*sh);
hk_check = .5*pk_real_check(1)+.5*pk_real_check(N_k/2+1)*cos(pi*N_k*sh);

% ha_vc = .5*pa_real(1)+.5*pa_real(N_k/2+1)*cos(pi*N_k*sh);

for k = 1:N_k/2-1
        hk_check = hk_check + pk_real_check(k+1)*cos(2*pi*(k)*sh)-pk_im_check(k+1)*sin(2*pi*(k)*sh);
        hk_interp = hk_interp + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
end

knee_vc_est = double(subs(hk_interp, sh, pv));
knee_vc_est_5 = double(subs(hk_interp, sh, pv_5));

figure
sgtitle("qh_{min}: -5")
subplot(121)
plot(gc,pv)
hold on
plot(gc,pv_5)
ylabel("Phase")
xlabel("Gait Cycle (%)")
legend("Original","qh_{min} +5")
subplot(122)
plot(gc,knee_mean,'linewidth',2)
hold on
plot(gc,smooth(knee_vc_est),'--','linewidth',2)
plot(gc,smooth(knee_vc_est_5),'--','linewidth',2)
ylabel("Knee Angle (^o)")
xlabel("Gait Cycle (%)")
% plot(gc,knee_vc_check,'--')