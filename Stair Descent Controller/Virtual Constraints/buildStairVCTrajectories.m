clearvars -except Streaming Normalized stairAscentTrialData R01 rawR01 estimatedAngles;

close all
addpath("../Utility Functions")

if ~exist('Normalized')
    load '../Reference Data/Normalized.mat'
end
if exist('estimatedAngles')
    load estimatedAngles.mat
end

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};

trial={'s3'};

incline={'i20'};

[thigh_i20, knee_i20, ankle_i20, thigh_i20sd, knee_i20sd, ankle_i20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i25'};

[thigh_i25, knee_i25, ankle_i25, thigh_i25sd, knee_i25sd, ankle_i25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i30'};

[thigh_i30, knee_i30, ankle_i30, thigh_i30sd, knee_i30sd, ankle_i30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i35'};

[thigh_i35, knee_i35, ankle_i35, thigh_i35sd, knee_i35sd, ankle_i35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in20'};

[thigh_d20, knee_d20, ankle_d20, thigh_d20sd, knee_d20sd, ankle_d20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in25'};

[thigh_d25, knee_d25, ankle_d25, thigh_d25sd, knee_d25sd, ankle_d25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in30'};

[thigh_d30, knee_d30, ankle_d30, thigh_d30sd, knee_d30sd, ankle_d30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in35'};

[thigh_d35, knee_d35, ankle_d35, thigh_d35sd, knee_d35sd, ankle_d35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

ascend = 1;
L = length(thigh_i20);

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s1'};
incline={'i0'};
[thigh_0, knee_0, ankle_0, thigh_0sd, knee_0sd, ankle_0sd] = averageJointKinematics_Walk(Normalized,sub,trial,incline);

t = linspace(0,1,length(thigh_i20));
% figure
% plot(t,thigh_i30)
% hold on
% plot(t,thigh_i35)


thigh_0_shift = circshift(thigh_0,-find(thigh_0 == max(thigh_0)));
thigh_i20_shift = circshift(thigh_i20,-find(thigh_i20 == max(thigh_i20)));
thigh_i25_shift = circshift(thigh_i25,-find(thigh_i25 == max(thigh_i25)));
thigh_i30_shift = circshift(thigh_i30,-find(thigh_i30 == max(thigh_i30)));
thigh_i35_shift = circshift(thigh_i35,-find(thigh_i35 == max(thigh_i35)));
thigh_d20_shift = circshift(thigh_d20,-find(thigh_d20 == max(thigh_d20)));
thigh_d25_shift = circshift(thigh_d25,-find(thigh_d25 == max(thigh_d25)));
thigh_d30_shift = circshift(thigh_d30,-find(thigh_d30 == max(thigh_d30)));
thigh_d35_shift = circshift(thigh_d35,-find(thigh_d35 == max(thigh_d35)));

knee_0_shift = circshift(knee_0,-find(thigh_0 == max(thigh_0)));
knee_i20_shift = circshift(knee_i20,-find(thigh_i20 == max(thigh_i20)));
knee_i25_shift = circshift(knee_i25,-find(thigh_i25 == max(thigh_i25)));
knee_i30_shift = circshift(knee_i30,-find(thigh_i30 == max(thigh_i30)));
knee_i35_shift = circshift(knee_i35,-find(thigh_i35 == max(thigh_i35)));
knee_d20_shift = circshift(knee_d20,-find(thigh_d20 == max(thigh_d20)));
knee_d25_shift = circshift(knee_d25,-find(thigh_d25 == max(thigh_d25)));
knee_d30_shift = circshift(knee_d30,-find(thigh_d30 == max(thigh_d30)));
knee_d35_shift = circshift(knee_d35,-find(thigh_d35 == max(thigh_d35)));

ankle__0_shift = circshift(ankle_0,-find(thigh_0 == max(thigh_0)));
ankle_i20_shift = circshift(ankle_i20,-find(thigh_i20 == max(thigh_i20)));
ankle_i25_shift = circshift(ankle_i25,-find(thigh_i25 == max(thigh_i25)));
ankle_i30_shift = circshift(ankle_i30,-find(thigh_i30 == max(thigh_i30)));
ankle_i35_shift = circshift(ankle_i35,-find(thigh_i35 == max(thigh_i35)));
ankle_d20_shift = circshift(ankle_d20,-find(thigh_d20 == max(thigh_d20)));
ankle_d25_shift = circshift(ankle_d25,-find(thigh_d25 == max(thigh_d25)));
ankle_d30_shift = circshift(ankle_d30,-find(thigh_d30 == max(thigh_d30)));
ankle_d35_shift = circshift(ankle_d35,-find(thigh_d35 == max(thigh_d35)));

c_0 = t(find(thigh_0_shift == min(thigh_0_shift)));
c_i20 = t(find(thigh_i20_shift == min(thigh_i20_shift)));
c_i25 = t(find(thigh_i25_shift == min(thigh_i25_shift)));
c_i30 = t(find(thigh_i30_shift == min(thigh_i30_shift)));
c_i35 = t(find(thigh_i35_shift == min(thigh_i35_shift)));
c_d20 = t(find(thigh_d20_shift == min(thigh_d20_shift)));
c_d25 = t(find(thigh_d25_shift == min(thigh_d25_shift)));
c_d30 = t(find(thigh_d30_shift == min(thigh_d30_shift)));
c_d35 = t(find(thigh_d35_shift == min(thigh_d35_shift)));

c_avg = mode([c_i20 c_i25 c_i30 c_i35 c_d20 c_d25 c_d30 c_d35]);

FF = 1;

figure
for ind = 1:9
    
    switch ind
        case 1
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_0;%circshift(thigh_0,-find(thigh_0 == max(thigh_0)));
            knee_mean = knee_0;
            ankle_mean = ankle_0;
            current_incline = '0^o'
            pv_swing_thresh = .85;
            c = c_0;
            q_po = -10;
            ascend = 0;
        case 2
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_i20;
            knee_mean = knee_i20;
            ankle_mean = ankle_i20;
            current_incline = '20^o'
            pv_swing_thresh = .65;
            c= c_i20;
            q_po = -4;
            ascend = 1;
        case 3
            %             load '../Data/filtStairTraj_i25'
            thigh_mean = thigh_i25;
            knee_mean = knee_i25;
            ankle_mean = ankle_i25;
            current_incline = '25^o'
            pv_swing_thresh = .65;
            c= c_i25;
            q_po = -4;
            ascend = 1;
        case 4
            %             load '../Data/filtStairTraj_i30'
            thigh_mean = thigh_i30;
            knee_mean = knee_i30;
            ankle_mean = ankle_i30;
            current_incline = '30^o'
            pv_swing_thresh = .65;
            c= c_i30;
            q_po = -4;
            ascend = 1;
        case 5
            %             load '../Data/filtStairTraj_i35'
            thigh_mean = thigh_i35;
            knee_mean = knee_i35;
            ankle_mean = ankle_i35;
            current_incline = '35^o';
            pv_swing_thresh = .65;
            c= c_i35;
            q_po = -4;
            ascend = 1;
        case 6
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_d20;
            knee_mean = knee_d20;
            ankle_mean = ankle_d20;
            current_incline = '-20^o'
            s_po = .55;
            pv_swing_thresh = .96;
            c= c_d20;
            ascend = 0;
        case 7
            %             load '../Data/filtStairTraj_i25'
            thigh_mean = thigh_d25;
            knee_mean = knee_d25;
            ankle_mean = ankle_d25;
            current_incline = '-25^o'
            s_po = .55;
            pv_swing_thresh = .96;
            c= c_d25;
            ascend = 0;
        case 8
            %             load '../Data/filtStairTraj_i30'
            
            dt_orig = 1/100;
            samp_orig = 135;
            time_orig = linspace(0,samp_orig*dt_orig-dt_orig,samp_orig);
            L = 150+1;
            time_normalized = interp1(1:length(time_orig), time_orig, 1:length(time_orig)/(L):length(time_orig));
            dt_normalized = mode(diff(time_normalized));
            
            filt_freq = 5 ; %Hz
            samprate = 1/mode(dt_normalized); % Hz
            [B_f,A_f] = butter(2,filt_freq/(samprate/2)) ;
            
            
%             thigh_pos_d30_shift = filtfilt(B_f,A_f,circshift(thigh_pos_d30,-find(thigh_pos_d30 == max(thigh_pos_d30))));
            
            thigh_mean = circshift(filtfilt(B_f,A_f,thigh_d30_shift),find(thigh_d30 == max(thigh_d30)));
            
            filt_freq = 6; %Hz
            samprate = 1/mode(dt_normalized); % Hz
            [B_f,A_f] = butter(2,filt_freq/(samprate/2));
      
            knee_mean = circshift(filtfilt(B_f,A_f, knee_d30_shift),find(thigh_d30 == max(thigh_d30)));
            
            figure
            plot(t,knee_d30)
            hold on
            plot(t, knee_mean)
            
            filt_freq = 7; %Hz
            samprate = 1/mode(dt_normalized); % Hz
            [B_f,A_f] = butter(2,filt_freq/(samprate/2));
      
            ankle_mean = circshift(filtfilt(B_f,A_f, ankle_d30_shift),find(thigh_d30 == max(thigh_d30)));
            
            figure
            plot(t,ankle_d30)
            hold on
            plot(t,ankle_mean)
            
            current_incline = '-30^o'
            
            s_po = .55;
            pv_swing_thresh = .96;
            c= c_d30;
            ascend = 0;
        case 9
            %             load '../Data/filtStairTraj_i35'
            thigh_mean = thigh_d35;
            knee_mean = knee_d35;
            ankle_mean = ankle_d35;
            current_incline = '-35^o'
            s_po = .55;
            pv_swing_thresh = .96;
            c= c_d35;
            ascend = 0;
        otherwise
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_i20;
            knee_mean = knee_i20;
            ankle_mean = ankle_i20;
            current_incline = '20^o';
            pv_swing_thresh = .96;
            c= c_i20;
            ascend = 1;
    end
    
    thighd_mean = ddt(thigh_mean);
    prevPV = 0;
    prevState = 1;
    sm = 0;
    qhm = 0;
    qh_max = max(thigh_mean);
    qh_min = min(thigh_mean);
    
    thresh = .00001;
    minpeakh = 15;
    minpeakd = 50;
    mhf = 0;
    temp_traj = [];
    ypeak = [];
    xpeak = [];
    
    thresh = .0000001;
    minpeakh_mhe = 6;
    minpeakd_mhe = 6;
    thigh_start_mhe = 7;
    mhe = 0;
    temp_traj_MHE = [];
%     pv_swing_thresh = .96;
    ytrough = [];
    xtrough = [];
    
    for i = 1:length(t)
        
        thigh = thigh_mean(i);
        thighd = thighd_mean(i);
        
        
        
%         if prevState == 2|| prevState == 3
%             temp_traj_MHE = [temp_traj_MHE -thigh];
%             if length(temp_traj_MHE) > 3
%                 [pks_MHE,locs_MHE] = findpeaks(temp_traj_MHE,'threshold', thresh,'MinPeakHeight',-minpeakh_mhe);
%                 if ~isempty(pks_MHE)
%                     temp_traj_MHE = [];
%                     ytrough = -[ytrough pks_MHE];
%                     xtrough = [xtrough i-1];
%                     mhe = 1;
%                     
%                 end
%             end
%             
%             
%         end
%         if prevState == 3 || prevState == 4
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
%                         qh_max = mean([ypeak max(thigh_mean)]);
%                         mhf = 1;
%                     end
%                 end
%                 
%             end
%         end

        if qh_max == thigh
            mhf = 1;
        end

        if ascend == 1 && FF == 1
            [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair_FF_S2(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
        elseif current_incline == "0^o"
            [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Walk(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf,pv_swing_thresh)
        else
            [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_StairDescent_Ratcheting(thigh, thighd, qh_min, qh_max,s_po, c, prevState,prevPV, sm, qhm,mhf,pv_swing_thresh);
        end
        
        
        pv(i) = currPV;
        state(i) = prevState;
        prevState = currState;
        prevPV = currPV;
        
    end
    
    
    
    if 1
        figure
        subplot(211)
        plot(pv)
        hold on
        yyaxis right
        plot(state)
        subplot(212)
        plot(thigh_mean)
        yyaxis right
        plot(state)
        sgtitle(strcat("Incline: ",num2str(current_incline)))
    end
    
    [pv_unique,iu] = unique(pv,'stable')
    T(ind,:) = interp1(pv,t,pv,'linear','extrap');
    knee_interp(ind,:) = interp1(pv,knee_mean,t,'linear','extrap');
    ankle_interp(ind,:) = interp1(pv, ankle_mean, t,'linear','extrap');
end



knee_0_incvc = knee_interp(1,:);
knee_20_incvc = knee_interp(2,:);
knee_25_incvc = knee_interp(3,:);
knee_30_incvc = knee_interp(4,:);
knee_35_incvc = knee_interp(5,:);

knee_20_decvc = knee_interp(6,:);
knee_25_decvc = knee_interp(7,:);
knee_30_decvc = knee_interp(8,:);
knee_35_decvc = knee_interp(9,:);



ankle_0_incvc = ankle_interp(1,:);
ankle_20_incvc = ankle_interp(2,:);
ankle_25_incvc = ankle_interp(3,:);
ankle_30_incvc = ankle_interp(4,:);
ankle_35_incvc = ankle_interp(5,:);

ankle_20_decvc = ankle_interp(6,:);
ankle_25_decvc = ankle_interp(7,:);
ankle_30_decvc = ankle_interp(8,:);
ankle_35_decvc = ankle_interp(9,:);

stairVCTrajectories.knee.i0 = knee_0_incvc;
stairVCTrajectories.knee.i20 = knee_20_incvc;
stairVCTrajectories.knee.i25 = knee_25_incvc;
stairVCTrajectories.knee.i30 = knee_30_incvc;
stairVCTrajectories.knee.i35 = knee_35_incvc;

stairVCTrajectories.knee.in20 = knee_20_decvc;
stairVCTrajectories.knee.in25 = knee_25_decvc;
stairVCTrajectories.knee.in30 = knee_30_decvc;
stairVCTrajectories.knee.in35 = knee_35_decvc;

stairVCTrajectories.ankle.i0 = ankle_0_incvc;
stairVCTrajectories.ankle.i20 = ankle_20_incvc;
stairVCTrajectories.ankle.i25 = ankle_25_incvc;
stairVCTrajectories.ankle.i30 = ankle_30_incvc;
stairVCTrajectories.ankle.i35 = ankle_35_incvc;

stairVCTrajectories.ankle.in20 = ankle_20_decvc;
stairVCTrajectories.ankle.in25 = ankle_25_decvc;
stairVCTrajectories.ankle.in30 = ankle_30_decvc;
stairVCTrajectories.ankle.in35 = ankle_35_decvc;

save stairVCTrajectories_DescentTrials.mat stairVCTrajectories