%% Load Data
clearvars -except Streaming Normalized R01 rawR01;
close all
clc;
addpath('../Utility Functions')
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
if ~exist('Normalized')
    load '../Data/Normalized.mat'
end
% percentGait=linspace(0,1,150);


trial={'s3'};

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

L = length(thigh_20);


%% Load Walking Trajectories
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s1'};
incline={'i0'};
[thigh_0, knee_0, ankle_0, thigh_0sd, knee_0sd, ankle_0sd] = averageJointKinematics_Walk(Normalized,sub,trial,incline);

figure
plot(thigh_0)

%% Parameterize Joint Trajectories as functions of Phase
current_incline = '20^o';
for ind = 1:5
    
    switch ind
        case 1
            thigh_mean = thigh_0;
            knee_mean = knee_0;
            ankle_mean = ankle_0;
            current_incline = '0^o';
        case 2
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_20;
            knee_mean = knee_20;
            ankle_mean = ankle_20;
            current_incline = '20^o';
        case 3
            %             load '../Data/filtStairTraj_i25'
            thigh_mean = thigh_25;
            knee_mean = knee_25;
            ankle_mean = ankle_25;
            current_incline = '25^o';
        case 4
            %             load '../Data/filtStairTraj_i30'
            thigh_mean = thigh_30;
            knee_mean = knee_30;
            ankle_mean = ankle_30;
            current_incline = '30^o';
        case 5
            %             load '../Data/filtStairTraj_i35'
            thigh_mean = thigh_35;
            knee_mean = knee_35;
            ankle_mean = ankle_35;
            current_incline = '35^o';
        otherwise
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_20;
            knee_mean = knee_20;
            ankle_mean = ankle_20;
            current_incline = '20^o';
    end
    
    knee_mean = knee_mean';
    thigh_mean = thigh_mean';
    ankle_mean = ankle_mean';
    thighd_mean = ddt(thigh_mean);
    prevPV = 0;
    prevState = 1;
    sm = 0;
    qhm = 0;
    qh_max = max(thigh_mean);
    qh_min = min(thigh_mean);
    temp_max = qh_max;
    ypeak = [];
    xpeak = [];
    temp_traj = [];
    ix = 1:100;
    mhf = 0;
    q_po = -5;%thigh_mean(find(ankle_mean(ix)==max(ankle_mean(ix))))
    %peak detection parameters
    %threshold and min distance
    thresh = .00001;
    minpeakh = 20;
    minpeakd = 50;
    
    
    figure
    
    
    pv = zeros(1,length(thigh_mean));
    t = linspace(0,100,length(thigh_mean));
    c = t(find(thigh_mean == min(thigh_mean)))/100;
    
    
    for i = 1:length(t)
        
        thigh = thigh_mean(i);
        thighd = thighd_mean(i);
        
        
        if prevState == 3 || prevState == 4
            if thigh > 20 %&& i-p > minpeakd
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
                        
                        %                 plot(1:i-1,pv)
                        %                 ylabel('phasevariable')
                        %                 title('PV Streaming')
                        %                 hold on
                    end
                end
                
            end
        end
        
        
        
        [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV,sm, qhm,mhf,.69);
       
        pv(i) = currPV;
        prevState = currState;
        prevPV = currPV;
        
    end
    
    
    figure
    plot(t,pv)
    xlabel('Normalized Time')
    ylabel('Phase Variable')
    
    figure
    plot(t,thigh_mean,t,ankle_mean)
    
%     pv = unique(pv,'stable');
%     pv = smooth(interp1(1:length(pv), pv, 1:length(pv)/151:length(pv)))';
    
    T(ind,:) = interp1(pv,t/100,pv,'linear','extrap');
    knee_interp(ind,:) = interp1(pv,knee_mean,T(ind,:),'linear','extrap');
    ankle_interp(ind,:) = interp1(pv, ankle_mean, T(ind,:),'linear','extrap');
    knee_traj(ind,:) = knee_mean;
    ankle_traj(ind,:) = ankle_mean;
    
end



stairVCTrajectories.knee.i35 = knee_interp(5,:);
stairVCTrajectories.knee.i30 = knee_interp(4,:);
stairVCTrajectories.knee.i25 = knee_interp(3,:);
stairVCTrajectories.knee.i20 = knee_interp(2,:);
stairVCTrajectories.knee.i0 = knee_interp(1,:);

stairVCTrajectories.ankle.i35 = ankle_interp(5,:);
stairVCTrajectories.ankle.i30 = ankle_interp(4,:);
stairVCTrajectories.ankle.i25 = ankle_interp(3,:);
stairVCTrajectories.ankle.i20 = ankle_interp(2,:);
stairVCTrajectories.ankle.i0 = ankle_interp(1,:);




for ind = 1:4
    
    switch ind
        case 1
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_d20;
            knee_mean = knee_d20;
            ankle_mean = ankle_d20;
            current_incline = '20^o';
        case 3
            %             load '../Data/filtStairTraj_i25'
            thigh_mean = thigh_d25;
            knee_mean = knee_d25;
            ankle_mean = ankle_d25;
            current_incline = '25^o';
        case 4
            %             load '../Data/filtStairTraj_i30'
            thigh_mean = thigh_d30;
            knee_mean = knee_d30;
            ankle_mean = ankle_d30;
            current_incline = '30^o';
        case 5
            %             load '../Data/filtStairTraj_i35'
            thigh_mean = thigh_d35;
            knee_mean = knee_d35;
            ankle_mean = ankle_d35;
            current_incline = '35^o';
        otherwise
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_d20;
            knee_mean = knee_d20;
            ankle_mean = ankle_d20;
            current_incline = '20^o';
    end
    
    
    thighd_mean = ddt(thigh_mean);
    prevPV = 0;
    prevState = 1;
    sm = 0;
    qhm = 0;
    qh_max = max(thigh_mean);
    qh_min = min(thigh_mean);
    ix = 1:100;
    q_po = 7;%thigh_mean(find(ankle_mean(ix)==max(ankle_mean(ix))))
    
    %peak detection parameters
    %threshold and min distance
    thresh = .00001;
    minpeakh = 15;
    minpeakd = 50;
    mhf = 0;
    temp_traj = [];
    ypeak = [];
    xpeak = [];
    pv_swing_thresh = .90;
    
    

    
    
    pv = zeros(1,length(thigh_mean));
    t = linspace(0,100,length(thigh_mean));
    c = t(find(thigh_mean == min(thigh_mean)))/100;
    q_po = 7;
    
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
    pv_swing_thresh = .90;
    ytrough = [];
    xtrough = [];
    
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
        [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
        %
        
        
        pv(i) = currPV;
        prevState = currState;
        prevPV = currPV;
        
    end
    
    figure
    
    plot(t,pv)
    xlabel('Normalized Time')
    ylabel('Phase Variable')
    
    T(ind,:) = interp1(pv,t/100,pv,'linear','extrap');
    knee_interpd(ind,:) = interp1(pv,knee_mean,T(ind,:),'linear','extrap');
    ankle_interpd(ind,:) = interp1(pv, ankle_mean, T(ind,:),'linear','extrap');
    knee_trajd(ind,:) = knee_mean;
    ankle_trajd(ind,:) = ankle_mean;
    

    
end

stairVCTrajectories.knee.in20 = knee_interpd(1,:);
stairVCTrajectories.knee.in25 = knee_interpd(2,:);
stairVCTrajectories.knee.in30 = knee_interpd(3,:);
stairVCTrajectories.knee.in35 = knee_interpd(4,:);

stairVCTrajectories.ankle.in20 = ankle_interpd(1,:);
stairVCTrajectories.ankle.in25 = ankle_interpd(2,:);
stairVCTrajectories.ankle.in30 = ankle_interpd(3,:);
stairVCTrajectories.ankle.in35 = ankle_interpd(4,:);


save ../Data/stairVCTrajectories.mat stairVCTrajectories
save stairVCTrajectories.mat stairVCTrajectories
