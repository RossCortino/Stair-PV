addpath('Utility Functions')
clearvars -except Streaming Normalized R01 rawR01 stairVCTrajectories;
close all
clc;

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s3'};

load('stairVCTrajectories_DescentTrials.mat');


if ~exist('Normalized','var')
    load('../Reference Data/Normalized.mat');
end

for ind= 1:9
    switch ind
        
        case 1
            trial={'s3'};
            incline = {'i35'};
            [thigh_35, knee_35, ankle_35, thigh_35sd, knee_35sd, ankle_35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);
            thigh_mean = thigh_35;
            
        case 2
            trial={'s3'};
            incline = {'i30'};
            [thigh_30, knee_30, ankle_30, thigh_30sd, knee_30sd, ankle_30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);
            thigh_mean = thigh_30;
            
        case 3
            trial={'s3'};
            trial={'s3'};
            incline = {'i25'};
            [thigh_25, knee_25, ankle_25, thigh_25sd, knee_25sd, ankle_25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);
            thigh_mean = thigh_25;
        case 4
            trial={'s3'};
            incline = {'i20'};
            [thigh_20, knee_20, ankle_20, thigh_20sd, knee_20sd, ankle_20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);
            thigh_mean = thigh_20;
            
        case 5
            trial={'s1'};
            incline = {'i0'};
            [thigh_0, knee_0, ankle_0, thigh_0sd, knee_0sd, ankle_0sd] = averageJointKinematics_Walk(Normalized,sub,trial,incline);
            thigh_mean = thigh_0;
            
        case 6
            trial={'s3'};
            incline = {'in20'};
            [thigh_d20, knee_d20, ankle_d20, thigh_d20sd, knee_d20sd, ankle_d20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);
            thigh_mean = thigh_d20;
        case 7
            trial={'s3'};
            incline = {'in25'};
            [thigh_d25, knee_d25, ankle_d25, thigh_d25sd, knee_d25sd, ankle_d25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);
            thigh_mean = thigh_d25;
        case 8
            trial={'s3'};
            incline = {'in30'};
            [thigh_d30, knee_d30, ankle_d30, thigh_d30sd, knee_d30sd, ankle_d30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);
            thigh_mean = thigh_d30;
        case 9
            trial={'s3'};
            incline = {'in35'};
            [thigh_d35, knee_d35, ankle_d35, thigh_d35sd, knee_d35sd, ankle_d35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);
            thigh_mean = thigh_d35;
        otherwise
            trial={'s3'};
            incline = {'in20'};
            [thigh_20, knee_20, ankle_20, thigh_20sd, knee_20sd, ankle_20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);
            thigh_mean = thigh_d20;
    end
    
    %     thigh_traj(ind,:) = max(thigh_mean);
    knee_traj(ind,:) = stairVCTrajectories.knee.(incline{1});
    ankle_traj(ind,:) = stairVCTrajectories.ankle.(incline{1});
    
    N_k = 14;
    N_a = 14;
    
    syms sh
    % calculate knee
    Y_k = fft(knee_traj(ind,:));
    L = length(Y_k);
    pk_real = real(Y_k/(L/2));
    pk_im = imag(Y_k/(L/2));
    
    
    % calculate ankle
    Y_a = fft(ankle_traj(ind,:));
    L = length(Y_a);
    pa_real = real(Y_a/(L/2));
    pa_im = imag(Y_a/(L/2));
    
    
    knee_coeff = pk_real(1);
    ankle_coeff = pa_real(1);
    thigh_max(ind,1) = max(thigh_mean);
    thigh_min(ind,1) = min(thigh_mean);
    t = linspace(0,1,length(thigh_mean));
    thigh_mean_shift = circshift(thigh_mean,-find(thigh_mean == max(thigh_mean)));
    c(ind,1) = t(find(thigh_mean_shift == min(thigh_mean_shift)));
    for i = 2:N_k/2
        
        knee_coeff = [knee_coeff pk_real(i) pk_im(i)];
        
    end
    
    knee(ind,:) = [knee_coeff pk_real(N_k/2+1)];
    
    for i = 2:N_a/2
        
        ankle_coeff = [ankle_coeff pa_real(i) pa_im(i)];
    end
    
    ankle(ind,:) = [ankle_coeff pa_real(N_a/2+1)];
end

syms phase

hk = .5;
ha = .5;

for k = 1:N_k/2-1
    hk = [hk, cos(2*pi*(k)*phase), -sin(2*pi*(k)*phase)];
end

for a = 1:N_k/2-1
    ha = [ha, cos(2*pi*(a)*phase), -sin(2*pi*(a)*phase)];
end

hk = [hk .5*cos(pi*N_k*phase)];
ha =[ha .5*cos(pi*N_a*phase)];


delete('ankle_VC_func.m')
matlabFunction(ha,'File','ankle_VC_func','Vars',[phase]);
delete('knee_VC_func.m')
matlabFunction(hk,'File','knee_VC_func','Vars',[phase]);

X = struct;

X.knee = knee';
X.ankle = ankle';
X.thigh.maxima = thigh_max';
X.thigh.minima = thigh_min';
X.inclines = [35 30 25 20 0 -20 -25 -30 -35];
X.thigh.c = c';
delete('controllerConstants_StairAscent.m')
matlab.io.saveVariablesToScript('controllerConstants_StairAscent.m','X');

