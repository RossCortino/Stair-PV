%% Load Data 
clearvars -except Streaming Normalized R01 rawR01;
close all
clc;

addpath('../Utility Functions')

if ~exist('Normalized')
    load '../Data/Normalized.mat'
end

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s3'};
% if ~exist('Streaming')
%     load ../Data/Streaming.mat
% end
    
% prompt = 'Leave out -20deg(1), -25deg(2), -30deg(3), -35deg(4): ';
% leftOutInc = input(prompt);
% class(leftOutInc)
incline={'in20'};

[thigh_20, knee_20, ankle_20, thigh_20sd, knee_20sd, ankle_20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in25'};

[thigh_25, knee_25, ankle_25, thigh_25sd, knee_25sd, ankle_25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in30'};

[thigh_30, knee_30, ankle_30, thigh_30sd, knee_30sd, ankle_30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in35'};

[thigh_35, knee_35, ankle_35, thigh_35sd, knee_35sd, ankle_35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

t = linspace(0,100,length(thigh_20));
figure
plot(t, thigh_20)
hold on
plot(t, thigh_25)
plot(t, thigh_30)
plot(t, thigh_35)

L = length(thigh_20);

%% Calculate PV
current_incline = '-20^o';
for ind = 1:4
    
    switch ind
        case 1
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_20;
            knee_mean = knee_20;
            ankle_mean = ankle_20;
            current_incline = '-20^o'
        case 2
            %             load '../Data/filtStairTraj_i25'
            thigh_mean = thigh_25;
            knee_mean = knee_25;
            ankle_mean = ankle_25;
            current_incline = '-25^o'
        case 3
            %             load '../Data/filtStairTraj_i30'
            thigh_mean = thigh_30;
            knee_mean = knee_30;
            ankle_mean = ankle_30;
            current_incline = '-30^o'
        case 4
            %             load '../Data/filtStairTraj_i35'
            thigh_mean = thigh_35;
            knee_mean = knee_35;
            ankle_mean = ankle_35;
            current_incline = '-35^o';
        otherwise
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_20;
            knee_mean = knee_20;
            ankle_mean = ankle_20;
            current_incline = '-20^o';
    end
    
    knee_mean = knee_mean';
    thigh_mean = thigh_mean';
    
    thighd_mean = ddt(thigh_mean);
    prevPV = 0;
    prevState = 1;
    sm = 0;
    qhm = 0;
    qh_max = max(thigh_mean);
    qh_min = min(thigh_mean);
    
    c = .53;
    pv = zeros(1,length(thigh_mean));
    t = linspace(0,100,length(thigh_mean));
    q_po = 7;
    
    
    %peak detection parameters
    %threshold and min distance
    thresh = .00001;
    minpeakh = 15;
    minpeakd = 50;
    mhf = 0;
    temp_traj = [];
    pv_swing_thresh = .96;
    ypeak = [];
    xpeak = [];
    
    for i = 1:length(t)
        
        thigh = thigh_mean(i);
        thighd = thighd_mean(i);
        
        
        [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
        
        
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
                        
                        %                 plot(1:i-1,pv)
                        %                 ylabel('phasevariable')
                        %                 title('PV Streaming')
                        %                 hold on
                    end
                end
                
            end
        end
        %
        
        
        pv(i) = currPV;
        prevState = currState;
        prevPV = currPV;
        
    end
    
    
    % figure
    % plot(t,pv)
    % xlabel('Normalized Time')
    % ylabel('Phase Variable')
    
%     pv = unique(pv,'stable')
%     pv = smooth(interp1(1:length(pv), pv, 1:length(pv)/151:length(pv)))';
    
    T(ind,:) = interp1(pv,t/100,pv,'linear','extrap');
    knee_interp = interp1(pv,knee_mean,T(ind,:),'linear','extrap');
    ankle_interp = interp1(pv, ankle_mean, T(ind,:),'linear','extrap');
    
    L = length(knee_interp);
    fs = 150;
%     f = fs*(0:(L/2))/L;
%     fc = 6;
% 
%     [b,a] = butter(2,fc/(fs/2),'low');
% 
%     knee_interp = filtfilt(b,a,knee_interp);
%     
%     fc = 6;
% 
%     [b,a] = butter(2,fc/(fs/2),'low');
%     
%     ankle_interp = filtfilt(b,a,ankle_interp);

    
%     figure
%     plot(t,pv)
    
    % calculate knee
    Y_k = fft(knee_interp);
    L = length(Y_k);
    pk_real = real(Y_k/(L/2));
    pk_im = imag(Y_k/(L/2));
    
    
    % calculate ankle
    Y_a = fft(ankle_interp);
    L = length(Y_a);
    f = 116*(0:(L/2))/L;
    pa_real = real(Y_a/(L/2));
    pa_im = imag(Y_a/(L/2));
    
    sh = sym('sh');
    
    N_k = 20;
    N_a = 20;
    hk = .5*pk_real(1) + .5*pk_real(N_k/2)*cos(pi*N_k*sh);
    ha = .5*pa_real(1) + .5*pa_real(N_a/2)*cos(pi*N_a*sh);
    
    for k = 1:N_k/2-1
        hk = hk + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
    end
    
    for a = 1:N_a/2-1
        ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
    end
    pv_data(ind,:) = pv;
    knee_est(ind,:) = double(subs(hk, sh, pv));
    ankle_est(ind,:) = double(subs(ha,sh,pv));
    knee_ref(ind,:) = knee_mean;
    ankle_ref(ind,:) = ankle_mean;
    T(ind,:) = interp1(pv,t/100,pv,'linear','extrap');
end

pvcolors = [ 70 130 180
    30 144 255
    0 191 255
    135 206 235
    ]./255;

figure

colororder(pvcolors)
%20
p1 = plot(t,pv_data(1,:),'Linewidth',2);
hold on
%25
p2 = plot(t,pv_data(2,:),'Linewidth',2);
%30
p3 = plot(t,pv_data(3,:),'Linewidth',2);
%35
p4 = plot(t,pv_data(4,:),'Linewidth',2);
xlabel('Gait Cycle (%)')
ylabel('Phase')
grid on
legend({'-20^o','-25^o','-30^o','-35^o'},'location','northwest')
hold off


newcolors = [ 70 130 180
    70 130 180
    30 144 255
    30 144 255
    0 191 255
    0 191 255
    135 206 235
    135 206 235
    ]./255;
figure

colororder(newcolors)
%20
pk11 = plot(T(1,:),knee_est(1,:),'Linewidth',2)
hold on
pk12 = plot(T(1,:),knee_ref(1,:),'--','Linewidth',2)
%25
pk21 = plot(T(2,:),knee_est(2,:),'Linewidth',2)
pk22 = plot(T(2,:),knee_ref(2,:),'--','Linewidth',2)
%30
pk31 = plot(T(3,:),knee_est(3,:),'Linewidth',2)
pk32 = plot(T(3,:),knee_ref(3,:),'--','Linewidth',2)
%35
pk41 = plot(T(4,:),knee_est(4,:),'Linewidth',2)
pk42 = plot(T(4,:),knee_ref(4,:),'--','Linewidth',2)
hold off
grid on
xlabel('Gait Cycle (%)')
ylabel('Knee Angle (^o)')
legend([pk12,pk11],{'Reference','Measured'},'location','northwest')


figure
colororder(newcolors)
%20
plot(T(1,:),ankle_est(1,:),'Linewidth',2)
hold on
plot(T(1,:),ankle_ref(1,:),'--','Linewidth',2)
%25
plot(T(2,:),ankle_est(2,:),'Linewidth',2)
plot(T(2,:),ankle_ref(2,:),'--','Linewidth',2)
%30
plot(T(3,:),ankle_est(3,:),'Linewidth',2)
plot(T(3,:),ankle_ref(3,:),'--','Linewidth',2)
%35
plot(T(4,:),ankle_est(4,:),'Linewidth',2)
plot(T(4,:),ankle_ref(4,:),'--','Linewidth',2)
xlabel('Gait Cycle (%)')
ylabel('Ankle Angle (^o)')
grid on
hold off