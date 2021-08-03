
%% Load Data
clearvars -except Streaming Normalized R01 rawR01;
close all
clc;


if ~exist('Normalized')
    load '../Data/Normalized.mat'
end


sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};

% percentGait=linspace(0,1,150);


trial={'s3'};
% if ~exist('Streaming')
%     load ../Data/Streaming.mat
% end

prompt = 'Leave out 20deg(1), 25deg(2), 30deg(3), 35deg(4): ';
leftOutInc = input(prompt);


class(leftOutInc);
% load '../Data/filtStairTraj_i20.mat'
%
% thigh_20 = thigh_mean;
% knee_20 = knee_mean;
% ankle_20 = ankle_mean;
%
% load '../Data/filtStairTraj_i25.mat'
%
% thigh_25 = thigh_mean';
% knee_25 = knee_mean;
% ankle_25 = ankle_mean;
%
% load '../Data/filtStairTraj_i30.mat'
%
% thigh_30 = thigh_mean';
% knee_30 = knee_mean;
% ankle_30 = ankle_mean;
%
% load '../Data/filtStairTraj_i35.mat'
%
% thigh_35 = thigh_mean';
% knee_35 = knee_mean;
% ankle_35 = ankle_mean;

incline={'i20'};

[thigh_20, knee_20, ankle_20, thigh_20sd, knee_20sd, ankle_20sd] = averageJointKinematics(Normalized,sub,trial,incline);

incline={'i25'};

[thigh_25, knee_25, ankle_25, thigh_25sd, knee_25sd, ankle_25sd] = averageJointKinematics(Normalized,sub,trial,incline);

incline={'i30'};

[thigh_30, knee_30, ankle_30, thigh_30sd, knee_30sd, ankle_30sd] = averageJointKinematics(Normalized,sub,trial,incline);

incline={'i35'};

[thigh_35, knee_35, ankle_35, thigh_35sd, knee_35sd, ankle_35sd] = averageJointKinematics(Normalized,sub,trial,incline);



L = length(thigh_20);


%% Parameterize Joint Trajectories as functions of Phase
current_incline = '20^o';
for ind = 1:4
    
    switch ind
        case 1
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_20;
            knee_mean = knee_20;
            ankle_mean = ankle_20;
            current_incline = '20^o';
        case 2
            %             load '../Data/filtStairTraj_i25'
            thigh_mean = thigh_25;
            knee_mean = knee_25;
            ankle_mean = ankle_25;
            current_incline = '25^o';
        case 3
            %             load '../Data/filtStairTraj_i30'
            thigh_mean = thigh_30;
            knee_mean = knee_30;
            ankle_mean = ankle_30;
            current_incline = '30^o';
        case 4
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
        
        
        
        [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV,sm, qhm,mhf);
        
        
        
        %
        
        
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
    
    %     L = length(knee_interp(ind,:));
    %     fs = 100;
    %     %     f = fs*(0:(L/2))/L;
    %     fc = 3.75;
    %
    %     [b,a] = butter(2,fc/(fs/2),'low');
    %     alpha = .4;
    %
    %     knee_interp(ind,:) = firstOrderLowpass(knee_interp(ind,:),alpha);% filtfilt(b,a,knee_interp(ind,:));
    %
    %     beta = .375;
    % %     [b,a] = butter(2,fc/(fs/2),'low');
    %
    %     ankle_interp(ind,:) = firstOrderLowpass(ankle_interp(ind,:),beta);% filtfilt(b,a,ankle_interp(ind,:));
    %
    %     pv = [];
    
end

knee_20_incvc = knee_interp(1,:);
knee_25_incvc = knee_interp(2,:);
knee_30_incvc = knee_interp(3,:);
knee_35_incvc = knee_interp(4,:);

ankle_20_incvc = ankle_interp(1,:);
ankle_25_incvc = ankle_interp(2,:);
ankle_30_incvc = ankle_interp(3,:);
ankle_35_incvc = ankle_interp(4,:);



knee_20_inc = knee_traj(1,:);
knee_25_inc = knee_traj(2,:);
knee_30_inc = knee_traj(3,:);
knee_35_inc = knee_traj(4,:);

ankle_20_inc = ankle_traj(1,:);
ankle_25_inc = ankle_traj(2,:);
ankle_30_inc = ankle_traj(3,:);
ankle_35_inc = ankle_traj(4,:);

% Coefficients incline and vc interp
L = length(knee_20);
N = 150;
N_k = 12;
N_a = 14;


switch leftOutInc
    case 1 %20
        pk_real25_incvc = real(fft(knee_25_incvc)/(L/2));
        pk_real30_incvc = real(fft(knee_30_incvc)/(L/2));
        pk_real35_incvc = real(fft(knee_35_incvc)/(L/2));
        pk_im25_incvc = imag(fft(knee_25_incvc)/(L/2));
        pk_im30_incvc = imag(fft(knee_30_incvc)/(L/2));
        pk_im35_incvc = imag(fft(knee_35_incvc)/(L/2));
        
        pa_real25_incvc = real(fft(ankle_25_incvc)/(L/2));
        pa_real30_incvc = real(fft(ankle_30_incvc)/(L/2));
        pa_real35_incvc = real(fft(ankle_35_incvc)/(L/2));
        pa_im25_incvc = imag(fft(ankle_25_incvc)/(L/2));
        pa_im30_incvc = imag(fft(ankle_30_incvc)/(L/2));
        pa_im35_incvc = imag(fft(ankle_35_incvc)/(L/2));
        
        x = [max(thigh_25) max(thigh_30) max(thigh_35)];
    case 2 %25
        pk_real20_incvc = real(fft(knee_20_incvc)/(L/2));
        pk_real30_incvc = real(fft(knee_30_incvc)/(L/2));
        pk_real35_incvc = real(fft(knee_35_incvc)/(L/2));
        pk_im20_incvc = imag(fft(knee_20_incvc)/(L/2));
        pk_im30_incvc = imag(fft(knee_30_incvc)/(L/2));
        pk_im35_incvc = imag(fft(knee_35_incvc)/(L/2));
        
        pa_real20_incvc = real(fft(ankle_20_incvc)/(L/2));
        pa_real30_incvc = real(fft(ankle_30_incvc)/(L/2));
        pa_real35_incvc = real(fft(ankle_35_incvc)/(L/2));
        pa_im20_incvc = imag(fft(ankle_20_incvc)/(L/2));
        pa_im30_incvc = imag(fft(ankle_30_incvc)/(L/2));
        pa_im35_incvc = imag(fft(ankle_35_incvc)/(L/2));
        
        x = [max(thigh_20) max(thigh_30) max(thigh_35)];
    case 3 %30
        pk_real20_incvc = real(fft(knee_20_incvc)/(L/2));
        pk_real25_incvc = real(fft(knee_25_incvc)/(L/2));
        pk_real35_incvc = real(fft(knee_35_incvc)/(L/2));
        pk_im20_incvc = imag(fft(knee_20_incvc)/(L/2));
        pk_im25_incvc = imag(fft(knee_25_incvc)/(L/2));
        pk_im35_incvc = imag(fft(knee_35_incvc)/(L/2));
        
        pa_real20_incvc = real(fft(ankle_20_incvc)/(L/2));
        pa_real25_incvc = real(fft(ankle_25_incvc)/(L/2));
        pa_real35_incvc = real(fft(ankle_35_incvc)/(L/2));
        pa_im20_incvc = imag(fft(ankle_20_incvc)/(L/2));
        pa_im25_incvc = imag(fft(ankle_25_incvc)/(L/2));
        pa_im35_incvc = imag(fft(ankle_35_incvc)/(L/2));
        
        x = [max(thigh_20) max(thigh_25) max(thigh_35)];
    case 4 %35
        pk_real20_incvc = real(fft(knee_20_incvc)/(L/2));
        pk_real25_incvc = real(fft(knee_25_incvc)/(L/2));
        pk_real30_incvc = real(fft(knee_30_incvc)/(L/2));
        pk_im20_incvc = imag(fft(knee_20_incvc)/(L/2));
        pk_im25_incvc = imag(fft(knee_25_incvc)/(L/2));
        pk_im30_incvc = imag(fft(knee_30_incvc)/(L/2));
        
        pa_real20_incvc = real(fft(ankle_20_incvc)/(L/2));
        pa_real25_incvc = real(fft(ankle_25_incvc)/(L/2));
        pa_real30_incvc = real(fft(ankle_30_incvc)/(L/2));
        pa_im20_incvc = imag(fft(ankle_20_incvc)/(L/2));
        pa_im25_incvc = imag(fft(ankle_25_incvc)/(L/2));
        pa_im30_incvc = imag(fft(ankle_30_incvc)/(L/2));
        
        x = [max(thigh_20) max(thigh_25) max(thigh_30)];
    otherwise
        pk_real25_incvc = real(fft(knee_25_incvc)/(L/2));
        pk_real30_incvc = real(fft(knee_30_incvc)/(L/2));
        pk_real35_incvc = real(fft(knee_35_incvc)/(L/2));
        pk_im25_incvc = imag(fft(knee_25_incvc)/(L/2));
        pk_im30_incvc = imag(fft(knee_30_incvc)/(L/2));
        pk_im35_incvc = imag(fft(knee_35_incvc)/(L/2));
        
        pa_real25_incvc = real(fft(ankle_25_incvc)/(L/2));
        pa_real30_incvc = real(fft(ankle_30_incvc)/(L/2));
        pa_real35_incvc = real(fft(ankle_35_incvc)/(L/2));
        pa_im25_incvc = imag(fft(ankle_25_incvc)/(L/2));
        pa_im30_incvc = imag(fft(ankle_30_incvc)/(L/2));
        pa_im35_incvc = imag(fft(ankle_35_incvc)/(L/2));
        
        x = [max(thigh_25) max(thigh_30) max(thigh_35)];
end

for i = 1:N/2
    switch leftOutInc
        case 1 %20
            vkreal_incvc(i,:) = [pk_real25_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i)];
            vkim_incvc(i,:) = [pk_im25_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i)];
            vareal_incvc(i,:) = [pa_real25_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i)];
            vaim_incvc(i,:) = [pa_im25_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i)];
            
        case 2 %25
            vkreal_incvc(i,:) = [pk_real20_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i)];
            vkim_incvc(i,:) = [pk_im20_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i)];
            vareal_incvc(i,:) = [pa_real20_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i)];
            vaim_incvc(i,:) = [pa_im20_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i)];
        case 3 %30
            vkreal_incvc(i,:) = [pk_real20_incvc(i) pk_real25_incvc(i) pk_real35_incvc(i)];
            vkim_incvc(i,:) = [pk_im20_incvc(i) pk_im25_incvc(i) pk_im35_incvc(i)];
            vareal_incvc(i,:) = [pa_real20_incvc(i) pa_real25_incvc(i) pa_real35_incvc(i)];
            vaim_incvc(i,:) = [pa_im20_incvc(i) pa_im25_incvc(i) pa_im35_incvc(i)];
        case 4 %35
            vkreal_incvc(i,:) = [pk_real20_incvc(i) pk_real25_incvc(i) pk_real30_incvc(i)];
            vkim_incvc(i,:) = [pk_im20_incvc(i) pk_im25_incvc(i) pk_im30_incvc(i)];
            vareal_incvc(i,:) = [pa_real20_incvc(i) pa_real25_incvc(i) pa_real30_incvc(i)];
            vaim_incvc(i,:) = [pa_im20_incvc(i) pa_im25_incvc(i) pa_im30_incvc(i)];
        otherwise
            vkreal_incvc(i,:) = [pk_real25_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i)];
            vkim_incvc(i,:) = [pk_im25_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i)];
            vareal_incvc(i,:) = [pa_real25_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i)];
            vaim_incvc(i,:) = [pa_im25_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i)];
    end
end


% Incline Interp version

L = length(knee_20);


switch leftOutInc
    case 1 %20
        pk_real25_inc = real(fft(knee_25_inc)/(L/2));
        pk_real30_inc = real(fft(knee_30_inc)/(L/2));
        pk_real35_inc = real(fft(knee_35_inc)/(L/2));
        pk_im25_inc = imag(fft(knee_25_inc)/(L/2));
        pk_im30_inc = imag(fft(knee_30_inc)/(L/2));
        pk_im35_inc = imag(fft(knee_35_inc)/(L/2));
        
        pa_real25_inc = real(fft(ankle_25_inc)/(L/2));
        pa_real30_inc = real(fft(ankle_30_inc)/(L/2));
        pa_real35_inc = real(fft(ankle_35_inc)/(L/2));
        pa_im25_inc = imag(fft(ankle_25_inc)/(L/2));
        pa_im30_inc = imag(fft(ankle_30_inc)/(L/2));
        pa_im35_inc = imag(fft(ankle_35_inc)/(L/2));
        
        x = [max(thigh_25) max(thigh_30) max(thigh_35)];
    case 2 %25
        pk_real20_inc = real(fft(knee_20_inc)/(L/2));
        pk_real30_inc = real(fft(knee_30_inc)/(L/2));
        pk_real35_inc = real(fft(knee_35_inc)/(L/2));
        pk_im20_inc = imag(fft(knee_20_inc)/(L/2));
        pk_im30_inc = imag(fft(knee_30_inc)/(L/2));
        pk_im35_inc = imag(fft(knee_35_inc)/(L/2));
        
        pa_real20_inc = real(fft(ankle_20_inc)/(L/2));
        pa_real30_inc = real(fft(ankle_30_inc)/(L/2));
        pa_real35_inc = real(fft(ankle_35_inc)/(L/2));
        pa_im20_inc = imag(fft(ankle_20_inc)/(L/2));
        pa_im30_inc = imag(fft(ankle_30_inc)/(L/2));
        pa_im35_inc = imag(fft(ankle_35_inc)/(L/2));
        
        x = [max(thigh_20) max(thigh_30) max(thigh_35)];
    case 3 %30
        pk_real20_inc = real(fft(knee_20_inc)/(L/2));
        pk_real25_inc = real(fft(knee_25_inc)/(L/2));
        pk_real35_inc = real(fft(knee_35_inc)/(L/2));
        pk_im20_inc = imag(fft(knee_20_inc)/(L/2));
        pk_im25_inc = imag(fft(knee_25_inc)/(L/2));
        pk_im35_inc = imag(fft(knee_35_inc)/(L/2));
        
        pa_real20_inc = real(fft(ankle_20_inc)/(L/2));
        pa_real25_inc = real(fft(ankle_25_inc)/(L/2));
        pa_real35_inc = real(fft(ankle_35_inc)/(L/2));
        pa_im20_inc = imag(fft(ankle_20_inc)/(L/2));
        pa_im25_inc = imag(fft(ankle_25_inc)/(L/2));
        pa_im35_inc = imag(fft(ankle_35_inc)/(L/2));
        
        x = [max(thigh_20) max(thigh_25) max(thigh_35)];
    case 4 %35
        pk_real20_inc = real(fft(knee_20_inc)/(L/2));
        pk_real25_inc = real(fft(knee_25_inc)/(L/2));
        pk_real30_inc = real(fft(knee_30_inc)/(L/2));
        pk_im20_inc = imag(fft(knee_20_inc)/(L/2));
        pk_im25_inc = imag(fft(knee_25_inc)/(L/2));
        pk_im30_inc = imag(fft(knee_30_inc)/(L/2));
        
        pa_real20_inc = real(fft(ankle_20_inc)/(L/2));
        pa_real25_inc = real(fft(ankle_25_inc)/(L/2));
        pa_real30_inc = real(fft(ankle_30_inc)/(L/2));
        pa_im20_inc = imag(fft(ankle_20_inc)/(L/2));
        pa_im25_inc = imag(fft(ankle_25_inc)/(L/2));
        pa_im30_inc = imag(fft(ankle_30_inc)/(L/2));
        
        x = [max(thigh_20) max(thigh_25) max(thigh_30)];
    otherwise
        pk_real25_inc = real(fft(knee_25_inc)/(L/2));
        pk_real30_inc = real(fft(knee_30_inc)/(L/2));
        pk_real35_inc = real(fft(knee_35_inc)/(L/2));
        pk_im25_inc = imag(fft(knee_25_inc)/(L/2));
        pk_im30_inc = imag(fft(knee_30_inc)/(L/2));
        pk_im35_inc = imag(fft(knee_35_inc)/(L/2));
        
        pa_real25_inc = real(fft(ankle_25_inc)/(L/2));
        pa_real30_inc = real(fft(ankle_30_inc)/(L/2));
        pa_real35_inc = real(fft(ankle_35_inc)/(L/2));
        pa_im25_inc = imag(fft(ankle_25_inc)/(L/2));
        pa_im30_inc = imag(fft(ankle_30_inc)/(L/2));
        pa_im35_inc = imag(fft(ankle_35_inc)/(L/2));
        
        x = [max(thigh_25) max(thigh_30) max(thigh_35)];
end

for i = 1:N/2
    switch leftOutInc
        case 1 %20
            vkreal_inc(i,:) = [pk_real25_inc(i) pk_real30_inc(i) pk_real35_inc(i)];
            vkim_inc(i,:) = [pk_im25_inc(i) pk_im30_inc(i) pk_im35_inc(i)];
            vareal_inc(i,:) = [pa_real25_inc(i) pa_real30_inc(i) pa_real35_inc(i)];
            vaim_inc(i,:) = [pa_im25_inc(i) pa_im30_inc(i) pa_im35_inc(i)];
            
        case 2 %25
            vkreal_inc(i,:) = [pk_real20_inc(i) pk_real30_inc(i) pk_real35_inc(i)];
            vkim_inc(i,:) = [pk_im20_inc(i) pk_im30_inc(i) pk_im35_inc(i)];
            vareal_inc(i,:) = [pa_real20_inc(i) pa_real30_inc(i) pa_real35_inc(i)];
            vaim_inc(i,:) = [pa_im20_inc(i) pa_im30_inc(i) pa_im35_inc(i)];
        case 3 %30
            vkreal_inc(i,:) = [pk_real20_inc(i) pk_real25_inc(i) pk_real35_inc(i)];
            vkim_inc(i,:) = [pk_im20_inc(i) pk_im25_inc(i) pk_im35_inc(i)];
            vareal_inc(i,:) = [pa_real20_inc(i) pa_real25_inc(i) pa_real35_inc(i)];
            vaim_inc(i,:) = [pa_im20_inc(i) pa_im25_inc(i) pa_im35_inc(i)];
        case 4 %35
            vkreal_inc(i,:) = [pk_real20_inc(i) pk_real25_inc(i) pk_real30_inc(i)];
            vkim_inc(i,:) = [pk_im20_inc(i) pk_im25_inc(i) pk_im30_inc(i)];
            vareal_inc(i,:) = [pa_real20_inc(i) pa_real25_inc(i) pa_real30_inc(i)];
            vaim_inc(i,:) = [pa_im20_inc(i) pa_im25_inc(i) pa_im30_inc(i)];
        otherwise
            vkreal_inc(i,:) = [pk_real25_inc(i) pk_real30_inc(i) pk_real35_inc(i)];
            vkim_inc(i,:) = [pk_im25_inc(i) pk_im30_inc(i) pk_im35_inc(i)];
            vareal_inc(i,:) = [pa_real25_inc(i) pa_real30_inc(i) pa_real35_inc(i)];
            vaim_inc(i,:) = [pa_im25_inc(i) pa_im30_inc(i) pa_im35_inc(i)];
    end
end




% Subject Streaming Data Test
close all
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};

% percentGait=linspace(0,1,150);


trial={'s3'};

switch leftOutInc
    case 1
        thigh_ref = thigh_20;
        knee_ref = knee_20;
        ankle_ref = ankle_20;
        
        thigh_sd =thigh_20sd;
        ankle_sd =ankle_20sd;
        knee_sd =knee_20sd;
        thigh_traj = [thigh_25;thigh_30;thigh_35];
        incline={'i20'};
        knee_reparam = knee_interp(1,:);
        ankle_reparam = ankle_interp(1,:);
        
        leg = ["25^o", "30^o", "35^o", "Unknown (20^o)"];
        
        strIncline = "20^o"
        
        knee_ref = knee_20;
        ankle_ref = ankle_20;
        %             load '../Data/filtStairTraj_i20.mat'
    case 2
        thigh_ref = thigh_25;
        knee_ref = knee_25;
        ankle_ref = ankle_25;
        
        thigh_sd =thigh_25sd;
        ankle_sd =ankle_25sd;
        knee_sd =knee_25sd;
        
        thigh_traj = [thigh_20;thigh_30;thigh_35];
        incline={'i25'};
        knee_reparam = knee_interp(2,:);
        ankle_reparam = ankle_interp(2,:);
        
        leg = ["20^o", "30^o", "35^o", "Unknown (25^o)"];
        
        strIncline = "25^o"
        
        knee_ref = knee_25;
        ankle_ref = ankle_25;
        %             load '../Data/filtStairTraj_i25.mat'
    case 3
        thigh_ref = thigh_30;
        knee_ref = knee_30;
        ankle_ref = ankle_30;
        
        thigh_sd =thigh_30sd;
        ankle_sd =ankle_30sd;
        knee_sd =knee_30sd;
        thigh_traj = [thigh_20;thigh_25;thigh_35];
        incline={'i30'};
        knee_reparam = knee_interp(3,:);
        ankle_reparam = ankle_interp(3,:);
        
        leg = ["20^o", "25^o", "35^o", "Unknown (30^o)"];
        
        strIncline = "30^o"
        
        knee_ref = knee_30;
        ankle_ref = ankle_30;
        %             load '../Data/filtStairTraj_i30.mat'
    case 4
        thigh_ref = thigh_35;
        knee_ref = knee_35;
        ankle_ref = ankle_35;
        
        thigh_sd =thigh_35sd;
        ankle_sd =ankle_35sd;
        knee_sd =knee_35sd;
        incline={'i35'};
        thigh_traj = [thigh_20;thigh_25;thigh_30];
        knee_reparam = knee_interp(4,:);
        ankle_reparam = ankle_interp(4,:);
        
        strIncline = "35^o";
        
        leg = ["20^o", "25^o", "30^o", "Unknown (35^o)"];
        
        knee_ref = knee_35;
        ankle_ref = ankle_35;
        %             load '../Data/filtStairTraj_i35.mat'
    otherwise
        thigh_ref = thigh_20;
        knee_ref = knee_20;
        ankle_ref = ankle_20;
        
        thigh_sd =thigh_20sd;
        ankle_sd =ankle_20sd;
        knee_sd =knee_20sd;
        
        
        incline={'i20'};
        knee_reparam = knee_interp(1,:);
        ankle_reparam = ankle_interp(1,:);
        
        leg = ["25^o", "30^o", "35^o", "Unknown (20^o)"];
        
        strIncline = "20^o";
        thigh_traj = [thigh_25;thigh_30;thigh_35];
        knee_ref = knee_20;
        ankle_ref = ankle_20;
        %             load '../Data/filtStairTraj_i20.mat'
end

% calculate knee
Y_k = fft(knee_reparam);
L = length(Y_k);
pk_real_vc = real(Y_k/(L/2));
pk_im_vc = imag(Y_k/(L/2));


% calculate ankle
Y_a = fft(ankle_reparam);
L = length(Y_a);
f = 116*(0:(L/2))/L;
pa_real_vc = real(Y_a/(L/2));
pa_im_vc = imag(Y_a/(L/2));


% calculate knee
Y_k = fft(knee_ref);
L = length(Y_k);
pk_real = real(Y_k/(L/2));
pk_im = imag(Y_k/(L/2));


% calculate ankle
Y_a = fft(ankle_ref);
L = length(Y_a);
f = 116*(0:(L/2))/L;
pa_real = real(Y_a/(L/2));
pa_im = imag(Y_a/(L/2));



[thigh_mean,knee_mean,ankle_mean] = averageJointKinematics(Normalized,sub,trial,incline);

% trial={'s1'};
% [thigh_s1,knee_s1,ankle_s1] = averageJointKinematics(Normalized,sub,trial,incline);
%
% trial={'s3'};
% [thigh_s3,knee_s3,ankle_s3] = averageJointKinematics(Normalized,sub,trial,incline);
%


thigh_stream = thigh_ref;%thigh_s3'% smooth([thigh_s1 thigh_s3],10)';
knee_stream = knee_ref;%knee_s3' % smooth([knee_s1 knee_s3],10)';
ankle_stream = ankle_ref; % smooth([ankle_s1 ankle_s3],10)';

prevPV = 0;
prevState = 1;
sm = 0;
qhm = 0;
thighd_stream = ddt(thigh_stream);
t = linspace(0,1,length(thigh_mean));
c_20 = t(find(thigh_20 == min(thigh_20)));
c_25 = t(find(thigh_25 == min(thigh_25)));
c_30 = t(find(thigh_30 == min(thigh_30)));
c_35 = t(find(thigh_35 == min(thigh_35)));
c = mean([c_20 c_25 c_30 c_35]);


% figure
% plot(t,thigh_20)
% hold on
% plot(t,thigh_25)
% plot(t,thigh_30)
% plot(t,thigh_35)

%peak detection parameters
%threshold and min distance
thresh = .00001;
minpeakh = 20;
minpeakd = 50;

xpeak = [];
ypeak = [];
temp_traj = [];
p = 1;
mhf = 0;
% Add minimum hip flexion detection as well
qh_min = min(thigh_stream);
qh_max = max(thigh_stream);
temp_max = 0;

sh = sym('sh');

gc = linspace(0,100,length(thigh_stream));

knee_est = [];
%Calculate Phase Variable
for i = 1:length(thigh_stream)
    thigh = thigh_stream(i);
    thighd = thighd_stream(i);
    
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
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf);
    s(i) = currPV;
    prevState = currState;
    state(i) = currState;
    prevPV = currPV;
    
    
    
    if qh_max ~= temp_max
        xq = qh_max;
        hk_incvc = .5*interp1(x,vkreal_incvc(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_incvc(N/2,:),xq,'linear','extrap')*cos(pi*N_k*sh);
        ha_incvc = .5*interp1(x,vareal_incvc(1,:),xq,'linear','extrap')+.5*interp1(x,vaim_incvc(N/2,:),xq,'linear','extrap')*cos(pi*N_a*sh);
        
        hk_inc = .5*interp1(x,vkreal_inc(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_inc(N/2,:),xq,'linear','extrap')*cos(pi*N_k*sh);
        ha_inc = .5*interp1(x,vareal_inc(1,:),xq,'linear','extrap')+.5*interp1(x,vaim_inc(N/2,:),xq,'linear','extrap')*cos(pi*N_a*sh);
        
        hk_vc = .5*pk_real_vc(1) + .5*pk_real_vc(N_k/2)*cos(pi*N_k*sh);
        ha_vc = .5*pa_real_vc(1) + .5*pa_real_vc(N_a/2)*cos(pi*N_a*sh);
        
        hk = .5*pk_real(1) + .5*pk_real(N_k/2)*cos(pi*N_k*sh);
        ha = .5*pa_real(1) + .5*pa_real(N_a/2)*cos(pi*N_a*sh);
        
        for k = 1:N_k/2-1
            hk = hk + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
            hk_vc = hk_vc + pk_real_vc(k+1)*cos(2*pi*(k)*sh)-pk_im_vc(k+1)*sin(2*pi*(k)*sh);
            hk_incvc = hk_incvc + interp1(x,vkreal_incvc(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_incvc(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
            hk_inc = hk_inc + interp1(x,vkreal_inc(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_inc(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
        end
        
        for a = 1:N_a/2-1
            ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
            ha_vc = ha_vc + pa_real_vc(a+1)*cos(2*pi*(a)*sh)-pa_im_vc(a+1)*sin(2*pi*(a)*sh);
            ha_incvc = ha_incvc + interp1(x,vareal_incvc(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_incvc(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
            ha_inc = ha_inc + interp1(x,vareal_inc(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_inc(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
        end
        temp_max = qh_max;
    end
    knee_incvc_est(i) = double(subs(hk_incvc, sh, s(i)));
    ankle_incvc_est(i) = double(subs(ha_incvc, sh, s(i)));
    
    knee_inc_est(i) = double(subs(hk_inc, sh, s(i)));
    ankle_inc_est(i) = double(subs(ha_inc, sh, s(i)));
    
    %     knee_vc_est(i) = double(subs(hk_vc, sh, s(i)));
    %     ankle_vc_est(i) = double(subs(ha_vc, sh, s(i)));
    %
    %     knee_est(i) = double(subs(hk, sh, s(i)));
    %     ankle_est(i) = double(subs(ha, sh, s(i)));
    
end



samples = 1:length(thigh_stream);

figure
sgtitle(strcat("Incline Stair Walking Cross Validation: Leave Out ",num2str(strIncline)))
subplot(411)
plot(gc,thigh_traj(1,:),'linewidth',2)
hold on
plot(gc,thigh_traj(2,:),'linewidth',2)
plot(gc,thigh_traj(3,:),'linewidth',2)
plot(gc,thigh_stream,'--','linewidth',2)
grid on
legend(leg)
ylabel('Thigh Angle (^o)')
title('Thigh Joint Position')

subplot(412)
yyaxis left
plot(gc,s,'k')
ylabel('Phase Variable')
yyaxis right
plot(gc,state)
ylabel('State')
title('Phase/State')
grid on

kfiltcoeff = .3;
afiltcoeff = .5;

subplot(413)


k1 = knee_stream+knee_sd;
k2 = knee_stream-knee_sd;

gc2 = [gc, fliplr(gc)];
inBetween = [k1,fliplr(k2)];
pk1 = fill(gc2,inBetween,[0.3010, 0.7450, 0.9330], 'linestyle','None');
hold on
pk2 = plot(gc, knee_stream,'linewidth',2,'color',[0, 0.4470, 0.7410]);
pk3 = plot(gc, knee_incvc_est, '--','linewidth',2, 'color',[0.8500, 0.3250, 0.0980]);
pk4 = plot(gc, knee_inc_est, ':','color','black','linewidth',2)
% xlabel('samples')
ylabel('Knee Angle (^o)')
title('Knee Joint Position')
legend([pk2,pk3,pk4],'Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
grid on
kneeRMSE_incvc = RMSE(knee_incvc_est,knee_stream)
kneeRMSE_inc = RMSE(knee_inc_est,knee_stream)

subplot(414)

a1 = ankle_stream+ankle_sd;
a2 = ankle_stream-ankle_sd;

gc2 = [gc, fliplr(gc)];
inBetween = [a1,fliplr(a2)];
pa1 = fill(gc2,inBetween,[0.3010, 0.7450, 0.9330], 'linestyle','None');
hold on
pa2 = plot(gc, ankle_stream,'linewidth',2,'color',[0, 0.4470, 0.7410]);
pa3 = plot(gc, ankle_incvc_est,'--','linewidth',2, 'color',[0.8500, 0.3250, 0.0980]);
pa4 = plot(gc, ankle_inc_est,':','color','black','linewidth',2);
legend([pa2,pa3,pa4],'Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
ylabel('Ankle Angle (^o)')
xlabel('Gait Cycle (%)','FontWeight','bold')
title('Ankle Joint Position')
ankleRMSE_incvc = RMSE(ankle_incvc_est,ankle_stream)
ankleRMSE_inc = RMSE(ankle_inc_est,ankle_stream)
grid on


% figure
% sgtitle(strcat("Incline Stair Walking Cross Validation: Leave Out ",num2str(strIncline)))
% subplot(311)
% plot(gc,thigh_traj(1,:),'linewidth',2)
% hold on
% plot(gc,thigh_traj(2,:),'linewidth',2)
% plot(gc,thigh_traj(3,:),'linewidth',2)
% plot(gc,thigh_stream,'--','linewidth',2)
% grid on
% legend(leg)
% ylabel('Thigh Angle (^o)')
% title('Thigh Joint Position')
%
% subplot(412)
% yyaxis left
% plot(gc,s,'k')
% ylabel('Phase Variable')
% yyaxis right
% plot(gc,state)
% ylabel('State')
% title('Phase/State')
% grid on
%
% kfiltcoeff = .3;
% afiltcoeff = .5;
%
% subplot(413)
%
% plot(gc, knee_stream,'linewidth',2)
% hold on
% plot(gc, knee_incvc_est, '--','linewidth',2)
% plot(gc, knee_inc_est, ':','color','black','linewidth',2)
% % xlabel('samples')
% ylabel('Knee Angle (^o)')
% title('Knee Joint Position')
% legend('Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
% grid on
% kneeRMSE_incvc = RMSE(knee_incvc_est,knee_stream)
% kneeRMSE_inc = RMSE(knee_inc_est,knee_stream)
%
% subplot(414)
%
% plot(gc, ankle_stream,'linewidth',2)
% hold on
% plot(gc, ankle_incvc_est,'--','linewidth',2)
% plot(gc, ankle_inc_est,':','color','black','linewidth',2)
% legend('Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
% ylabel('Ankle Angle (^o)')
% xlabel('Gait Cycle (%)','FontWeight','bold')
% title('Ankle Joint Position')
% ankleRMSE_incvc = RMSE(ankle_incvc_est,ankle_stream)
% ankleRMSE_inc = RMSE(ankle_inc_est,ankle_stream)
% grid on














