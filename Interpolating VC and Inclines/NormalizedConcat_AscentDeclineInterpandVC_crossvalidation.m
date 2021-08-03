
%% Load Data 
clearvars -except Streaming Normalized R01 rawR01;
close all
clc;


if ~exist('Normalized')
    load '../Data/Normalized.mat'
end

% if ~exist('Streaming')
%     load ../Data/Streaming.mat
% end
    
prompt = 'Leave out -20deg(1), -25deg(2), -30deg(3), -35deg(4): ';
leftOutInc = input(prompt);


class(leftOutInc)
load '../Data/filtStairTraj_d20.mat'



thigh_20 = thigh_mean';
knee_20 = knee_mean';
ankle_20 = ankle_mean';

load '../Data/filtStairTraj_d25.mat'

thigh_25 = thigh_mean';
knee_25 = knee_mean';
ankle_25 = ankle_mean';

load '../Data/filtStairTraj_d30.mat'

thigh_30 = thigh_mean';
knee_30 = knee_mean';
ankle_30 = ankle_mean';

load '../Data/filtStairTraj_d35.mat'

thigh_35 = thigh_mean';
knee_35 = knee_mean';
ankle_35 = ankle_mean';



L = length(thigh_20);


%% Parameterize Joint Trajectories as functions of Phase
current_incline = '-20^o';
for ind = 1:4
    
    switch ind
        case 1
            load '../Data/filtStairTraj_d20'
            current_incline = '-20^o';
        case 2
            load '../Data/filtStairTraj_d25'
            current_incline = '-25^o';
        case 3
            load '../Data/filtStairTraj_d30'
            current_incline = '-30^o';
        case 4
            load '../Data/filtStairTraj_d35'
            current_incline = '-35^o';
        otherwise
            load '../Data/filtStairTraj_d20'
            current_incline = '-20^o';
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
    ix = 1:100;
    q_po = 10;%thigh_mean(find(ankle_mean(ix)==max(ankle_mean(ix))))
    
    figure
    
    
    pv = zeros(1,length(thigh_mean));
    t = linspace(0,100,length(thigh_mean));
    c = t(find(thigh_mean == min(thigh_mean)))/100;
    
    
    for i = 1:length(t)
        
        thigh = thigh_mean(i);
        thighd = thighd_mean(i);
        
        
        [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair_Normalized(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm);
        
        
        
        %
        
        
        pv(i) = currPV;
        prevState = currState;
        prevPV = currPV;
        
    end
    
    
%     figure
%     plot(t,pv)
%     xlabel('Normalized Time')
%     ylabel('Phase Variable')
%     
%     figure
%     plot(t,thigh_mean,t,ankle_mean)
    
    pv = unique(pv,'stable');
    pv = smooth(interp1(1:length(pv), pv, 1:length(pv)/151:length(pv)))';
    
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

knee_20_decvc = knee_interp(1,:);
knee_25_decvc = knee_interp(2,:);
knee_30_decvc = knee_interp(3,:);
knee_35_decvc = knee_interp(4,:);

ankle_20_decvc = ankle_interp(1,:);
ankle_25_decvc = ankle_interp(2,:);
ankle_30_decvc = ankle_interp(3,:);
ankle_35_decvc = ankle_interp(4,:);



knee_20_dec = knee_traj(1,:);
knee_25_dec = knee_traj(2,:);
knee_30_dec = knee_traj(3,:);
knee_35_dec = knee_traj(4,:);

ankle_20_dec = ankle_traj(1,:);
ankle_25_dec = ankle_traj(2,:);
ankle_30_dec = ankle_traj(3,:);
ankle_35_dec = ankle_traj(4,:);

% Coefficients incline and vc interp
L = length(knee_20);
N = 120;
N_k = 20;
N_a = 20;
switch leftOutInc
    case 1 %20
        pk_real25_decvc = real(fft(knee_25_decvc)/(L/2));
        pk_real30_decvc = real(fft(knee_30_decvc)/(L/2));
        pk_real35_decvc = real(fft(knee_35_decvc)/(L/2));
        pk_im25_decvc = imag(fft(knee_25_decvc)/(L/2));
        pk_im30_decvc = imag(fft(knee_30_decvc)/(L/2));
        pk_im35_decvc = imag(fft(knee_35_decvc)/(L/2));
        
        pa_real25_decvc = real(fft(ankle_25_decvc)/(L/2));
        pa_real30_decvc = real(fft(ankle_30_decvc)/(L/2));
        pa_real35_decvc = real(fft(ankle_35_decvc)/(L/2));
        pa_im25_decvc = imag(fft(ankle_25_decvc)/(L/2));
        pa_im30_decvc = imag(fft(ankle_30_decvc)/(L/2));
        pa_im35_decvc = imag(fft(ankle_35_decvc)/(L/2));
        
        x = [min(thigh_25) min(thigh_30) min(thigh_35)];
    case 2 %25
        pk_real20_decvc = real(fft(knee_20_decvc)/(L/2));
        pk_real30_decvc = real(fft(knee_30_decvc)/(L/2));
        pk_real35_decvc = real(fft(knee_35_decvc)/(L/2));
        pk_im20_decvc = imag(fft(knee_20_decvc)/(L/2));
        pk_im30_decvc = imag(fft(knee_30_decvc)/(L/2));
        pk_im35_decvc = imag(fft(knee_35_decvc)/(L/2));
        
        pa_real20_deccvc = real(fft(ankle_20_decvc)/(L/2));
        pa_real30_decvc = real(fft(ankle_30_decvc)/(L/2));
        pa_real35_decvc = real(fft(ankle_35_decvc)/(L/2));
        pa_im20_decvc = imag(fft(ankle_20_decvc)/(L/2));
        pa_im30_decvc = imag(fft(ankle_30_decvc)/(L/2));
        pa_im35_decvc = imag(fft(ankle_35_decvc)/(L/2));
        
        x = [min(thigh_20) min(thigh_30) min(thigh_35)];
    case 3 %30
        pk_real20_decvc = real(fft(knee_20_decvc)/(L/2));
        pk_real25_decvc = real(fft(knee_25_decvc)/(L/2));
        pk_real35_decvc = real(fft(knee_35_decvc)/(L/2));
        pk_im20_decvc = imag(fft(knee_20_decvc)/(L/2));
        pk_im25_decvc = imag(fft(knee_25_decvc)/(L/2));
        pk_im35_decvc = imag(fft(knee_35_decvc)/(L/2));
        
        pa_real20_deccvc = real(fft(ankle_20_decvc)/(L/2));
        pa_real25_decvc = real(fft(ankle_25_decvc)/(L/2));
        pa_real35_decvc = real(fft(ankle_35_decvc)/(L/2));
        pa_im20_decvc = imag(fft(ankle_20_decvc)/(L/2));
        pa_im25_decvc = imag(fft(ankle_25_decvc)/(L/2));
        pa_im35_decvc = imag(fft(ankle_35_decvc)/(L/2));
        
        x = [min(thigh_20) min(thigh_25) min(thigh_35)];
    case 4 %35
        pk_real20_decvc = real(fft(knee_20_decvc)/(L/2));
        pk_real25_decvc = real(fft(knee_25_decvc)/(L/2));
        pk_real30_decvc = real(fft(knee_30_decvc)/(L/2));
        pk_im20_decvc = imag(fft(knee_20_decvc)/(L/2));
        pk_im25_decvc = imag(fft(knee_25_decvc)/(L/2));
        pk_im30_decvc = imag(fft(knee_30_decvc)/(L/2));
        
        pa_real20_deccvc = real(fft(ankle_20_decvc)/(L/2));
        pa_real25_decvc = real(fft(ankle_25_decvc)/(L/2));
        pa_real30_decvc = real(fft(ankle_30_decvc)/(L/2));
        pa_im20_decvc = imag(fft(ankle_20_decvc)/(L/2));
        pa_im25_decvc = imag(fft(ankle_25_decvc)/(L/2));
        pa_im30_decvc = imag(fft(ankle_30_decvc)/(L/2));
        
        x = [min(thigh_20) min(thigh_25) min(thigh_30)];
    otherwise
        pk_real25_decvc = real(fft(knee_25_decvc)/(L/2));
        pk_real30_decvc = real(fft(knee_30_decvc)/(L/2));
        pk_real35_decvc = real(fft(knee_35_decvc)/(L/2));
        pk_im25_decvc = imag(fft(knee_25_decvc)/(L/2));
        pk_im30_decvc = imag(fft(knee_30_decvc)/(L/2));
        pk_im35_decvc = imag(fft(knee_35_decvc)/(L/2));
        
        pa_real25_decvc = real(fft(ankle_25_decvc)/(L/2));
        pa_real30_decvc = real(fft(ankle_30_decvc)/(L/2));
        pa_real35_decvc = real(fft(ankle_35_decvc)/(L/2));
        pa_im25_decvc = imag(fft(ankle_25_decvc)/(L/2));
        pa_im30_decvc = imag(fft(ankle_30_decvc)/(L/2));
        pa_im35_decvc = imag(fft(ankle_35_decvc)/(L/2));
        
        x = [min(thigh_25) min(thigh_30) min(thigh_35)];
end
   
for i = 1:N/2
    switch leftOutInc
        case 1 %20
            vkreal_decvc(i,:) = [pk_real25_decvc(i) pk_real30_decvc(i) pk_real35_decvc(i)];
            vkim_decvc(i,:) = [pk_im25_decvc(i) pk_im30_decvc(i) pk_im35_decvc(i)];
            vareal_decvc(i,:) = [pa_real25_decvc(i) pa_real30_decvc(i) pa_real35_decvc(i)];
            vaim_decvc(i,:) = [pa_im25_decvc(i) pa_im30_decvc(i) pa_im35_decvc(i)];
            
        case 2 %25
            vkreal_decvc(i,:) = [pk_real20_decvc(i) pk_real30_decvc(i) pk_real35_decvc(i)];
            vkim_decvc(i,:) = [pk_im20_decvc(i) pk_im30_decvc(i) pk_im35_decvc(i)];
            vareal_decvc(i,:) = [pa_real20_deccvc(i) pa_real30_decvc(i) pa_real35_decvc(i)];
            vaim_decvc(i,:) = [pa_im20_decvc(i) pa_im30_decvc(i) pa_im35_decvc(i)];
        case 3 %30
            vkreal_decvc(i,:) = [pk_real20_decvc(i) pk_real25_decvc(i) pk_real35_decvc(i)];
            vkim_decvc(i,:) = [pk_im20_decvc(i) pk_im25_decvc(i) pk_im35_decvc(i)];
            vareal_decvc(i,:) = [pa_real20_deccvc(i) pa_real25_decvc(i) pa_real35_decvc(i)];
            vaim_decvc(i,:) = [pa_im20_decvc(i) pa_im25_decvc(i) pa_im35_decvc(i)];
        case 4 %35
            vkreal_decvc(i,:) = [pk_real20_decvc(i) pk_real25_decvc(i) pk_real30_decvc(i)];
            vkim_decvc(i,:) = [pk_im20_decvc(i) pk_im25_decvc(i) pk_im30_decvc(i)];
            vareal_decvc(i,:) = [pa_real20_deccvc(i) pa_real25_decvc(i) pa_real30_decvc(i)];
            vaim_decvc(i,:) = [pa_im20_decvc(i) pa_im25_decvc(i) pa_im30_decvc(i)];
        otherwise
            vkreal_decvc(i,:) = [pk_real25_decvc(i) pk_real30_decvc(i) pk_real35_decvc(i)];
            vkim_decvc(i,:) = [pk_im25_decvc(i) pk_im30_decvc(i) pk_im35_decvc(i)];
            vareal_decvc(i,:) = [pa_real25_decvc(i) pa_real30_decvc(i) pa_real35_decvc(i)];
            vaim_decvc(i,:) = [pa_im25_decvc(i) pa_im30_decvc(i) pa_im35_decvc(i)];
    end
end


% Incline Interp version

L = length(knee_20);


switch leftOutInc
    case 1 %20
        pk_real25_dec = real(fft(knee_25_dec)/(L/2));
        pk_real30_dec = real(fft(knee_30_dec)/(L/2));
        pk_real35_dec = real(fft(knee_35_dec)/(L/2));
        pk_im25_dec = imag(fft(knee_25_dec)/(L/2));
        pk_im30_dec = imag(fft(knee_30_dec)/(L/2));
        pk_im35_dec = imag(fft(knee_35_dec)/(L/2));
        
        pa_real25_dec = real(fft(ankle_25_dec)/(L/2));
        pa_real30_dec = real(fft(ankle_30_dec)/(L/2));
        pa_real35_dec = real(fft(ankle_35_dec)/(L/2));
        pa_im25_dec = imag(fft(ankle_25_dec)/(L/2));
        pa_im30_dec = imag(fft(ankle_30_dec)/(L/2));
        pa_im35_dec = imag(fft(ankle_35_dec)/(L/2));
        
        x = [min(thigh_25) min(thigh_30) min(thigh_35)];
    case 2 %25
        pk_real20_dec = real(fft(knee_20_dec)/(L/2));
        pk_real30_dec = real(fft(knee_30_dec)/(L/2));
        pk_real35_dec = real(fft(knee_35_dec)/(L/2));
        pk_im20_dec = imag(fft(knee_20_dec)/(L/2));
        pk_im30_dec = imag(fft(knee_30_dec)/(L/2));
        pk_im35_dec = imag(fft(knee_35_dec)/(L/2));
        
        pa_real20_dec = real(fft(ankle_20_dec)/(L/2));
        pa_real30_dec = real(fft(ankle_30_dec)/(L/2));
        pa_real35_dec = real(fft(ankle_35_dec)/(L/2));
        pa_im20_dec = imag(fft(ankle_20_dec)/(L/2));
        pa_im30_dec = imag(fft(ankle_30_dec)/(L/2));
        pa_im35_dec = imag(fft(ankle_35_dec)/(L/2));
        
        x = [min(thigh_20) min(thigh_30) min(thigh_35)];
    case 3 %30
        pk_real20_dec = real(fft(knee_20_dec)/(L/2));
        pk_real25_dec = real(fft(knee_25_dec)/(L/2));
        pk_real35_dec = real(fft(knee_35_dec)/(L/2));
        pk_im20_dec = imag(fft(knee_20_dec)/(L/2));
        pk_im25_dec = imag(fft(knee_25_dec)/(L/2));
        pk_im35_dec = imag(fft(knee_35_dec)/(L/2));
        
        pa_real20_dec = real(fft(ankle_20_dec)/(L/2));
        pa_real25_dec = real(fft(ankle_25_dec)/(L/2));
        pa_real35_dec = real(fft(ankle_35_dec)/(L/2));
        pa_im20_dec = imag(fft(ankle_20_dec)/(L/2));
        pa_im25_dec = imag(fft(ankle_25_dec)/(L/2));
        pa_im35_dec = imag(fft(ankle_35_dec)/(L/2));
        
        x = [min(thigh_20) min(thigh_25) min(thigh_35)];
    case 4 %35
        pk_real20_dec = real(fft(knee_20_dec)/(L/2));
        pk_real25_dec = real(fft(knee_25_dec)/(L/2));
        pk_real30_dec = real(fft(knee_30_dec)/(L/2));
        pk_im20_dec = imag(fft(knee_20_dec)/(L/2));
        pk_im25_dec = imag(fft(knee_25_dec)/(L/2));
        pk_im30_dec = imag(fft(knee_30_dec)/(L/2));
        
        pa_real20_dec = real(fft(ankle_20_dec)/(L/2));
        pa_real25_dec = real(fft(ankle_25_dec)/(L/2));
        pa_real30_dec = real(fft(ankle_30_dec)/(L/2));
        pa_im20_dec = imag(fft(ankle_20_dec)/(L/2));
        pa_im25_dec = imag(fft(ankle_25_dec)/(L/2));
        pa_im30_dec = imag(fft(ankle_30_dec)/(L/2));
        
        x = [min(thigh_20) min(thigh_25) min(thigh_30)];
    otherwise
        pk_real25_dec = real(fft(knee_25_dec)/(L/2));
        pk_real30_dec = real(fft(knee_30_dec)/(L/2));
        pk_real35_dec = real(fft(knee_35_dec)/(L/2));
        pk_im25_dec = imag(fft(knee_25_dec)/(L/2));
        pk_im30_dec = imag(fft(knee_30_dec)/(L/2));
        pk_im35_dec = imag(fft(knee_35_dec)/(L/2));
        
        pa_real25_dec = real(fft(ankle_25_dec)/(L/2));
        pa_real30_dec = real(fft(ankle_30_dec)/(L/2));
        pa_real35_dec = real(fft(ankle_35_dec)/(L/2));
        pa_im25_dec = imag(fft(ankle_25_dec)/(L/2));
        pa_im30_dec = imag(fft(ankle_30_dec)/(L/2));
        pa_im35_dec = imag(fft(ankle_35_dec)/(L/2));
        
        x = [min(thigh_25) min(thigh_30) min(thigh_35)];
end
   
for i = 1:N/2
    switch leftOutInc
        case 1 %20
            vkreal_dec(i,:) = [pk_real25_dec(i) pk_real30_dec(i) pk_real35_dec(i)];
            vkim_dec(i,:) = [pk_im25_dec(i) pk_im30_dec(i) pk_im35_dec(i)];
            vareal_dec(i,:) = [pa_real25_dec(i) pa_real30_dec(i) pa_real35_dec(i)];
            vaim_dec(i,:) = [pa_im25_dec(i) pa_im30_dec(i) pa_im35_dec(i)];
            
        case 2 %25
            vkreal_dec(i,:) = [pk_real20_dec(i) pk_real30_dec(i) pk_real35_dec(i)];
            vkim_dec(i,:) = [pk_im20_dec(i) pk_im30_dec(i) pk_im35_dec(i)];
            vareal_dec(i,:) = [pa_real20_dec(i) pa_real30_dec(i) pa_real35_dec(i)];
            vaim_dec(i,:) = [pa_im20_dec(i) pa_im30_dec(i) pa_im35_dec(i)];
        case 3 %30
            vkreal_dec(i,:) = [pk_real20_dec(i) pk_real25_dec(i) pk_real35_dec(i)];
            vkim_dec(i,:) = [pk_im20_dec(i) pk_im25_dec(i) pk_im35_dec(i)];
            vareal_dec(i,:) = [pa_real20_dec(i) pa_real25_dec(i) pa_real35_dec(i)];
            vaim_dec(i,:) = [pa_im20_dec(i) pa_im25_dec(i) pa_im35_dec(i)];
        case 4 %35
            vkreal_dec(i,:) = [pk_real20_dec(i) pk_real25_dec(i) pk_real30_dec(i)];
            vkim_dec(i,:) = [pk_im20_dec(i) pk_im25_dec(i) pk_im30_dec(i)];
            vareal_dec(i,:) = [pa_real20_dec(i) pa_real25_dec(i) pa_real30_dec(i)];
            vaim_dec(i,:) = [pa_im20_dec(i) pa_im25_dec(i) pa_im30_dec(i)];
        otherwise
            vkreal_dec(i,:) = [pk_real25_dec(i) pk_real30_dec(i) pk_real35_dec(i)];
            vkim_dec(i,:) = [pk_im25_dec(i) pk_im30_dec(i) pk_im35_dec(i)];
            vareal_dec(i,:) = [pa_real25_dec(i) pa_real30_dec(i) pa_real35_dec(i)];
            vaim_dec(i,:) = [pa_im25_dec(i) pa_im30_dec(i) pa_im35_dec(i)];
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
        thigh_traj = [thigh_25;thigh_30;thigh_35];
        incline={'in20'};
        knee_reparam = knee_interp(1,:);
        ankle_reparam = ankle_interp(1,:);
        
        leg = ["-25^o", "-30^o", "-35^o", "Unknown (-20^o)"];
        
        strIncline = "-20^o"
        
        knee_ref = knee_20;
        ankle_ref = ankle_20;
        %             load '../Data/filtStairTraj_i20.mat'
    case 2
        thigh_ref = thigh_25;
        thigh_traj = [thigh_20;thigh_30;thigh_35];
        incline={'in25'};
        knee_reparam = knee_interp(2,:);
        ankle_reparam = ankle_interp(2,:);
        
        leg = ["-20^o", "-30^o", "-35^o", "Unknown (-25^o)"];
        
        strIncline = "-25^o"
        
        knee_ref = knee_25;
        ankle_ref = ankle_25;
        %             load '../Data/filtStairTraj_i25.mat'
    case 3
        thigh_ref = thigh_30;
        thigh_traj = [thigh_20;thigh_25;thigh_35];
        incline={'in30'};
        knee_reparam = knee_interp(3,:);
        ankle_reparam = ankle_interp(3,:);
        
        leg = ["-20^o", "-25^o", "-35^o", "Unknown (-30^o)"];
        
        strIncline = "-30^o"
        
        knee_ref = knee_30;
        ankle_ref = ankle_30;
        %             load '../Data/filtStairTraj_i30.mat'
    case 4
        thigh_ref = thigh_35;
        incline={'in35'};
        thigh_traj = [thigh_20;thigh_25;thigh_30];
        knee_reparam = knee_interp(4,:);
        ankle_reparam = ankle_interp(4,:);
        
        strIncline = "-35^o"
        
        leg = ["-20^o", "-25^o", "-30^o", "Unknown (-35^o)"];
        
        knee_ref = knee_35;
        ankle_ref = ankle_35;
        %             load '../Data/filtStairTraj_i35.mat'
    otherwise
        thigh_ref = thigh_20;
        incline={'in20'};
        knee_reparam = knee_interp(1,:);
        ankle_reparam = ankle_interp(1,:);
        
        leg = ["-25^o", "-30^o", "-35^o", "Unknown (-20^o)"];
        
        strIncline = "-20^o"
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

trial={'s1'};
[thigh_s1,knee_s1,ankle_s1] = averageJointKinematics(Normalized,sub,trial,incline);

trial={'s3'};
[thigh_s3,knee_s3,ankle_s3] = averageJointKinematics(Normalized,sub,trial,incline);



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
qh_min = min(thigh_ref);
qh_max = max(thigh_ref);
temp_min = 0;

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
    
    
    
    if qh_min ~= temp_min
        xq = qh_min;
        hk_decvc = .5*interp1(x,vkreal_decvc(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_decvc(N/2,:),xq,'linear','extrap')*cos(pi*N_k*sh);
        ha_decvc = .5*interp1(x,vareal_decvc(1,:),xq,'linear','extrap')+.5*interp1(x,vaim_decvc(N/2,:),xq,'linear','extrap')*cos(pi*N_a*sh);
        
        hk_dec = .5*interp1(x,vkreal_dec(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_dec(N/2,:),xq,'linear','extrap')*cos(pi*N_k*sh);
        ha_dec = .5*interp1(x,vareal_dec(1,:),xq,'linear','extrap')+.5*interp1(x,vaim_dec(N/2,:),xq,'linear','extrap')*cos(pi*N_a*sh);

        hk_vc = .5*pk_real_vc(1) + .5*pk_real_vc(N_k/2)*cos(pi*N_k*sh);
        ha_vc = .5*pa_real_vc(1) + .5*pa_real_vc(N_a/2)*cos(pi*N_a*sh);
        
        hk = .5*pk_real(1) + .5*pk_real(N_k/2)*cos(pi*N_k*sh);
        ha = .5*pa_real(1) + .5*pa_real(N_a/2)*cos(pi*N_a*sh);
        
        for k = 1:N_k/2-1
            hk = hk + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
            hk_vc = hk_vc + pk_real_vc(k+1)*cos(2*pi*(k)*sh)-pk_im_vc(k+1)*sin(2*pi*(k)*sh);
            hk_decvc = hk_decvc + interp1(x,vkreal_decvc(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_decvc(k+1,:),xq,'spline','extrap')*sin(2*pi*(k)*sh);
            hk_dec = hk_dec + interp1(x,vkreal_dec(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_dec(k+1,:),xq,'spline','extrap')*sin(2*pi*(k)*sh);
        end
        
        for a = 1:N_a/2-1
            ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
            ha_vc = ha_vc + pa_real_vc(a+1)*cos(2*pi*(a)*sh)-pa_im_vc(a+1)*sin(2*pi*(a)*sh);
            ha_decvc = ha_decvc + interp1(x,vareal_decvc(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_decvc(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
            ha_dec = ha_dec + interp1(x,vareal_dec(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_dec(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
        end
        temp_max = qh_max;
    end
    knee_decvc_est(i) = double(subs(hk_decvc, sh, s(i)));
    ankle_decvc_est(i) = double(subs(ha_decvc, sh, s(i)));
    
    knee_dec_est(i) = double(subs(hk_dec, sh, s(i)));
    ankle_dec_est(i) = double(subs(ha_dec, sh, s(i)));
    
%     knee_vc_est(i) = double(subs(hk_vc, sh, s(i)));
%     ankle_vc_est(i) = double(subs(ha_vc, sh, s(i)));
%     
%     knee_est(i) = double(subs(hk, sh, s(i)));
%     ankle_est(i) = double(subs(ha, sh, s(i)));
   
end



samples = 1:length(thigh_stream);

figure
sgtitle(strcat("Decline Stair Walking Cross Validation: Leave Out ",num2str(strIncline)))
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

plot(gc, knee_stream,'linewidth',2)
hold on
plot(gc, knee_decvc_est, '--','linewidth',2)
plot(gc, knee_dec_est, ':','color','black','linewidth',2)
% xlabel('samples')
ylabel('Knee Angle (^o)')
title('Knee Joint Position')
legend('Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
grid on
kneeRMSE_incvc = RMSE(knee_decvc_est,knee_stream)
kneeRMSE_inc = RMSE(knee_dec_est,knee_stream)

subplot(414)

plot(gc, ankle_stream,'linewidth',2)
hold on
plot(gc, ankle_decvc_est,'--','linewidth',2)
plot(gc, ankle_dec_est,':','color','black','linewidth',2)
legend('Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
ylabel('Ankle Angle (^o)')
xlabel('Gait Cycle (%)','FontWeight','bold')
title('Ankle Joint Position')
% ankleRMSE_incvc = RMSE(ankle_incvc_est,ankle_stream)
% ankleRMSE_inc = RMSE(ankle_inc_est,ankle_stream)
grid on















