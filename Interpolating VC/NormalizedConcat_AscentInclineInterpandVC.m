
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
    
% prompt = 'Incline Interp(1), VC Reparam(2), Both(3)';
% z = input(prompt);

load '../Data/filtStairTraj_i20.mat'

thigh_20 = thigh_mean;
knee_20 = knee_mean;
ankle_20 = ankle_mean;

load '../Data/filtStairTraj_i25.mat'

thigh_25 = thigh_mean;
knee_25 = knee_mean;
ankle_25 = ankle_mean;

load '../Data/filtStairTraj_i30.mat'

thigh_30 = thigh_mean;
knee_30 = knee_mean;
ankle_30 = ankle_mean;

load '../Data/filtStairTraj_i35.mat'

thigh_35 = thigh_mean;
knee_35 = knee_mean;
ankle_35 = ankle_mean;



L = length(thigh_20);

%% Calculate PV

%% Parameterize Joint Trajectories as functions of Phase
current_incline = '20^o';
for ind = 1:4
    
    switch ind
        case 1
            load '../Data/filtStairTraj_i20'
            current_incline = '20^o';
        case 2
            load '../Data/filtStairTraj_i25'
            current_incline = '25^o';
        case 3
            load '../Data/filtStairTraj_i30'
            current_incline = '30^o';
        case 4
            load '../Data/filtStairTraj_i35'
            current_incline = '35^o';
        otherwise
            load '../Data/filtStairTraj_i20'
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
    ix = 1:100;
    q_po = 10 ;%thigh_mean(find(ankle_mean(ix)==max(ankle_mean(ix))))
    
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
    
    
    figure
    plot(t,pv)
    xlabel('Normalized Time')
    ylabel('Phase Variable')
    
    figure
    plot(t,thigh_mean,t,ankle_mean)
    
    pv = unique(pv,'stable');
    pv = smooth(interp1(1:length(pv), pv, 1:length(pv)/151:length(pv)))';
    
    T(ind,:) = interp1(pv,t/100,pv,'linear','extrap');
    knee_interp(ind,:) = interp1(pv,knee_mean,T(ind,:),'spline','extrap');
    ankle_interp(ind,:) = interp1(pv, ankle_mean, T(ind,:),'spline','extrap');
    knee_traj(ind,:) = knee_mean;
    ankle_traj(ind,:) = ankle_mean;
    
    L = length(knee_interp(ind,:));
    fs = 150;
    %     f = fs*(0:(L/2))/L;
    fc = 4;

    [b,a] = butter(2,fc/(fs/2),'low');

    knee_interp(ind,:) = smooth(knee_interp(ind,:),20);% filtfilt(b,a,knee_interp(ind,:));

    fc = 6;

    [b,a] = butter(2,fc/(fs/2),'low');

    ankle_interp(ind,:) =smooth(ankle_interp(ind,:));% filtfilt(b,a,ankle_interp(ind,:));
    
    pv = [];
    
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

%% Coefficients incline and vc interp
L = length(knee_20);
pk_real20_incvc = real(fft(knee_20_incvc)/(L/2));
pk_real25_incvc = real(fft(knee_25_incvc)/(L/2));
pk_real30_incvc = real(fft(knee_30_incvc)/(L/2));
pk_real35_incvc = real(fft(knee_35_incvc)/(L/2));

pk_im20_incvc = imag(fft(knee_20_incvc)/(L/2));
pk_im25_incvc = imag(fft(knee_25_incvc)/(L/2));
pk_im30_incvc = imag(fft(knee_30_incvc)/(L/2));
pk_im35_incvc = imag(fft(knee_35_incvc)/(L/2));


N = 150;
x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vkreal_incvc(i,:) = [pk_real20_incvc(i) pk_real25_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i)];
    vkim_incvc(i,:) = [pk_im20_incvc(i) pk_im25_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i)];
end

% Ankle Coefficients
pa_real20_incvc = real(fft(ankle_20_incvc)/(L/2));
pa_real25_incvc = real(fft(ankle_25_incvc)/(L/2));
pa_real30_incvc = real(fft(ankle_30_incvc)/(L/2));
pa_real35_incvc = real(fft(ankle_35_incvc)/(L/2));

pa_im20_incvc = imag(fft(ankle_20_incvc)/(L/2));
pa_im25_incvc = imag(fft(ankle_25_incvc)/(L/2));
pa_im30_incvc = imag(fft(ankle_30_incvc)/(L/2));
pa_im35_incvc = imag(fft(ankle_35_incvc)/(L/2));

N = 150;
x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vareal_incvc(i,:) = [pa_real20_incvc(i) pa_real25_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i)];
    vaim_incvc(i,:) = [pa_im20_incvc(i) pa_im25_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i)];
end

%% Incline Interp version
L = length(knee_20);
pk_real20_inc = real(fft(knee_20_inc)/(L/2));
pk_real25_inc = real(fft(knee_25_inc)/(L/2));
pk_real30_inc = real(fft(knee_30_inc)/(L/2));
pk_real35_inc = real(fft(knee_35_inc)/(L/2));

pk_im20_inc = imag(fft(knee_20_inc)/(L/2));
pk_im25_inc = imag(fft(knee_25_inc)/(L/2));
pk_im30_inc = imag(fft(knee_30_inc)/(L/2));
pk_im35_inc = imag(fft(knee_35_inc)/(L/2));


N = 150;
x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vkreal_inc(i,:) = [pk_real20_inc(i) pk_real25_inc(i) pk_real30_inc(i) pk_real35_inc(i)];
    vkim_inc(i,:) = [pk_im20_inc(i) pk_im25_inc(i) pk_im30_inc(i) pk_im35_inc(i)];
end

% Ankle Coefficients
pa_real20_inc = real(fft(ankle_20_inc)/(L/2));
pa_real25_inc = real(fft(ankle_25_inc)/(L/2));
pa_real30_inc = real(fft(ankle_30_inc)/(L/2));
pa_real35_inc = real(fft(ankle_35_inc)/(L/2));

pa_im20_inc = imag(fft(ankle_20_inc)/(L/2));
pa_im25_inc = imag(fft(ankle_25_inc)/(L/2));
pa_im30_inc = imag(fft(ankle_30_inc)/(L/2));
pa_im35_inc = imag(fft(ankle_35_inc)/(L/2));

N = 150;
x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vareal_inc(i,:) = [pa_real20_inc(i) pa_real25_inc(i) pa_real30_inc(i) pa_real35_inc(i)];
    vaim_inc(i,:) = [pa_im20_inc(i) pa_im25_inc(i) pa_im30_inc(i) pa_im35_inc(i)];
end



%% Subject Streaming Data Test
close all
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};

% percentGait=linspace(0,1,150);


trial={'s3'};
incline={'i25'};

switch incline{1}
        case 'i20'
            thigh_ref = thigh_20;
            
            knee_reparam = knee_interp(1,:);
            ankle_reparam = ankle_interp(1,:);
            
            knee_ref = knee_20;
            ankle_ref = ankle_20;
%             load '../Data/filtStairTraj_i20.mat'
        case 'i25'
            thigh_ref = thigh_25;
            
            knee_reparam = knee_interp(2,:);
            ankle_reparam = ankle_interp(2,:);
            
            knee_ref = knee_25;
            ankle_ref = ankle_25;
%             load '../Data/filtStairTraj_i25.mat'
        case 'i30'
            thigh_ref = thigh_30;
            
            knee_reparam = knee_interp(3,:);
            ankle_reparam = ankle_interp(3,:);
            
            knee_ref = knee_30;
            ankle_ref = ankle_30;
%             load '../Data/filtStairTraj_i30.mat'
        case 'i35'
            thigh_ref = thigh_35;
            knee_reparam = knee_interp(4,:);
            ankle_reparam = ankle_interp(4,:);
            
            knee_ref = knee_35;
            ankle_ref = ankle_35;
%             load '../Data/filtStairTraj_i35.mat'
        otherwise
            thigh_ref = thigh_20;
            knee_reparam = knee_interp(1,:);
            ankle_reparam = ankle_interp(1,:);
            
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

sub={'AB01','AB02','AB03','AB04','AB05'};

trial={'s1'};
[thigh_s1,knee_s1,ankle_s1] = averageJointKinematics(Normalized,sub,trial,incline);

trial={'s3'};
[thigh_s3,knee_s3,ankle_s3] = averageJointKinematics(Normalized,sub,trial,incline);



thigh_stream = thigh_s3'% smooth([thigh_s1 thigh_s3],10)';
knee_stream = knee_s3' % smooth([knee_s1 knee_s3],10)';
ankle_stream = ankle_s3' % smooth([ankle_s1 ankle_s3],10)';

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
temp_max = 0;
N_k = 20;
N_a = 20;
sh = sym('sh');
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
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max, c, prevState,prevPV, sm, qhm,mhf);
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
    
    knee_vc_est(i) = double(subs(hk_vc, sh, s(i)));
    ankle_vc_est(i) = double(subs(ha_vc, sh, s(i)));
    
    knee_est(i) = double(subs(hk, sh, s(i)));
    ankle_est(i) = double(subs(ha, sh, s(i)));
   
end



%     L = length(knee_est);
%     fs = 150;
%     %     f = fs*(0:(L/2))/L;
%     fc = 6;
% 
%     [b,a] = butter(2,fc/(fs/2),'low');
% 
%     knee_filt = filtfilt(b,a,knee_est);
% 
%     fc = 6;
% 
%     [b,a] = butter(2,fc/(fs/2),'low');
% 
%     ankle_filt = filtfilt(b,a,ankle_est);





samples = 1:length(thigh_stream);
% figure
% subplot(211)
% plot(samples,thigh_stream)
% hold on
% plot(xpeak,ypeak,'*')
% ylabel('Angle (^o)')
% title('Thigh Angle')
% 
% subplot(212)
% yyaxis left
% plot(samples,s,'k')
% ylabel('Phase Variable')
% yyaxis right
% plot(samples,state)
% ylabel('State')
% title('Phase Variable Real Time detection')
% 
% figure
% yyaxis left
% plot(samples,s,'k')
% ylabel('Phase Variable')
% yyaxis right
% plot(samples,state)
% ylabel('State')
% title('Phase Variable Real Time detection')
% 
% figure 
% plot(samples,thigh_stream, 'linewidth',2)
% hold on
% plot(xpeak,ypeak,'*')
% ylabel('Angle (^o)')

figure
sgtitle('Incline Example: Mean of first five subjects')
% subplot(411)
% plot(samples,thigh_stream)
% hold on
% plot(xpeak,ypeak,'*')
% ylabel('Angle (^o)')
% title('Thigh Angle')

subplot(311)
yyaxis left
plot(samples,s,'k')
ylabel('Phase Variable')
yyaxis right
plot(samples,state)
ylabel('State')

kfiltcoeff = .3;
afiltcoeff = .5;
subplot(312)

plot(samples, knee_stream,'linewidth',2)
hold on
plot(samples, knee_incvc_est, '--','linewidth',2)
plot(samples, knee_inc_est, '--','linewidth',2)
plot(samples, knee_vc_est, ':','linewidth',2)
plot(samples, knee_est, ':','linewidth',2)
% xlabel('samples')
ylabel('Knee Angle (^o)')
title('Knee Angle')
legend('Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Est(VC Interp Only)','Est(No interp)','Location','Southeast')

subplot(313)

plot(samples, ankle_stream,'linewidth',2)
hold on
plot(samples, ankle_incvc_est,'--','linewidth',2)
plot(samples, ankle_inc_est,'--','linewidth',2)
plot(samples, ankle_vc_est, ':','linewidth',2)
plot(samples, ankle_est, ':','linewidth',2)
legend('Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Est(VC Interp Only)','Est(No interp)','Location','Southeast')

ylabel('Ankle Angle (^o)')
















