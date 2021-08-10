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



mean_max = mean([max(thigh_20), max(thigh_25), max(thigh_30), max(thigh_35)])
mean_min = mean([min(thigh_20), min(thigh_25), min(thigh_30), min(thigh_35)])
t = linspace(0,1,length(thigh_20));
c_20 = t(find(thigh_20 == min(thigh_20)));
c_25 = t(find(thigh_25 == min(thigh_25)));
c_30 = t(find(thigh_30 == min(thigh_30)));
c_35 = t(find(thigh_35 == min(thigh_35)));
c = mean([c_20 c_25 c_30 c_35])

t = linspace(0,100,length(thigh_20));
figure
plot(t, thigh_20)
hold on
plot(t, thigh_25)
plot(t, thigh_30)
plot(t, thigh_35)

L = length(thigh_20);



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
    
    %     knee_mean = knee_mean;
    %     thigh_mean = thigh_mean;
    %     ankle_mean = ankle_mean;
    thighd_mean = ddt(thigh_mean);
    prevPV = 0;
    prevState = 1;
    sm = 0;
    qhm = 0;
    qh_max = max(thigh_mean);
    qh_min = min(thigh_mean);
    c = t(find(thigh_mean== qh_min))/100;
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
    pv_swing_thresh = .96;
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
    
    T(ind,:) = interp1(pv,t/100,pv,'linear','extrap');
    knee_interp(ind,:) = interp1(pv,knee_mean,T(ind,:),'linear','extrap');
    ankle_interp(ind,:) = interp1(pv, ankle_mean, T(ind,:),'linear','extrap');
    knee_traj(ind,:) = knee_mean;
    ankle_traj(ind,:) = ankle_mean;
end

% Crate Traj
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


figure
plot(t, knee_20_dec)
hold on
plot(t, knee_25_dec)
plot(t, knee_30_dec)
plot(t,knee_20,'--')
plot(t,knee_25,'--')
plot(t,knee_30,'--')



% Create Coefficients
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

N = length(t);
for i = 1:N/2
    vkreal_decvc(i,:) = [pk_real20_decvc(i) pk_real25_decvc(i) pk_real30_decvc(i)];
    vkim_decvc(i,:) = [pk_im20_decvc(i) pk_im25_decvc(i) pk_im30_decvc(i)];
    vareal_decvc(i,:) = [pa_real20_deccvc(i) pa_real25_decvc(i) pa_real30_decvc(i)];
    vaim_decvc(i,:) = [pa_im20_decvc(i) pa_im25_decvc(i) pa_im30_decvc(i)];
    
    vkreal_dec(i,:) = [pk_real20_dec(i) pk_real25_dec(i) pk_real30_dec(i)];
    vkim_dec(i,:) = [pk_im20_dec(i) pk_im25_dec(i) pk_im30_dec(i)];
    vareal_dec(i,:) = [pa_real20_dec(i) pa_real25_dec(i) pa_real30_dec(i)];
    vaim_dec(i,:) = [pa_im20_dec(i) pa_im25_dec(i) pa_im30_dec(i)];
end


thigh_ref = thigh_35;
knee_ref = knee_35;
ankle_ref = ankle_35;

thigh_sd =thigh_35sd;
ankle_sd =ankle_35sd;
knee_sd =knee_35sd;

thigh_traj = [thigh_20;thigh_25;thigh_30];
incline={'in35'};
knee_reparam = knee_interp(4,:);
ankle_reparam = ankle_interp(4,:);

leg = ["-20^o", "-25^o", "-30^o", "Unknown (-35^o)"];

strIncline = "-35^o";


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

thigh_stream = thigh_ref;%thigh_s3'% smooth([thigh_s1 thigh_s3],10)';
knee_stream = knee_ref;%knee_s3' % smooth([knee_s1 knee_s3],10)';
ankle_stream = ankle_ref; % smooth([ankle_s1 ankle_s3],10)';

%PV params
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

qh_max = max(thigh_stream);
qh_min = min(thigh_stream);
q_po = 7;

%peak detection parameters
%threshold and min distance
% thresh = .00001;
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
pv_swing_thresh = .94;
ytrough = [];
xtrough = [];
current_min = 0;

xq = qh_min;
syms sh

for i = 1:length(t)
    
    thigh = thigh_stream(i);
    thighd = thighd_stream(i);
    
    
    if current_min ~= qh_min
        
        xq = qh_min;
        N_k = 14;
        N_a = 14;
        hk_decvc = .5*interp1(x,vkreal_decvc(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_decvc(N_k/2+1,:),xq,'linear','extrap')*cos(pi*N_k*sh);
        ha_decvc = .5*interp1(x,vareal_decvc(1,:),xq,'linear','extrap')+.5*interp1(x,vaim_decvc(N_a/2+1,:),xq,'linear','extrap')*cos(pi*N_a*sh);
        
        hk_dec = .5*interp1(x,vkreal_dec(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_dec(N_k/2+1,:),xq,'linear','extrap')*cos(pi*N_k*sh);
        ha_dec = .5*interp1(x,vareal_dec(1,:),xq,'linear','extrap')+.5*interp1(x,vaim_dec(N_a/2+1,:),xq,'linear','extrap')*cos(pi*N_a*sh);
        
        hk_vc = .5*pk_real_vc(1) + .5*pk_real_vc(N_k/2)*cos(pi*N_k*sh);
        ha_vc = .5*pa_real_vc(1) + .5*pa_real_vc(N_a/2)*cos(pi*N_a*sh);
        
        hk = .5*pk_real(1) + .5*pk_real(N_k/2)*cos(pi*N_k*sh);
        ha = .5*pa_real(1) + .5*pa_real(N_a/2)*cos(pi*N_a*sh);
        
        for k = 1:N_k/2-1
            hk = hk + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
            hk_vc = hk_vc + pk_real_vc(k+1)*cos(2*pi*(k)*sh)-pk_im_vc(k+1)*sin(2*pi*(k)*sh);
            hk_decvc = hk_decvc + interp1(x,vkreal_decvc(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_decvc(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
            hk_dec = hk_dec + interp1(x,vkreal_dec(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_dec(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
        end
        
        for a = 1:N_a/2-1
            ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
            ha_vc = ha_vc + pa_real_vc(a+1)*cos(2*pi*(a)*sh)-pa_im_vc(a+1)*sin(2*pi*(a)*sh);
            ha_decvc = ha_decvc + interp1(x,vareal_decvc(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_decvc(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
            ha_dec = ha_dec + interp1(x,vareal_dec(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_dec(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
        end
        
        
        current_min = xq;
        
    end
    
    
    
    if prevState == 2|| prevState == 3
        temp_traj_MHE = [temp_traj_MHE -thigh];
        if length(temp_traj_MHE) > 3
            [pks_MHE,locs_MHE] = findpeaks(temp_traj_MHE,'threshold', thresh,'MinPeakHeight',-minpeakh_mhe);
            if ~isempty(pks_MHE)
                temp_traj_MHE = [];
                ytrough = -[ytrough pks_MHE];
                xtrough = [xtrough i-1];
                qh_min = mean(ytrough);
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
                    qh_max = mean([ypeak max(thigh_stream)]);
                    mhf = 1
                end
            end
            
        end
    end
    %
    [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
    %
    
    
    s(i) = currPV;
    prevState = currState;
    state(i) = currState;
    prevPV = currPV;
    
    knee_decvc_est(i) = double(subs(hk_decvc, sh, s(i)));
    ankle_decvc_est(i) = double(subs(ha_decvc, sh, s(i)));
    
    knee_dec_est(i) = double(subs(hk_dec, sh, s(i)));
    ankle_dec_est(i) = double(subs(ha_dec, sh, s(i)));
    
end

% xq = qh_min;
%
% syms sh
%
% N_k = 12;
% N_a = 12;
% hk_decvc = .5*interp1(x,vkreal_decvc(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_decvc(N_k/2,:),xq,'linear','extrap')*cos(pi*N_k*sh);
% ha_decvc = .5*interp1(x,vareal_decvc(1,:),xq,'linear','extrap')+.5*interp1(x,vaim_decvc(N_a/2,:),xq,'linear','extrap')*cos(pi*N_a*sh);
%
% hk_dec = .5*interp1(x,vkreal_dec(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_dec(N_k/2,:),xq,'linear','extrap')*cos(pi*N_k*sh);
% ha_dec = .5*interp1(x,vareal_dec(1,:),xq,'linear','extrap')+.5*interp1(x,vaim_dec(N_a/2,:),xq,'linear','extrap')*cos(pi*N_a*sh);
%
% hk_vc = .5*pk_real_vc(1) + .5*pk_real_vc(N_k/2)*cos(pi*N_k*sh);
% ha_vc = .5*pa_real_vc(1) + .5*pa_real_vc(N_a/2)*cos(pi*N_a*sh);
%
% hk = .5*pk_real(1) + .5*pk_real(N_k/2)*cos(pi*N_k*sh);
% ha = .5*pa_real(1) + .5*pa_real(N_a/2)*cos(pi*N_a*sh);
%
% for k = 1:N_k/2-1
%     hk = hk + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
%     hk_vc = hk_vc + pk_real_vc(k+1)*cos(2*pi*(k)*sh)-pk_im_vc(k+1)*sin(2*pi*(k)*sh);
%     hk_decvc = hk_decvc + interp1(x,vkreal_decvc(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_decvc(k+1,:),xq,'spline','extrap')*sin(2*pi*(k)*sh);
%     hk_dec = hk_dec + interp1(x,vkreal_dec(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_dec(k+1,:),xq,'spline','extrap')*sin(2*pi*(k)*sh);
% end
%
% for a = 1:N_a/2-1
%     ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
%     ha_vc = ha_vc + pa_real_vc(a+1)*cos(2*pi*(a)*sh)-pa_im_vc(a+1)*sin(2*pi*(a)*sh);
%     ha_decvc = ha_decvc + interp1(x,vareal_decvc(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_decvc(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
%     ha_dec = ha_dec + interp1(x,vareal_dec(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_dec(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
% end
%
% knee_decvc_est = double(subs(hk_decvc, sh, s));
% ankle_decvc_est= double(subs(ha_decvc, sh, s));
%
% knee_dec_est = double(subs(hk_dec, sh, s));
% ankle_dec_est = double(subs(ha_dec, sh, s));

gc = linspace(0,100,length(t));
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


subplot(413)


k1 = knee_stream+knee_sd;
k2 = knee_stream-knee_sd;

gc2 = [gc, fliplr(gc)];
inBetween = [k1,fliplr(k2)];
pk1 = fill(gc2,inBetween,[0.3010, 0.7450, 0.9330], 'linestyle','None');
hold on
pk2 = plot(gc, knee_stream,'linewidth',2,'color',[0, 0.4470, 0.7410]);
pk3 = plot(gc, knee_decvc_est, '--','linewidth',2, 'color',[0.8500, 0.3250, 0.0980]);
% pk4 = plot(gc, knee_dec_est, ':','color','black','linewidth',2);
% xlabel('samples')
ylabel('Knee Angle (^o)')
title('Knee Joint Position')
legend([pk2,pk3],'Raw','Est(Inc and VC interp)','Location','northeast')
% legend([pk2,pk3,pk4],'Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
grid on
kneeRMSE_incvc = RMSE(knee_decvc_est,knee_stream)
kneeRMSE_inc = RMSE(knee_dec_est,knee_stream)

subplot(414)

a1 = ankle_stream+ankle_sd;
a2 = ankle_stream-ankle_sd;

gc2 = [gc, fliplr(gc)];
inBetween = [a1,fliplr(a2)];
pa1 = fill(gc2,inBetween,[0.3010, 0.7450, 0.9330], 'linestyle','None');
hold on
pa2 = plot(gc, ankle_stream,'linewidth',2,'color',[0, 0.4470, 0.7410]);
pa3 = plot(gc, ankle_decvc_est,'--','linewidth',2, 'color',[0.8500, 0.3250, 0.0980]);
% pa4 = plot(gc, ankle_dec_est,':','color','black','linewidth',2);
legend([pa2,pa3],'Raw','Est(Inc and VC interp)','Location','northeast')
% legend([pa2,pa3,pa4],'Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
ylabel('Ankle Angle (^o)')
xlabel('Gait Cycle (%)','FontWeight','bold')
title('Ankle Joint Position')
ankleRMSE_incvc = RMSE(ankle_decvc_est,ankle_stream)
ankleRMSE_inc = RMSE(ankle_dec_est,ankle_stream)
grid on


