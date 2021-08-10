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

    knee_traj(ind,:) = knee_mean;
    ankle_traj(ind,:) = ankle_mean;
    
end
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
plot(t, knee_30_dec)
plot(t, knee_35_dec)
plot(t,knee_20,'--')
plot(t,knee_30,'--')
plot(t,knee_35,'--')


pk_real20_dec = real(fft(knee_20_dec)/(L/2));
pk_real30_dec = real(fft(knee_30_dec)/(L/2));
pk_real35_dec = real(fft(knee_35_dec)/(L/2));
pk_im20_dec = imag(fft(knee_20_dec)/(L/2));
pk_im30_dec = imag(fft(knee_30_dec)/(L/2));
pk_im35_dec = imag(fft(knee_35_dec)/(L/2));

N = length(t);
for i = 1:N/2
    vkreal_dec(i,:) = [pk_real20_dec(i) pk_real30_dec(i) pk_real35_dec(i)];
    vkim_dec(i,:) = [pk_im20_dec(i) pk_im30_dec(i) pk_im35_dec(i)];
end

thigh_ref = thigh_25;
knee_ref = knee_25;
ankle_ref = ankle_25;

thigh_sd =thigh_25sd;
ankle_sd =ankle_25sd;
knee_sd =knee_25sd;

thigh_traj = [thigh_20;thigh_30;thigh_35];
incline={'in25'};
% knee_reparam = knee_interp(2,:);
% ankle_reparam = ankle_interp(2,:);

leg = ["-20^o", "-30^o", "-35^o", "Unknown (-25^o)"];

strIncline = "-25^o";

% calculate knee
Y_k = fft(knee_ref);
L = length(Y_k);
pk_real = real(Y_k/(L/2));
pk_im = imag(Y_k/(L/2));

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
pv_swing_thresh = .90;
ytrough = [];
xtrough = [];
current_min = 0;

xq = qh_min;
syms sh


s_0 = linspace(0,1,150);

N_k = 20;


xq = qh_min;
x = [min(thigh_20) min(thigh_30) min(thigh_35)];


figure
plot(t, thigh_stream,'--')
hold on
plot(t, thigh_20)
plot(t, thigh_30)
plot(t, thigh_35)

for i = 1:length(t)
    
    thigh = thigh_stream(i);
    thighd = thighd_stream(i);
    
    
%     if current_min ~= qh_min
%         
% %         xq = current_min;
%         
%         
% %         for a = 1:N_a/2-1
% %             ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
% %             ha_vc = ha_vc + pa_real_vc(a+1)*cos(2*pi*(a)*sh)-pa_im_vc(a+1)*sin(2*pi*(a)*sh);
% %             ha_decvc = ha_decvc + interp1(x,vareal_decvc(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_decvc(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
% %             ha_dec = ha_dec + interp1(x,vareal_dec(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_dec(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
% %         end
% %         
% %         
%         current_min = qh_min;
%         
%     end
    
    
    
%     if prevState == 2|| prevState == 3
%         temp_traj_MHE = [temp_traj_MHE -thigh];
%         if length(temp_traj_MHE) > 3
%             [pks_MHE,locs_MHE] = findpeaks(temp_traj_MHE,'threshold', thresh,'MinPeakHeight',-minpeakh_mhe);
%             if ~isempty(pks_MHE)
%                 temp_traj_MHE = [];
%                 ytrough = -[ytrough pks_MHE];
%                 xtrough = [xtrough i-1];
%                 qh_min = mean(ytrough);
%                 mhe = 1;
%                 
%             end
%         end
        
        
%     end
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
    
%     knee_decvc_est(i) = double(subs(hk_decvc, sh, s(i)));
%     ankle_decvc_est(i) = double(subs(ha_decvc, sh, s(i)));
    
   
%     ankle_dec_est(i) = double(subs(ha_dec, sh, s(i)));
    
end

 
 
 

% figure
% plot(s,knee_dec_est);



N_k = 20;

hk_dec0 = .5*interp1(x,vkreal_dec(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_dec((N_k/2)+1,:),xq,'linear','extrap')*cos(pi*N_k*sh);
hk_dec = .5*interp1(x,vkreal_dec(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_dec((N_k/2)+1,:),xq,'linear','extrap')*cos(pi*N_k*sh);

for k = 1:N_k/2-1
    hk_dec = hk_dec + interp1(x,vkreal_dec(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_dec(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
    hk_dec0 = hk_dec0 + interp1(x,vkreal_dec(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_dec(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
end

knee_dec_est0 = double(subs(hk_dec0, sh, s_0));
knee_dec_est = double(subs(hk_dec, sh, s));

figure
plot(t,s)


figure
plot(t,knee_stream)
hold on
plot(t,knee_dec_est0,'--')

plot(t,knee_dec_est,'*')