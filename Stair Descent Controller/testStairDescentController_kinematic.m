clearvars -except Streaming Normalized stairAscentTrialData R01 rawR01 estimatedAngles;

close all
addpath("Utility Functions")
addpath("../Data")
if ~exist('Normalized')
    load '../Data/Normalized.mat'
end

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};

trial={'s3'};

incline={'in30'};
[thigh_pos_d30, knee_pos_d30, ankle_pos_d30, thigh_pos_d30sd, knee_pos_d30sd, ankle_pos_d30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline); 

[thigh_vel_d30, knee_vel_d30, ankle_vel_d30, thigh_vel_d30sd, knee_vel_d30sd, ankle_vel_d30sd] = averageJointVelocities_Stair(Normalized, sub, trial, incline);




t = linspace(0,1,length(thigh_pos_d30));



% thigh_pos_d30_shift = smooth(circshift(thigh_pos_d30,-find(thigh_pos_d30 == max(thigh_pos_d30))),25);
thigh_vel_d30_shift = circshift(thigh_vel_d30,-find(thigh_pos_d30 == max(thigh_pos_d30)));

dt_orig = 1/100;
samp_orig = 135;
time_orig = linspace(0,samp_orig*dt_orig-dt_orig,samp_orig);
L = 150+1;
time_normalized = interp1(1:length(time_orig), time_orig, 1:length(time_orig)/(L):length(time_orig));
dt_normalized = mode(diff(time_normalized));

filt_freq = 5 ; %Hz
samprate = 1/mode(dt_normalized); % Hz
[B_f,A_f] = butter(2,filt_freq/(samprate/2)) ; 


thigh_pos_d30_shift = filtfilt(B_f,A_f,circshift(thigh_pos_d30,-find(thigh_pos_d30 == max(thigh_pos_d30))));

filt_freq = 6; %Hz
samprate = 1/mode(dt_normalized); % Hz
[B_f,A_f] = butter(2,filt_freq/(samprate/2)) ; 

figure
plot(t, circshift(knee_pos_d30,-find(thigh_pos_d30 == max(thigh_pos_d30))));
hold on
plot(t,filtfilt(B_f,A_f,circshift(knee_pos_d30,-find(thigh_pos_d30 == max(thigh_pos_d30)))));


knee_pos_d30_shift = filtfilt(B_f,A_f,circshift(knee_pos_d30,-find(thigh_pos_d30 == max(thigh_pos_d30))));

filt_freq = 7; %Hz
samprate = 1/mode(dt_normalized); % Hz
[B_f,A_f] = butter(2,filt_freq/(samprate/2)) ; 

figure
plot(t, circshift(ankle_pos_d30,-find(thigh_pos_d30 == max(thigh_pos_d30))));
hold on
plot(t,filtfilt(B_f,A_f,circshift(ankle_pos_d30,-find(thigh_pos_d30 == max(thigh_pos_d30)))));


ankle_pos_d30_shift = filtfilt(B_f,A_f,circshift(ankle_pos_d30,-find(thigh_pos_d30 == max(thigh_pos_d30))));


c_d30 = t(find(thigh_pos_d30_shift == min(thigh_pos_d30_shift)));

s_po = .55;
pv_swing_thresh = .96;
c= c_d30;

prevPV = 0;
prevState = 1;
sm = 0;
qhm = 0;
qh_max = max(thigh_pos_d30_shift);
qh_min = min(thigh_pos_d30_shift);

%MHF Detection Params
thresh = .00001;
minpeakh = 15;
minpeakd = 50;
mhf = 0;
temp_traj = [];
ypeak = [];
xpeak = [];

pv = zeros(1,length(thigh_pos_d30_shift));
state = ones(1,length(thigh_pos_d30_shift));

for i = 1:length(t)
        
    thigh = thigh_pos_d30_shift(i);
    thighd = thigh_vel_d30_shift(i);
    
    %MHF Detection
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
    
    
    %Calculate Phase
    
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_StairDescent_Ratcheting(thigh, thighd, qh_min, qh_max,s_po, c, prevState,prevPV, sm, qhm,mhf,pv_swing_thresh);
    
    pv(i) = currPV;
    state(i) = currState;
    
    prevState = currState;
    prevPV = currPV;
end

gc = t*100;
figure
subplot(131)
plot(gc,thigh_pos_d30_shift)
yyaxis right
plot(gc,state)

subplot(132)
plot(gc,pv)
hold on
yyaxis right
plot(gc,state)

subplot(133)
plot(gc,thigh_vel_d30_shift)


knee_pos_d30_reparam = interp1(pv,knee_pos_d30_shift,t,'linear','extrap');

Y_k = fft(knee_pos_d30_reparam);
L = length(Y_k);
pk_real = real(Y_k/(L/2));
pk_im = imag(Y_k/(L/2));

N_k = 14;

syms s
hk = .5*pk_real(1) + .5*pk_real(N_k/2+1)*cos(pi*N_k*s);

for k = 1:N_k/2-1

    hk = hk + (pk_real(k+1)*cos(2*pi*k*s) - pk_im(k+1)*sin(2*pi*k*s));
    
end


knee_pos_d30_est = double(subs(hk,s,pv));


figure
plot(gc,knee_pos_d30_shift)
hold on
plot(gc,knee_pos_d30_est)
title("Knee VC")
ylabel("Knee Angle (^o)")
xlabel("Gait Cycle (%)")
legend('AB','Est')

figure
ax(1) = subplot(131);
plot(gc,pv);
ylabel("Phase")
xlabel("Gait Cycle (%)")
ax(2) = subplot(132);
plot(gc,ddt(deg2rad(knee_pos_d30_est),dt_normalized));
ylabel("Est Knee Vel")
xlabel("Gait Cycle (%)")
ax(3) = subplot(133);
plot(gc,ddt(deg2rad(thigh_pos_d30_shift),dt_normalized));
ylabel("Thigh Vel (rad/s)")
xlabel("Gait Cycle (%)")
linkaxes(ax,'x')
% plot(t,knee_pos_d30_reparam)

