

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
incline={'in35'};

[thigh_mean, knee_mean, ankle_mean, thigh_sd, knee_sd, ankle_sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);


thighd_mean = ddt(thigh_mean);
prevPV = 0;
prevState = 1;
sm = 0;
qhm = 0;
qh_max = max(thigh_mean);
qh_min = min(thigh_mean);

c = ;
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


figure
plot(t,-thigh_mean)
max(-thigh_mean)

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

%     pv = unique(pv,'stable')
%     pv = smooth(interp1(1:length(pv), pv, 1:length(pv)/151:length(pv)))';

T = interp1(pv,t/100,pv,'linear','extrap');
knee_interp = interp1(pv,knee_mean,T,'linear','extrap');
ankle_interp = interp1(pv, ankle_mean, T,'linear','extrap');

L = length(knee_interp);
fs = 150;


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


knee_est = double(subs(hk, sh, pv));
ankle_est = double(subs(ha,sh,pv));
knee_ref = knee_mean;
ankle_ref = ankle_mean;


figure
plot(t,thigh_mean)
hold on
plot(t(xpeak),thigh_mean(xpeak), '*')
plot(t(xtrough),thigh_mean(xtrough),'*')
legend('Traj','MHF','MHE')
grid on



