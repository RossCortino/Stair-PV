clearvars -except Streaming Normalized R01 rawR01;
close all
clc;


% sub={'AB01','AB02','AB03', 'AB04', 'AB05', 'AB06', 'AB07', 'AB08', 'AB09', 'AB10'};
%
% percentGait=linspace(0,1,150);
% trial={'s3'};
%
% incline={'i20'};
% [thigh_mean,knee_mean,ankle_mean] = averageJointKinematics(R01,sub,trial,incline);
% shift = find(thigh_mean == max(thigh_mean));
%
% thigh_mean = circshift(thigh_mean,-shift);
% knee_mean = circshift(knee_mean,-shift);
current_incline = '20^o';

load 'Data/filtStairTraj_i20.mat'
knee_mean = knee_mean';
thigh_mean = thigh_mean';

thighd_mean = ddt(thigh_mean);
prevPV = 0;
prevState = 1;
sm = 0;
qhm = 0;
qh_max = max(thigh_mean);
qh_min = min(thigh_mean);


pv = zeros(1,length(thigh_mean));
t = linspace(0,100,150);
c = t(find(thigh_mean == qh_min))/100;



for i = 1:length(t)
    
    thigh = thigh_mean(i);
    thighd = thighd_mean(i);
    
    
    [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair_Normalized(thigh, thighd, qh_min, qh_max, c, prevState,prevPV, sm, qhm);
    
    
    
    
    
    pv(i) = currPV;
    prevState = currState;
    prevPV = currPV;
    
end




pv = unique(pv,'stable')
pv = smooth(interp1(1:length(pv), pv, 1:length(pv)/151:length(pv)))';
T = griddedInterpolant(pv,t/100,'linear','linear');
figure
plot(t,pv)


knee_interp = interp1(pv,knee_mean,T(pv),'linear','extrap');
ankle_interp = interp1(pv, ankle_mean, T(pv),'linear','extrap');

L = length(knee_interp)
fs = 150
f = fs*(0:(L/2))/L;
fc = 8;

[b,a] = butter(2,fc/(fs/2),'low');

knee_interp = filtfilt(b,a,knee_interp);


% calculate knee
Y_k = fft(knee_interp);
L = length(Y_k);
pk_real = real(Y_k/(L/2));
pk_im = imag(Y_k/(L/2));


fc = 8;

[b,a] = butter(2,fc/(fs/2),'low');

ankle_interp = filtfilt(b,a,ankle_interp);

% calculate ankle
Y_a = fft(ankle_interp);
L = length(Y_a);
f = 116*(0:(L/2))/L;
pa_real = real(Y_a/(L/2));
pa_im = imag(Y_a/(L/2));

sh = sym('sh');

N_k = 20;
N_a = 26;
hk = .5*pk_real(1) + .5*pk_real(N_k/2)*cos(pi*N_k*sh);
ha = .5*pa_real(1) + .5*pa_real(N_a/2)*cos(pi*N_a*sh);

for k = 1:N_k/2-1
    hk = hk + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
end


knee_est = smooth(double(subs(hk, sh, pv)));



for a = 1:N_a/2-1
    ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
end

ankle_est = double(subs(ha,sh,pv));
knee_ref = knee_mean;
ankle_ref = ankle_mean;

krmse = sqrt(sum((knee_est-knee_ref).^2)/L)
armse = sqrt(sum((ankle_est-ankle_ref).^2)/L)

pvcolors = [ 70 130 180
    30 144 255
    0 191 255
    135 206 235
    ]./255;

figure

colororder(pvcolors)
%20
p1 = plot(t,pv,'Linewidth',2);
hold on
xlabel('Gait Cycle (%)')
ylabel('Phase')
grid on


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
pk11 = plot(t,knee_est,'Linewidth',2);
hold on
pk12 = plot(t,knee_ref,'--','Linewidth',2);
grid on
xlabel('Gait Cycle (%)')
ylabel('Knee Angle (^o)')
legend([pk12,pk11],{'Reference','Measured'},'location','northwest')


figure
colororder(newcolors)
%20
plot(t,ankle_est,'Linewidth',2)
hold on
plot(t,ankle_ref,'--','Linewidth',2)
grid on
hold off

%% Duplicate with 