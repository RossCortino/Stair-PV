clearvars -except Streaming Normalized stairAscentTrialData R01 rawR01 estimatedAngles;

close all
addpath("Utility Functions")

if ~exist('Normalized')
    load '../Data/Normalized.mat'
end
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s3'};
incline={'i30'};
[thigh_30, knee_30, ankle_30, thigh_30sd, knee_30sd, ankle_30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);



t = linspace(0,1, length(thigh_30));
thigh_30d = ddt(thigh_30);

pv_swing_thresh = .65;

thigh_30_shift = smooth(circshift(thigh_30,-(find(thigh_30 == max(thigh_30)))));
knee_30_shift = circshift(knee_30,-(find(thigh_30 == max(thigh_30))));
c_shift = t(find(thigh_30_shift == min(thigh_30_shift)));

thigh_30d = ddt(thigh_30_shift);

q_po = -4;
prevState = 1;
prevPV = 0;

qh_max = max(thigh_30);
qh_min = min(thigh_30);
mhf = 0;
pv = [];
sm = 0;
qhm = 0;

for i = 1:length(t)
    
    thigh = thigh_30_shift(i);
    thighd = thigh_30d(i);
    
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c_shift, prevState,prevPV, sm, qhm,mhf,pv_swing_thresh)
    pv(i) = currPV;
    prevState = currState;
    prevPV = currPV;
end

gc = linspace(0,100,length(t));


figure

plot(gc, thigh_30_shift,'linewidth',2)
set(gca,'FontSize',15)
xlabel('Gait Cycle (%)')
ylabel('Thigh Angle (^o)')
grid on

figure

plot(gc, pv,'linewidth',2)
set(gca,'FontSize',15)
xlabel('Gait Cycle (%)')
ylabel('Phase')
grid on

Y_k = fft(knee_30_shift);
L = length(Y_k);
pk_real = real(Y_k/(L/2));
pk_im = imag(Y_k/(L/2));

N_k = 14;

syms s
hk = .5*pk_real(1) + .5*pk_real(N_k/2+1)*cos(pi*N_k*s);

for k = 1:N_k/2-1

    hk = hk + (pk_real(k+1)*cos(2*pi*k*s) - pk_im(k+1)*sin(2*pi*k*s));
    
end

indigo = [0 119 187]/255;
% blue = [0 119 187]/255;
cyan = [136 204 238]/255;
teal = [68 170 153]/255;
green = [17 119 51]/255;
olive = [153 153 51]/255;
sand = [221 204 119]/255;
rose = [204 102 119]/255;
wine = [136 34 85]/255;
purple = [170 68 153]/255;
palegrey = [221,221,221]/255;



figure
plot(gc, knee_30,'linewidth',2,'color',indigo)
hold on
set(gca,'FontSize',15)
xlabel('Gait Cycle (%)')
ylabel('Knee Angle (^o)')
grid on

figure
plot(gc, knee_30,'--','linewidth',2,'color',indigo)
hold on
plot(gc,circshift(subs(hk,s,pv),(find(thigh_30 == max(thigh_30)))),'linewidth',2,'color',cyan)
legend('Ref.','Est.','location','northwest')
set(gca,'FontSize',15)
xlabel('Gait Cycle (%)')
ylabel('Knee Angle (^o)')
grid on

