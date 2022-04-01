clearvars Normalized 

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



thigh_30_shift = smooth(circshift(thigh_30,-(find(thigh_30 == max(thigh_30)))));
knee_30_shift = circshift(knee_30,-(find(thigh_30 == max(thigh_30))));
ankle_30_shift = circshift(ankle_30,-(find(thigh_30 == max(thigh_30))));
c_shift = t(find(thigh_30_shift == min(thigh_30_shift)));
% c = t(find(thigh_30== min(thigh_30)));
thigh_30d_shift = ddt(thigh_30_shift);

s_po = .55;
prevState = 1;
prevPV = 0;
pv_swing_thresh = .75;
qh_max = max(thigh_30);
qh_min = min(thigh_30);
pv = [];
sm = 0;
qhm = 0;
thresh = .00001;
minpeakh = 15;
minpeakd = 50;
mhf = 0;
temp_traj = [];
ypeak = [];
xpeak = [];

phaseRateScaler = .75;

gc = linspace(0,100,length(t));
state= zeros(1,length(gc));

for i = 1:length(t)
    state(i) = prevState;
    thigh = thigh_30_shift(i);
    
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair_FF_S2(thigh, qh_min, qh_max,s_po, c_shift, prevState,prevPV, sm, qhm, mhf, pv_swing_thresh, phaseRateScaler);
    
    pv(i) = currPV;
    
    prevState = currState;
    prevPV = currPV;
    
    % MHF Detection
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
    
end

knee_interp = interp1(pv,knee_30_shift,t,'linear','extrap');

figure

plot(gc, thigh_30_shift,'linewidth',2)
set(gca,'FontSize',15)
xlabel('Gait Cycle (%)')
ylabel('Thigh Angle (^o)')
yyaxis right
plot(gc,state)
ylabel('State')
grid on

figure
yyaxis left
plot(gc, pv,'linewidth',2)
hold on
set(gca,'FontSize',15)
xlabel('Gait Cycle (%)')
ylabel('Phase')
yyaxis right
plot(gc,state)
ylabel('State')
grid on

Y_k = fft(knee_interp);
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
plot(gc, knee_30_shift,'--','linewidth',2,'color',indigo)
hold on
plot(gc,subs(hk,s,pv),'linewidth',2,'color',cyan)
legend('Ref.','Est Orig.','location','northwest')
set(gca,'FontSize',15)
xlabel('Gait Cycle (%)')
ylabel('Knee Angle (^o)')
grid on


RMSE(double(subs(hk,s,pv)),knee_30_shift)
% RMSE(double(subs(hk_org,s,pv_org)),knee_30_shift)
