clearvars -except Streaming Normalized stairAscentTrialData R01 rawR01 estimatedAngles;


close all
addpath("Utility Functions")

if ~exist('Normalized')
    load '../Data/Normalized.mat'
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
incline={'in25'};
trial={'s3'};
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
[thigh_d30, knee_d30, ankle_d30, thigh_d30sd, knee_d30sd, ankle_d30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);


thigh_d30 = interp1(1:length(thigh_d30), thigh_d30, 1:length(thigh_d30)/(503):length(thigh_d30));
knee_d30 = interp1(1:length(knee_d30), knee_d30, 1:length(knee_d30)/(503):length(knee_d30));
thigh_d30_shift = circshift(thigh_d30,-find(thigh_d30 == max(thigh_d30)));

thigh_shift = smooth(thigh_d30_shift,85)'

knee_shift = smooth(circshift(knee_d30,-find(thigh_d30 == max(thigh_d30))))'

gc = linspace(0,100,length(thigh_d30));
t = linspace(0,1,length(gc));

c_d30 = t(find(thigh_d30_shift == min(thigh_d30_shift)));

thigh_mean = thigh_d30;
knee_mean = knee_d30;
ankle_mean = ankle_d30;
current_incline = '-30^o';
q_po = 7;
pv_swing_thresh = .85;
c= c_d30;

thighd_mean = ddt(thigh_shift);
prevPV = 0;
prevState = 1;
sm = 0;
qhm = 0;
qh_max = max(thigh_mean);
qh_min = min(thigh_mean);
%     c = t(find(thigh_mean== qh_min))


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
%     pv_swing_thresh = .96;
ytrough = [];
xtrough = [];


state =[]
for i = 1:length(t)
        
        thigh = thigh_shift(i);
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
%         if strcmp(current_incline, '0^o')
%             [currPV,currState,sm,qhm] = calculatePhaseVariable_Walk(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
%         else
%             [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
%         end
        
        [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
        %
        
        
        pv(i) = currPV;
        prevState = currState;
        prevPV = currPV;
        state(i) = currState;
end

knee_interp = interp1(pv,knee_shift,t,'linear','extrap');


Y_k = fft(knee_interp);
L = length(Y_k);
pk_real = real(Y_k/(L/2));
pk_im = imag(Y_k/(L/2));

syms s
N_k = 12

hk_shift = .5*pk_real(1) + .5*pk_real(N_k/2+1)*cos(pi*N_k*s);


for k = 1:N_k/2-1

    hk_shift = hk_shift + (pk_real(k+1)*cos(2*pi*k*s) - pk_im(k+1)*sin(2*pi*k*s));
    
end

figure
plot(gc,thigh_shift,"linewidth",2,'color',indigo)
xlabel("Gait Cycle (%)")
ylabel("Thigh Angle (^o)")
xline(gc(301+140),"k--","linewidth",2)
set(gca,'FontSize',15)
grid on


figure
plot(gc,knee_shift,'--',"linewidth",2,'color',indigo)
hold on
plot(gc,subs(hk_shift,s,pv),"linewidth",2,'color',teal)
xlabel("Gait Cycle (%)")
ylabel("Knee Angle (^o)")
xline(gc(301+140),"k--","linewidth",2)
set(gca,'FontSize',15)
legend("Ref.","Est.",'location','northwest')
grid on
knee_est = double(subs(hk_shift,s,pv));
rmse_knee = RMSE(knee_est,knee_shift)

figure
plot(gc,pv,"linewidth",2,'color',indigo)
xlabel("Gait Cycle (%)")
ylabel("Phase")
xline(gc(301+140),"k--","linewidth",2)
set(gca,'FontSize',15)
grid on
fs = 1/100;

figure
plot(gc,ddt(knee_shift,fs)./ddt(thigh_shift,fs),"linewidth",2,'color',indigo)
ylabel("Knee_{vel}/Thigh_{vel}")
xlabel("Gait Cycle (%)")
xline(gc(301+140),"k--","linewidth",2)
set(gca,'FontSize',15)
grid on

figure
subplot(511)
plot(gc,thigh_shift,"linewidth",2)
xlabel("Gait Cycle (%)")
ylabel("Thigh Angle (^o)")
xline(gc(90+42),"r--","linewidth",2)
grid on
subplot(512)
plot(gc,knee_shift,"linewidth",2)
xlabel("Gait Cycle (%)")
ylabel("Knee Angle (^o)")
xline(gc(90+42),"r--","linewidth",2)
grid on
subplot(513)
plot(gc,pv,"linewidth",2)
xlabel("Gait Cycle (%)")
ylabel("Phase Variable")
xline(gc(90+42),"r--","linewidth",2)
grid on
subplot(514)
plot(gc,state,"linewidth",2)
xlabel("Gait Cycle (%)")
ylabel("State")
xline(gc(90+42),"r--","linewidth",2)
grid on
subplot(515)
plot(gc,ddt(knee_shift,fs)./ddt(thigh_shift,fs),"linewidth",2)
ylabel("Knee_{vel}/Thigh_{vel}")
xlabel("Gait Cycle (%)")
xline(gc(90+42),"r--","linewidth",2)
grid on

