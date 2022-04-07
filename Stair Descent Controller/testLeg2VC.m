clearvars -except Streaming Normalized stairAscentTrialData R01 rawR01 estimatedAngles;

close all
addpath("Utility Functions")
addpath("../Data")
addpath("Virtual Constraints")
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


X = controllerConstants_StairAscent();
knownInclines = X.inclines;
knee = X.knee;
ankle = X.ankle;

ascend = 0;

rmIndex = 8;

predeterminedIncline = 1;

inclineAngle = -30;

q_h_max = X.thigh.maxima(rmIndex);
q_h_min = X.thigh.minima(rmIndex);

qh_max = max(thigh_pos_d30_shift);
qh_min = min(thigh_pos_d30_shift);
% b = generateWeightedMatrix_leftOut(q_h_max, q_h_min, X, 1, ascend, -30,8);


phase = pv;
HS = 1;
for i = 1:length(phase)
    [knee_des, ankle_des, knee_vel_d, ankle_vel_d, ...
        estimatedIncline] = calcStairVC(phase(i), HS, q_h_max, q_h_min, predeterminedIncline, ascend, inclineAngle);
    knee_est(i) = knee_des;
    ankle_est(i) = ankle_des;
end



% knee_pos_d30_reparam = interp1(pv,knee_pos_d30_shift,t,'linear','extrap');
% 
% Y_k = fft(knee_pos_d30_reparam);
% L = length(Y_k);
% pk_real = real(Y_k/(L/2));
% pk_im = imag(Y_k/(L/2));
% 
% N_k = 14;
% 
% syms s
% hk = .5*pk_real(1) + .5*pk_real(N_k/2+1)*cos(pi*N_k*s);
% 
% for k = 1:N_k/2-1
% 
%     hk = hk + (pk_real(k+1)*cos(2*pi*k*s) - pk_im(k+1)*sin(2*pi*k*s));
%     
% end




figure
plot(gc,knee_pos_d30_shift)
hold on
plot(gc,knee_est)
title("Knee VC")
ylabel("Knee Angle (^o)")
xlabel("Gait Cycle (%)")
legend('AB','Est')

% figure
% ax(1) = subplot(131);
% plot(gc,pv);
% ylabel("Phase")
% xlabel("Gait Cycle (%)")
% ax(2) = subplot(132);
% plot(gc,ddt(deg2rad(knee_pos_d30_est),dt_normalized));
% ylabel("Est Knee Vel")
% xlabel("Gait Cycle (%)")
% ax(3) = subplot(133);
% plot(gc,ddt(deg2rad(thigh_pos_d30_shift),dt_normalized));
% ylabel("Thigh Vel (rad/s)")
% xlabel("Gait Cycle (%)")
% linkaxes(ax,'x')
% % plot(t,knee_pos_d30_reparam)



function [b] = generateWeightedMatrix_leftOut(q_h_max, q_h_min, X, predeterminedIncline, ascend, inclineAngle,rmIndex)
%CALCSTAIRVC Summary of this function goes here
%   Detailed explanation goes here
% uses pre-defined angle




if predeterminedIncline
    b = zeros(9,1);
    closest_max = zeros(1,2);
    closest_min = zeros(1,2);
    closest_incline = zeros(1,2);
    
    knownMaxima = X.thigh.maxima;
    knownMinima = X.thigh.minima;
    knownInclines = X.inclines;

    switch inclineAngle
        case 35
            b(1) = 1;
        case 30
            b(2) = 1;
        case 25
            b(3) = 1;
        case 20
            b(4) = 1;
        case 0
            b(5) = 1;
        case -20
            b(6) = 1;
        case -25
            b(7) = 1;
        case -30
            b(8) = 1;
        case -35
            b(9) = 1;
        otherwise
            [~,indexOfMin] = min(abs(X.inclines-inclineAngle));
            b(indexOfMin) = 1;
    end
else
    
    b = zeros(9,1);
    closest_max = zeros(1,2);
    closest_min = zeros(1,2);
    closest_incline = zeros(1,2);
    
    knownMaxima = X.thigh.maxima;
    knownMinima = X.thigh.minima;
    knownInclines = X.inclines; 
    
    if ascend == 1
        
       
        
        x = q_h_max;
        %bound 1
        usefulMaxima = knownMaxima(1:5);
        usefulMaxima(rmIndex) = [];
        
        usefulInclines = knownInclines(1:5);
        usefulInclines(rmIndex) = [];
        
        [~,min_Index] = min(abs(usefulMaxima-x));
        bounds = min_Index;
        
        if (usefulMaxima(min_Index) <= x || usefulMaxima(min_Index) == usefulMaxima(end)) && usefulMaxima(min_Index) ~= usefulMaxima(1)
            min_Index2 = 1;
        else
            min_Index2 = 2;
        end
        
        if min_Index2 == 1
            %Closest  % 2ndClosest
            bounds = [min_Index-1 min_Index];
            closest_incline =  usefulInclines(bounds);
            closest_max = usefulMaxima(bounds);
            
            x0 = closest_max(2);
            y0 = closest_incline(2);
            
            x1 = closest_max(1);
            y1 = closest_incline(1);
            
            
            
            b1 = (x-x0)/(x1-x0);
            b0 = 1-b1;
            b;
            b(bounds)= [b1 b0];
            
        else
            bounds = [min_Index min_Index+1];
            
            closest_incline =  usefulInclines(bounds);
            closest_max = usefulMaxima(bounds);
            
            x0 = closest_max(1);
            y0 = closest_incline(1);
            
            x1 = closest_max(2);
            y1 = closest_incline(2);
            
            b1 = (x-x0)/(x1-x0);
            b0 = 1-b1;
            b;
            b(bounds)= [b0 b1];
        end
        %         knownInclines*b
        %
        %
        %         knownMaxima*b
        
    else
        x = q_h_min;
        %bound 1
        usefulMinima = knownMinima(6:end);
        [~,min_Index] = min(abs(usefulMinima-x));
        % closest_max(1) = knownMaxima(min_Index);
        % closest_incline(1) = knownInclines(min_Index);
        bounds = min_Index;
        
        if (usefulMinima(min_Index) >= x || usefulMinima(min_Index) == usefulMinima(end)) && usefulMinima(min_Index) ~= usefulMinima(1)
            min_Index2 = 1;
        else
            min_Index2 = 2;
        end
        
        %         boundMatrix = [abs(usefulMinima(min_Index-1)-q_h_min), abs(usefulMinima(min_Index+1)-q_h_min)];
        %
        %         [~,min_Index2] = min(boundMatrix);
        if min_Index2 == 1
            %Closest  % 2ndClosest
            bounds = [min_Index-1 min_Index];
            closest_incline =  knownInclines(bounds);
            closest_min = usefulMinima(bounds);
            
            x0 = closest_min(2);
            y0 = closest_incline(2);
            
            x1 = closest_min(1);
            y1 = closest_incline(1);
            
            
            
            b1 = (x-x0)/(x1-x0);
            b0 = 1-b1;
            %             b
            b(bounds+(length(knownMinima)-length(usefulMinima)))= [b1 b0];
            
        else
            bounds = [min_Index min_Index+1];
            
            closest_incline =  knownInclines(bounds);
            closest_min = usefulMinima(bounds);
            
            x0 = closest_min(1);
            y0 = closest_incline(1);
            
            x1 = closest_min(2);
            y1 = closest_incline(2);
            
            b1 = (x-x0)/(x1-x0);
            b0 = 1-b1;
            %             b
            b(bounds+(length(knownMinima)-length(usefulMinima)))= [b0 b1];
        end
        %         knownInclines*b
        %
        %
        %         knownMinima*b
        
        
    end
end

% estimatedIncline = knownInclines*b;

end

function ha = ankle_VC_func(phase)
%ANKLE_VC_FUNC
%    HA = ANKLE_VC_FUNC(PHASE)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    06-Aug-2021 09:50:01

t2 = phase.*pi.*2.0;
t3 = phase.*pi.*4.0;
t4 = phase.*pi.*6.0;
t5 = phase.*pi.*8.0;
t6 = phase.*pi.*1.0e+1;
t7 = phase.*pi.*1.2e+1;
ha = [1.0./2.0,cos(t2),-sin(t2),cos(t3),-sin(t3),cos(t4),-sin(t4),cos(t5),-sin(t5),cos(t6),-sin(t6),cos(t7),-sin(t7),cos(phase.*pi.*1.4e+1)./2.0];

end
function hk = knee_VC_func(phase)
%KNEE_VC_FUNC
%    HK = KNEE_VC_FUNC(PHASE)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    06-Aug-2021 09:50:01

t2 = phase.*pi.*2.0;
t3 = phase.*pi.*4.0;
t4 = phase.*pi.*6.0;
t5 = phase.*pi.*8.0;
t6 = phase.*pi.*1.0e+1;
t7 = phase.*pi.*1.2e+1;
hk = [1.0./2.0,cos(t2),-sin(t2),cos(t3),-sin(t3),cos(t4),-sin(t4),cos(t5),-sin(t5),cos(t6),-sin(t6),cos(t7),-sin(t7),cos(phase.*pi.*1.4e+1)./2.0];
end
