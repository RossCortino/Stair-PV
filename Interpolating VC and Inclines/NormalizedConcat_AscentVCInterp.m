
%% Load Data 
clearvars -except Streaming Normalized R01 rawR01;
close all
clc;

load 'Data/filtStairTraj_i20.mat'

thigh_20 = thigh_mean;
knee_20 = knee_mean;
ankle_20 = ankle_mean;

load 'Data/filtStairTraj_i25.mat'

thigh_25 = thigh_mean;
knee_25 = knee_mean;
ankle_25 = ankle_mean;

load 'Data/filtStairTraj_i30.mat'

thigh_30 = thigh_mean;
knee_30 = knee_mean;
ankle_30 = ankle_mean;

load 'Data/filtStairTraj_i35.mat'

thigh_35 = thigh_mean;
knee_35 = knee_mean;
ankle_35 = ankle_mean;


L = length(thigh_20);

%% Calculate PV

current_incline = '20^o';

load 'Data/filtStairTraj_i20.mat'
knee_mean = knee_20';
thigh_mean = thigh_20';

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




pv = unique(pv,'stable');
pv = smooth(interp1(1:length(pv), pv, 1:length(pv)/151:length(pv)))';
T = griddedInterpolant(pv,t/100,'linear','linear');
figure
plot(t,pv)

knee_interp = interp1(pv,knee_mean,T(pv),'linear','extrap');
ankle_interp = interp1(pv, ankle_mean, T(pv),'linear','extrap');


L = length(knee_interp);
fs = 150;
%     f = fs*(0:(L/2))/L;
fc = 6;

[b,a] = butter(2,fc/(fs/2),'low');

knee_interp = filtfilt(b,a,knee_interp);

fc = 6;

[b,a] = butter(2,fc/(fs/2),'low');

ankle_interp = filtfilt(b,a,ankle_interp);


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


%% Subject Streaming Data Test

sub={'AB01','AB02','AB03', 'AB04', 'AB05', 'AB06', 'AB07', 'AB08', 'AB09', 'AB10'};

% percentGait=linspace(0,1,150);
incline={'i20'};

trial={'s1'};
[thigh_s1,knee_s1,ankle_s1] = averageJointKinematics(Normalized,sub,trial,incline);

trial={'s3'};
[thigh_s3,knee_s3,ankle_s3] = averageJointKinematics(Normalized,sub,trial,incline);

thigh_stream = thigh_s3%smooth([thigh_s1 thigh_s3],10)';
knee_stream = knee_s3 %smooth([knee_s1 knee_s3],10)';
ankle_stream = ankle_s3%smooth([ankle_s1 ankle_s3],10)';

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
c = mean([c_20 c_25 c_30 c_35])


% figure
% plot(t,thigh_20)
% hold on
% plot(t,thigh_25)
% plot(t,thigh_30)
% plot(t,thigh_35)

%peak detection parameters
%threshold and min distance
thresh = .001;
minpeakh = 20;
minpeakd = 50;

xpeak = [];
ypeak = [];
temp_traj = [];
p = 1;
mhf = 0;
% Add minimum hip flexion detection as well
qh_min = min(thigh_20);
qh_max = max(thigh_20);
temp_max = 0;
N_k = 18;
N_a = 18;
sh = sym('sh');
knee_est = [];
%Calculate Phase Variable
for i = 1:length(thigh_stream)
    thigh = thigh_stream(i);
    thighd = thighd_stream(i);
    
    
   
    if thigh > 20 && i-p > minpeakd
        temp_traj = [temp_traj thigh];
        if length(temp_traj) > 3
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
    
    [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max, c, prevState,prevPV, sm, qhm,mhf);
    s(i) = currPV;
    prevState = currState;
    state(i) = currState;
    prevPV = currPV;
    
    
    
    if qh_max ~= temp_max
        xq = qh_max;
%       hk = .5*interp1(x,vkreal(1,:),xq,'linear','extrap')+.5*interp1(x,vkim(N/2,:),xq,'linear','extrap')*cos(pi*N*sh);
        hk = .5*pk_real(1) + .5*pk_real(N_k/2)*cos(pi*N_k*sh);
        ha = .5*pa_real(1) + .5*pa_real(N_a/2)*cos(pi*N_a*sh);
        for k = 1:N_k/2-1
            hk = hk + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
            %hk = hk + interp1(x,vkreal(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
        end
        for a = 1:N_a/2-1
            ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
        end
        temp_max = qh_max;
    end
    knee_est(i) = double(subs(hk, sh, s(i)));
    ankle_est(i) = double(subs(ha, sh, s(i)));
   
end


samples = 1:length(thigh_stream);
figure
subplot(211)
plot(samples,thigh_stream)
hold on
plot(xpeak,ypeak,'*')
ylabel('Angle (^o)')
title('Thigh Angle')

subplot(212)
yyaxis left
plot(samples,s,'k')
ylabel('Phase Variable')
yyaxis right
plot(samples,state)
ylabel('State')
title('Phase Variable Real Time detection')

figure
yyaxis left
plot(samples,s,'k')
ylabel('Phase Variable')
yyaxis right
plot(samples,state)
ylabel('State')
title('Phase Variable Real Time detection')

figure 
plot(samples,thigh_stream, 'linewidth',2)
hold on
plot(xpeak,ypeak,'*')
ylabel('Angle (^o)')

figure
sgtitle("Incline VC Interp Knee Example")
subplot(311)
plot(samples,thigh_stream)
hold on
plot(xpeak,ypeak,'*')
ylabel('Angle (^o)')
title('Thigh Angle')

subplot(312)
plot(samples, knee_stream,'linewidth',2)
hold on
plot(samples, knee_est, '--','linewidth',2)
% xlabel('samples')
ylabel('Knee Angle (^o)')
title('Knee Angle')
legend('Raw','Est','Location','South')



subplot(313)
plot(samples,s,'k')
title("Phase Variable")
ylabel('Phase Variable')
xlabel('samples')

figure
sgtitle("Incline VC Interp Ankle Example")
subplot(311)
plot(samples,thigh_stream)
hold on
plot(xpeak,ypeak,'*')
ylabel('Angle (^o)')
title('Thigh Angle')
subplot(312)
plot(samples, ankle_stream,'linewidth',2)
hold on
plot(samples, ankle_est, '--','linewidth',2)
legend('Raw','Est','Location','South')
title('Ankle Angle')

ylabel('Ankle Angle (^o)')
subplot(313)
plot(samples,s,'k')
ylabel('Phase Variable')
title("Phase Variable")
xlabel('samples')












