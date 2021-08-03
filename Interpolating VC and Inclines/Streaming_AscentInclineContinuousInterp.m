
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

%% Knee Coefficients
pk_real20 = real(fft(knee_20)/(L/2));
pk_real25 = real(fft(knee_25)/(L/2));
pk_real30 = real(fft(knee_30)/(L/2));
pk_real35 = real(fft(knee_35)/(L/2));

pk_im20 = imag(fft(knee_20)/(L/2));
pk_im25 = imag(fft(knee_25)/(L/2));
pk_im30 = imag(fft(knee_30)/(L/2));
pk_im35 = imag(fft(knee_35)/(L/2));

N = 50;
x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vkreal(i,:) = [pk_real20(i) pk_real25(i) pk_real30(i) pk_real35(i)];
    vkim(i,:) = [pk_im20(i) pk_im25(i) pk_im30(i) pk_im35(i)];
end
%% Ankle Coefficients
pa_real20 = real(fft(ankle_20)/(L/2));
pa_real25 = real(fft(ankle_25)/(L/2));
pa_real30 = real(fft(ankle_30)/(L/2));
pa_real35 = real(fft(ankle_35)/(L/2));

pa_im20 = imag(fft(ankle_20)/(L/2));
pa_im25 = imag(fft(ankle_25)/(L/2));
pa_im30 = imag(fft(ankle_30)/(L/2));
pa_im35 = imag(fft(ankle_35)/(L/2));

N = 50;
x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vareal(i,:) = [pa_real20(i) pa_real25(i) pa_real30(i) pa_real35(i)];
    vaim(i,:) = [pa_im20(i) pa_im25(i) pa_im30(i) pa_im35(i)];
end

%% Subject Streaming Data Test

sub={'AB01'};
trial={'s20dg_01'};
data = rawR01.(sub{1}).Stair.(trial{1});

knee_stream = data.jointAngles.RKneeAngles(:,1);
ankle_stream = data.jointAngles.RAnkleAngles(:,1);
hip_stream = data.jointAngles.RHipAngles(:,1);


thigh_stream = hip_stream';
knee_stream = knee_stream';
ankle_stream= ankle_stream';


figure
plot(thigh_stream)



prevPV = 0;
prevState = 1;
sm = 0;
qhm = 0;
thighd_stream = ddt(thigh_stream);
t = linspace(0,1,length(thigh_mean));
t_20 = t(find(thigh_20 == min(thigh_20)));
t_25 = t(find(thigh_25 == min(thigh_25)));
t_30 = t(find(thigh_30 == min(thigh_30)));
t_35 = t(find(thigh_35 == min(thigh_35)));
c = mean([t_20 t_25 t_30 t_35])


figure
plot(t,thigh_20)
hold on
plot(t,thigh_25)
plot(t,thigh_30)
plot(t,thigh_35)
%peak detection parameters
%threshold and min distance
thresh = .001;
minpeakh = 30;
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
N = 20;
sh = sym('sh');
knee_est = [];
%Calculate Phase Variable
for i = 1:length(thigh_stream)
    thigh = thigh_stream(i);
    thighd = thighd_stream(i);
    
    
    
    if thigh > 25 && i-p > minpeakd
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
    prevPV = currPV;
    
    
    
    if qh_max ~= temp_max
        xq = qh_max;
        hk = .5*interp1(x,vkreal(1,:),xq,'linear','extrap')+.5*interp1(x,vkim(N/2,:),xq,'linear','extrap')*cos(pi*N*sh);
        for k = 1:N/2-1
        %     hk = hk + pk_real(k+1,theta)*cos(2*pi*(k)*sh)-pk_im(k+1,theta)*sin(2*pi*(k)*sh);
            hk = hk + interp1(x,vkreal(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
        end
        temp_max = qh_max;
    end
    
    knee_est(i) = double(subs(hk, sh, s(i)));
   
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
plot(samples,s,'k')
ylabel('Phase Variable')
title('Phase Variable Real Time detection')


figure 
plot(samples,thigh_stream, 'linewidth',2)
hold on
plot(xpeak,ypeak,'*')
ylabel('Angle (^o)')

figure
plot(samples, knee_stream,'linewidth',2)
hold on
plot(samples, knee_est, '--','linewidth',2)
xlabel('samples')
ylabel('Angle (^o)')












