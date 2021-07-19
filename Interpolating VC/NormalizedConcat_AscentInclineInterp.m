
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


%% Knee Coefficients
pk_real20 = real(fft(knee_20)/(L/2));
pk_real25 = real(fft(knee_25)/(L/2));
pk_real30 = real(fft(knee_30)/(L/2));
pk_real35 = real(fft(knee_35)/(L/2));

pk_im20 = imag(fft(knee_20)/(L/2));
pk_im25 = imag(fft(knee_25)/(L/2));
pk_im30 = imag(fft(knee_30)/(L/2));
pk_im35 = imag(fft(knee_35)/(L/2));

N = 100;
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

N = 100;
x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vareal(i,:) = [pa_real20(i) pa_real25(i) pa_real30(i) pa_real35(i)];
    vaim(i,:) = [pa_im20(i) pa_im25(i) pa_im30(i) pa_im35(i)];
end



%% Subject Streaming Data Test

sub={'AB01','AB02','AB03', 'AB04', 'AB05', 'AB06', 'AB07', 'AB08', 'AB09', 'AB10'};

% percentGait=linspace(0,1,150);
incline={'i30'};

thigh_ref = thigh_30;

trial={'s1'};
[thigh_s1,knee_s1,ankle_s1] = averageJointKinematics(Normalized,sub,trial,incline);

trial={'s3'};
[thigh_s3,knee_s3,ankle_s3] = averageJointKinematics(Normalized,sub,trial,incline);

thigh_stream = smooth([thigh_s1 thigh_s3],10)';
knee_stream = smooth([knee_s1 knee_s3],10)';
ankle_stream = smooth([ankle_s1 ankle_s3],10)';


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


% figure
% plot(t,thigh_20)
% hold on
% plot(t,thigh_25)
% plot(t,thigh_30)
% plot(t,thigh_35)

%peak detection parameters
%threshold and min distance
thresh = .00001;
minpeakh = 20;
minpeakd = 50;

xpeak = [];
ypeak = [];
temp_traj = [];
p = 1;
mhf = 0;
% Add minimum hip flexion detection as well
qh_min = min(thigh_ref);
qh_max = max(thigh_ref);
temp_max = 0;
N_k = 18;
N_a = 18;
sh = sym('sh');
knee_est = [];
%Calculate Phase Variable
for i = 1:length(thigh_stream)
    thigh = thigh_stream(i);
    thighd = thighd_stream(i);
    
    if prevState == 3 || prevState == 4
        if thigh > 20 %&& i-p > minpeakd
            temp_traj = [temp_traj thigh];
            if length(temp_traj) > 3
                temp_traj
                [pks,locs] = findpeaks(temp_traj,'threshold', thresh,'MinPeakHeight',minpeakh);
                if ~isempty(pks)
                    p = i-1;
                    temp_traj = [];
                    ypeak = [ypeak pks];
                    xpeak = [xpeak p];
                    temp_max = qh_max;
                    qh_max = max(thigh_ref) %max([ypeak max(thigh_ref)]);%mean([ypeak max(thigh_mean)]);
                    mhf = 1;

    %                 plot(1:i-1,pv)
    %                 ylabel('phasevariable')
    %                 title('PV Streaming')
    %                 hold on
                end
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
        hk = .5*interp1(x,vkreal(1,:),xq,'linear','extrap')+.5*interp1(x,vkim(N/2,:),xq,'linear','extrap')*cos(pi*N_k*sh);
%         hk = .5*pk_real(1) + .5*pk_real(N_k/2)*cos(pi*N_k*sh);
%         ha = .5*pa_real(1) + .5*pa_real(N_a/2)*cos(pi*N_a*sh);
        for k = 1:N_k/2-1
            %hk = hk + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
            hk = hk + interp1(x,vkreal(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
        end
        ha = .5*interp1(x,vareal(1,:),xq,'linear','extrap')+.5*interp1(x,vaim(N/2,:),xq,'linear','extrap')*cos(pi*N_a*sh);
        for a = 1:N_a/2-1
%             ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
            ha = ha + interp1(x,vareal(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
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












