clearvars -except Streaming Normalized R01;
close all
clc;


sub={'AB01','AB02','AB03', 'AB04', 'AB05', 'AB06', 'AB07', 'AB08', 'AB09', 'AB10'};

percentGait=linspace(0,1,150);
trial={'s3'};

incline={'i20'};
[thigh_20,knee_20,ankle_20] = averageJointKinematics(Normalized,sub,trial,incline);

incline={'i25'};
[thigh_25,knee_25,ankle_25] = averageJointKinematics(Normalized,sub,trial,incline);

incline={'i30'};
[thigh_30,knee_30,ankle_30] = averageJointKinematics(Normalized,sub,trial,incline);

incline={'i35'};
[thigh_35,knee_35,ankle_35] = averageJointKinematics(Normalized,sub,trial,incline);

L = 150;
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
x = [max(thigh_20) max(thigh_30) max(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vreal(i,:) = [pk_real20(i) pk_real30(i) pk_real35(i)];
    vim(i,:) = [pk_im20(i) pk_im30(i) pk_im35(i)];
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


x = [max(thigh_20) max(thigh_30) max(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vareal(i,:) = [pa_real20(i) pa_real30(i) pa_real35(i)];
    vaim(i,:) = [pa_im20(i) pa_im30(i) pa_im35(i)];
end

%% Random Test
% Test With Random Subject

sub={'AB01','AB02','AB03', 'AB04', 'AB05', 'AB06', 'AB07', 'AB08', 'AB09', 'AB10'};
trial={'s3'};
incline={'i25'};
temp_task = Normalized.(sub{1}).Stair.(trial{1}).(incline{1}); 

temp_knee = temp_task.jointAngles.KneeAngles;
temp_ankle = temp_task.jointAngles.AnkleAngles;
temp_pelvis = temp_task.jointAngles.PelvisAngles;
temp_hip = temp_task.jointAngles.HipAngles;
temp_thigh = temp_hip - temp_pelvis;

% 1.the first dimension of data is 150 points, uniformly spread
% throughout the gait cycl
% 2. the second dimension indicates 3 axes of rotation
% 3. the third dimension indicates repeated strides

%this command cuts the data down to only the saggital plane of every
%stride
temp_saggital_thigh = squeeze( temp_thigh(:,1,:));
temp_saggital_knee = squeeze( temp_knee(:,1,:));
temp_saggital_ankle = squeeze( temp_ankle(:,1,:));




thigh_mean = thigh_25;
knee_mean = knee_25;
ankle_mean = ankle_25;

figure
sgtitle(strcat("Knee Interpolation: ",sub{1},", ",incline{1}))
subplot(311)
plot(percentGait,thigh_20,percentGait,thigh_25,percentGait,thigh_30,percentGait,thigh_35,percentGait,thigh_mean,'--')
legend('20^{o}','25^{o}','30^{o}','35^{o}','Unknown')
title('Thigh Traj')
subplot(312)
plot(percentGait,knee_20,percentGait,knee_25,percentGait,knee_30,percentGait,knee_35,percentGait,knee_mean,'--')
legend('20^{o}','25^{o}','30^{o}','35^{o}','Unknown')
title('Knee Traj')


sh = sym('sh');
xq = max(thigh_mean);
hk = .5*interp1(x,vreal(1,:),xq,'linear','extrap')+.5*interp1(x,vim(N/2,:),xq,'linear','extrap')*cos(pi*N*sh);
for k = 1:N/2-1
%     hk = hk + pk_real(k+1,theta)*cos(2*pi*(k)*sh)-pk_im(k+1,theta)*sin(2*pi*(k)*sh);
    hk = hk + interp1(x,vreal(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vim(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
end

s_h = 0:1/(L-1):1;

knee_est = double(subs(hk, sh, s_h));


subplot(313)
plot(percentGait, knee_mean, percentGait, knee_est,'--')
title("Knee Angle Interpolation Estimate")
xlabel("Gaitcycle(%)")
ylabel("Angle(^{o})")
legend('Knee','Estimated Knee (Interp Coefficients)')
grid on



figure
sgtitle(strcat("Ankle Interpolation: ",sub{1},", ",incline{1}))
subplot(311)
plot(percentGait,thigh_20,percentGait,thigh_25,percentGait,thigh_30,percentGait,thigh_35,percentGait,thigh_mean,'--')
legend('20^{o}','25^{o}','30^{o}','35^{o}','Unknown')
title('Thigh Traj')
subplot(312)
plot(percentGait,ankle_20,percentGait,ankle_25,percentGait,ankle_30,percentGait,ankle_35,percentGait,ankle_mean,'--')
legend('20^{o}','25^{o}','30^{o}','35^{o}','Unknown')
title('Ankle Traj')




ha = .5*interp1(x,vareal(1,:),xq,'linear','extrap')+.5*interp1(x,vaim(N/2,:),xq,'linear','extrap')*cos(pi*N*sh);
for k = 1:N/2-1
%     hk = hk + pk_real(k+1,theta)*cos(2*pi*(k)*sh)-pk_im(k+1,theta)*sin(2*pi*(k)*sh);
    ha = ha + interp1(x,vareal(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vaim(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
end
ankle_est = double(subs(ha, sh, s_h));


subplot(313)
plot(percentGait, ankle_mean, percentGait, ankle_est,'--')
title("Ankle Angle Interpolation Estimate")
xlabel("Gaitcycle(%)")
ylabel("Angle(^{o})")
legend('Ankle','Estimated Ankle (Interp Coefficients)')
grid on


newcolors = [ 70 130 180
    30 144 255
    0 191 255
    135 206 235
    ]./255;
figure
colororder(newcolors)
plot(percentGait,thigh_20,'linewidth',2)
hold on
plot(percentGait,thigh_30,'linewidth',2)
plot(percentGait,thigh_35,'linewidth',2)
plot(percentGait,thigh_mean,'--','linewidth',2)
legend('20^{o}','30^{o}','35^{o}','Unknown(25^{o})','location','southeast')
grid on
title('Thigh Traj')

figure
colororder(newcolors)
plot(percentGait, knee_mean,'linewidth',2)
hold on
plot(percentGait, knee_est,'--','linewidth',2)
title("Knee Angle Interpolation Estimate")
xlabel("Gaitcycle(%)")
ylabel("Angle(^{o})")
legend('Ref','Estimated (Interp Coefficients)','location','northwest')
grid on

figure
colororder(newcolors)
plot(percentGait, ankle_mean,'linewidth',2)
hold on
plot(percentGait, ankle_est,'--','linewidth',2)
title("Ankle Angle Interpolation Estimate")
xlabel("Gaitcycle(%)")
ylabel("Angle(^{o})")

grid on








