clearvars -except Streaming Normalized R01;
close all
clc;


sub={'AB01','AB02','AB03', 'AB04', 'AB05', 'AB06', 'AB07', 'AB08', 'AB09', 'AB10'};

percentGait=linspace(0,1,150);
trial={'s3'};

incline={'in20'};
[thigh_20,knee_20,ankle_20] = averageJointKinematics(R01,sub,trial,incline);

incline={'in25'};
[thigh_25,knee_25,ankle_25] = averageJointKinematics(R01,sub,trial,incline);

incline={'in30'};
[thigh_30,knee_30,ankle_30] = averageJointKinematics(R01,sub,trial,incline);

incline={'in35'};
[thigh_35,knee_35,ankle_35] = averageJointKinematics(R01,sub,trial,incline);

L = 150;
%% Knee Coefficients
pk_real20 = real(fft(knee_20)/(L/2));
% pk_real25 = real(fft(knee_25)/(L/2));
pk_real30 = real(fft(knee_30)/(L/2));
pk_real35 = real(fft(knee_35)/(L/2));

pk_im20 = imag(fft(knee_20)/(L/2));
% pk_im25 = imag(fft(knee_25)/(L/2));
pk_im30 = imag(fft(knee_30)/(L/2));
pk_im35 = imag(fft(knee_35)/(L/2));

N = 20;
x = [min(thigh_20) min(thigh_30) min(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vkreal(i,:) = [pk_real20(i) pk_real30(i) pk_real35(i)];
    vkim(i,:) = [pk_im20(i) pk_im30(i) pk_im35(i)];
end

%% Ankle Coefficients
pa_real20 = real(fft(ankle_20)/(L/2));
% pa_real25 = real(fft(ankle_25)/(L/2));
pa_real30 = real(fft(ankle_30)/(L/2));
pa_real35 = real(fft(ankle_35)/(L/2));

pa_im20 = imag(fft(ankle_20)/(L/2));
% pa_im25 = imag(fft(ankle_25)/(L/2));
pa_im30 = imag(fft(ankle_30)/(L/2));
pa_im35 = imag(fft(ankle_35)/(L/2));

N = 20;
x = [min(thigh_20) min(thigh_30) min(thigh_35)];
% x = [max(thigh_20) max(thigh_25) max(thigh_30) max(thigh_35)];
for i = 1:N/2
    vareal(i,:) = [pa_real20(i) pa_real25(i) pa_real30(i) pa_real35(i)];
    vaim(i,:) = [pa_im20(i) pa_im25(i) pa_im30(i) pa_im35(i)];
end

%% Random Test
% Test With Random Subject

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s3'};
incline={'in25'};

[thigh_25, knee_25, ankle_25, thigh_25sd, knee_25sd, ankle_25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

thigh_mean = thigh_25;
knee_mean = knee_25;
ankle_mean = knee_25;
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
xq = min(thigh_mean);



hk = .5*interp1(x,vkreal(1,:),xq,'linear','extrap')+.5*interp1(x,vkim(N/2,:),xq,'linear','extrap')*cos(pi*N*sh);
for k = 1:N/2-1
%     hk = hk + pk_real(k+1,theta)*cos(2*pi*(k)*sh)-pk_im(k+1,theta)*sin(2*pi*(k)*sh);
    hk = hk + interp1(x,vkreal(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
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


figure
subplot(311)
plot(percentGait,thigh_20,percentGait,thigh_25,percentGait,thigh_30,percentGait,thigh_35,percentGait,thigh_mean,'--')
legend('20^{o}','25^{o}','30^{o}','35^{o}','Unknown')
title('Thigh Traj')
subplot(312)
plot(percentGait, knee_mean, percentGait, knee_est,'--')
title("Knee Angle Interpolation Estimate")
xlabel("Gaitcycle(%)")
ylabel("Angle(^{o})")
legend('Ref','Estimated (Interp Coefficients)')
grid on
subplot(313)
plot(percentGait, ankle_mean, percentGait, ankle_est,'--')
title("Ankle Angle Interpolation Estimate")
xlabel("Gaitcycle(%)")
ylabel("Angle(^{o})")

grid on



