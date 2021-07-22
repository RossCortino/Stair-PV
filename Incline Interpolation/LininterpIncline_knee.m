% Load Data
% load('Z:\Projects\ContinuouslyVarying\Matlab Processing - RN\R01.mat')
% This file takes some time to load, it is recommend to load it once and
% avoid removing it from memory by using "clearvars -except" when clearing
% the matlab workspace
clearvars -except Streaming Normalized R01;
close all
clc;

R01 = Normalized;
sub={'AB01','AB02','AB03', 'AB04', 'AB05', 'AB06', 'AB07', 'AB08', 'AB09', 'AB10'};

percentGait=linspace(0,1,150);
trial={'s3'};

incline={'i20'};
[thigh_20,knee_20,ankle_20] = averageJointKinematics(R01,sub,trial,incline);

incline={'i25'};
[thigh_25,knee_25,ankle_25] = averageJointKinematics(R01,sub,trial,incline);

incline={'i30'};
[thigh_30,knee_30,ankle_30] = averageJointKinematics(R01,sub,trial,incline);

incline={'i35'};
[thigh_35,knee_35,ankle_35] = averageJointKinematics(R01,sub,trial,incline);

L = 150;

pk_real20 = real(fft(knee_20)/(L/2));
pk_real25 = real(fft(knee_25)/(L/2));
pk_real30 = real(fft(knee_30)/(L/2));
pk_real35 = real(fft(knee_35)/(L/2));

pk_im20 = imag(fft(knee_20)/(L/2));
pk_im25 = imag(fft(knee_25)/(L/2));
pk_im30 = imag(fft(knee_30)/(L/2));
pk_im35 = imag(fft(knee_35)/(L/2));

N = 10;


vk_real = [pk_real20(1:N/2) pk_real30(1:N/2) pk_real35(1:N/2)]';
vk_im = [pk_im20(1:N/2) pk_im30(1:N/2) pk_im35(1:N/2)]';
x = [1:N/2 1:N/2 1:N/2]';
y = repelem([max(thigh_20); max(thigh_30); max(thigh_35)],N/2);

pk_real = scatteredInterpolant(x,y,vk_real,'linear','linear');
pk_im = scatteredInterpolant(x,y,vk_im, 'linear', 'linear');

sh = sym('sh');

theta = max(thigh_25);
hk = .5*pk_real(1,theta) + .5*pk_real(N/2,theta)*cos(pi*N*sh);
% ha = .5*pa_real(1,theta) + .5*pa_real(N/2,theta)*cos(pi*N*sh);

for k = 1:N/2-1
    hk = hk + pk_real(k+1,theta)*cos(2*pi*(k)*sh)-pk_im(k+1,theta)*sin(2*pi*(k)*sh);
end

% for a = 1:N/2-1
%         ha = ha + pa_real(a+1,theta)*cos(2*pi*(a)*sh)-pa_im(a+1,theta)*sin(2*pi*(a)*sh);
% end
s_h = 0:1/(L-1):1;

knee_25_est = double(subs(hk, sh, s_h));

figure

plot(percentGait, knee_25, percentGait, knee_25_est)
title("Knee Angle at 25^{o} Incline")
xlabel("Gaitcycle(%)")
ylabel("Angle(^{o})")
legend('Knee','Estimated Knee (Interp Coefficients)')


i = 1:N/2;
L = 1:150;
figure
plot(i,pk_real20(i),i,pk_real25(i),i,pk_real30(i), i,pk_real35(i))
title("Real Coefficients")
legend('20','25','30','35')

figure
plot(i,pk_im20(i),i,pk_im25(i),i,pk_im30(i), i,pk_im35(i))
title("Imaginary Coefficients")
legend('20','25','30','35')




