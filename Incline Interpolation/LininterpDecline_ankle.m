% Load Data
% load('Z:\Projects\ContinuouslyVarying\Matlab Processing - RN\R01.mat')
% This file takes some time to load, it is recommend to load it once and
% avoid removing it from memory by using "clearvars -except" when clearing
% the matlab workspace
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

pa_real20 = real(fft(ankle_20)/(L/2));
pa_real25 = real(fft(ankle_25)/(L/2));
pa_real30 = real(fft(ankle_30)/(L/2));
pa_real35 = real(fft(ankle_35)/(L/2));

pa_im20 = imag(fft(ankle_20)/(L/2));
pa_im25 = imag(fft(ankle_25)/(L/2));
pa_im30 = imag(fft(ankle_30)/(L/2));
pa_im35 = imag(fft(ankle_35)/(L/2));

N = 30;


vk_real = [pa_real20(1:N/2) pa_real30(1:N/2) pa_real35(1:N/2)]';
vk_im = [pa_im20(1:N/2) pa_im30(1:N/2) pa_im35(1:N/2)]';
x = [1:N/2 1:N/2 1:N/2]';
y = repelem([max(thigh_20); max(thigh_30); max(thigh_35)],N/2);

pa_real = scatteredInterpolant(x,y,vk_real,'linear','linear');
pa_im = scatteredInterpolant(x,y,vk_im, 'linear', 'linear');

sh = sym('sh');

theta = max(thigh_30);
% hk = .5*pk_real(1,theta) + .5*pk_real(N/2,theta)*cos(pi*N*sh);
ha = .5*pa_real(1,theta) + .5*pa_real(N/2,theta)*cos(pi*N*sh);


for a = 1:N/2-1
        ha = ha + pa_real(a+1,theta)*cos(2*pi*(a)*sh)-pa_im(a+1,theta)*sin(2*pi*(a)*sh);
end
s_h = 0:1/(L-1):1;

ankle_30_est = double(subs(ha, sh, s_h));

figure

plot(percentGait, ankle_30, percentGait, ankle_30_est)
title("Knee Angle at 25^{o} Decline")
xlabel("Gaitcycle(%)")
ylabel("Angle(^{o})")
legend('Ankle','Estimated Ankle (Interp Coefficients)')


i = 1:N/2;
L = 1:150;
figure
plot(i,pa_real20(i),i,pa_real25(i),i,pa_real30(i), i,pa_real35(i))
title("Real Coefficients")
legend('20','25','30','35')

figure
plot(i,pa_im20(i),i,pa_im25(i),i,pa_im30(i), i,pa_im35(i))
title("Imaginary Coefficients")
legend('20','25','30','35')