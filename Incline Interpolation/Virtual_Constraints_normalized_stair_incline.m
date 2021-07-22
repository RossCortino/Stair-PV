%% Load Data
% load('Z:\Projects\ContinuouslyVarying\Matlab Processing - RN\R01.mat')
% This file takes some time to load, it is recommend to load it once and
% avoid removing it from memory by using "clearvars -except" when clearing
% the matlab workspace
clearvars -except Streaming Normalized R01;
close all
clc;


sub={'AB01','AB02','AB03', 'AB04', 'AB05', 'AB06', 'AB07', 'AB08', 'AB09', 'AB10'};
incline={'i20'};
% incline={'i25'};
% incline={'i30'};
% incline={'i35'};
percentGait=linspace(0,1,150);
trial={'s3'};

for s=1:numel(sub)
    
    % this line uses a dynamic field reference to access the data of each
    % subject in a for loop. Dynamic field references: https://blogs.mathworks.com/loren/2005/12/13/use-dynamic-field-references/
    temp_task = Normalized.(sub{s}).Stair.(trial{1}).(incline{1}); 
    
   % load pelvis and hip data, calculate thigh angle 
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
    
 
    
    
    thigh_mean(s,:) = mean(temp_saggital_thigh');
    knee_mean(s,:) = mean(temp_saggital_knee');
    ankle_mean(s,:) = mean(temp_saggital_ankle');
%     plot(percentGait,thigh_mean, percentGait, knee_mean, percentGait, ankle_mean) % thigh angle vs. percent gait
%     title([sub{s},': Joint Kinematics'])
%     xlabel('Percent Gait')
%     ylabel('deg')

end
    thigh_mean = mean(thigh_mean);
    knee_mean = mean(knee_mean);
    ankle_mean = mean(ankle_mean);
    L = length(knee_mean);
    Y_k = fft(knee_mean);
    pk_real = real(Y_k/(L/2));
    pk_im = imag(Y_k/(L/2));


    %calculate ankle
    Y_a = fft(ankle_mean);
    f = 116*(0:(L/2))/L;
    pa_real = real(Y_a/(L/2));
    pa_im = imag(Y_a/(L/2));


    sh = sym('sh');
    
    s_h = 0:1/(L-1):1;
    norm_time = 0:1/(L-1):1;
    
    N_k = 20;
    N_a = 20;
    hk = .5*pk_real(1) + .5*pk_real(N_k/2)*cos(pi*N_k*sh);
    ha = .5*pa_real(1) + .5*pa_real(N_a/2)*cos(pi*N_a*sh);

    for k = 1:N_k/2-1
       hk = hk + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
    end

    for a = 1:N_a/2-1
        ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
    end

    
    knee_est = double(subs(hk, sh, s_h));
    ankle_est = double(subs(ha, sh, s_h));

    e_k = sqrt(sum((knee_mean - knee_est).^2)/L);
    e_a = sqrt(sum((ankle_mean - ankle_est).^2)/L);
    
    ek_text = strcat("RMSE = ",num2str(e_k));
    ea_text = strcat("RMSE = ",num2str(e_a));
 

    figure

    sgtitle(['Virtual Constraint Steady State Stair Ascent ', incline{1}] );
    subplot(2,2,1)
    plot(norm_time,thigh_mean,norm_time,knee_mean,norm_time,ankle_mean)
    legend('Thigh','Knee','Ankle');
    title(' Measured Joint Angles')
    ylabel('degrees'); xlabel('Normalized Time (t/T)')

    subplot(2,2,2)
    plot(norm_time,s_h)
    title('Phase Variable')
    ylabel('Phase Value'); xlabel('Normalized Time (t/T)')

    subplot(2,2,3)
    plot(norm_time,knee_mean,'b', norm_time, knee_est, '--')
    title('Knee Constraint Comparison')
    ylabel('degrees'); xlabel('Normalized Time (t/T)')
    legend('Measured', 'Estimated');
    text(.4,30,ek_text)
    
    
    subplot(2,2,4)
    plot(norm_time,ankle_mean,'b', norm_time, ankle_est, '--')
    title('Ankle Constraint Comparison')
    ylabel('degrees'); xlabel('Normalized Time (t/T)')
    legend('Measured', 'Estimated');
    text(.4,0,ea_text)
    
    print VC_I35_1 -dpng

