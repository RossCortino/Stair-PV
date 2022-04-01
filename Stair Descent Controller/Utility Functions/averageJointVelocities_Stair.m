function [thigh_mean,knee_mean,ankle_mean,thigh_sd, knee_sd, ankle_sd] = averageJointVelocities_Stair(Normalized, sub, trial, incline)

for s=1:numel(sub)
    
    % this line uses a dynamic field reference to access the data of each
    % subject in a for loop. Dynamic field references: https://blogs.mathworks.com/loren/2005/12/13/use-dynamic-field-references/
    temp_task = Normalized.(sub{s}).Stair.(trial{1}).(incline{1}); 
    
   % load pelvis and hip data, calculate thigh angle 
    temp_knee = temp_task.jointAngleVels.KneeAngleVels;
    temp_ankle = temp_task.jointAngleVels.AnkleAngleVels;
    temp_pelvis = temp_task.jointAngleVels.PelvisAngleVels;
    temp_hip = temp_task.jointAngleVels.HipAngleVels;
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
    thigh_sd = std(thigh_mean);
    knee_sd = std(knee_mean);
    ankle_sd = std(ankle_mean);
    thigh_mean = mean(thigh_mean);
    knee_mean = mean(knee_mean);
    ankle_mean = mean(ankle_mean);
    

end