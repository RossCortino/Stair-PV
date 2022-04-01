function [thigh_mean,knee_mean,ankle_mean,thigh_sd, knee_sd, ankle_sd, ...
    thigh_torque,knee_torque,ankle_torque,thigh_torque_sd, knee_torque_sd, ankle_torque_sd] = averageJointVelocities_Walk(Normalized, sub, trial, incline)

for s=1:numel(sub)
    
    % this line uses a dynamic field reference to access the data of each
    % subject in a for loop. Dynamic field references: https://blogs.mathworks.com/loren/2005/12/13/use-dynamic-field-references/
    temp_task = Normalized.(sub{s}).Walk.(trial{1}).(incline{1});
    
    % load pelvis and hip data, calculate thigh angle
    temp_knee = temp_task.jointAngleVels.KneeAngles;
    temp_ankle = temp_task.jointAngleVels.AnkleAngles;
    temp_pelvis = temp_task.jointAngleVels.PelvisAngles;
    temp_hip = temp_task.jointAngleVels.HipAngles;
    temp_thigh = temp_hip - temp_pelvis;
    
    temp_knee_torque = temp_task.jointMoments.KneeMoment;
    temp_ankle_torque = temp_task.jointMoments.AnkleMoment;
    %     temp_pelvis = temp_task.jointMoments.PelvisMoment;
    temp_hip_torque = temp_task.jointMoments.HipMoment;
    %     temp_thigh = temp_hip - temp_pelvis;
    
    % 1.the first dimension of data is 150 points, uniformly spread
    % throughout the gait cycl
    % 2. the second dimension indicates 3 axes of rotation
    % 3. the third dimension indicates repeated strides
    
    %this command cuts the data down to only the saggital plane of every
    %stride
    temp_saggital_thigh = squeeze( temp_thigh(:,1,:));
    temp_saggital_knee = squeeze( temp_knee(:,1,:));
    temp_saggital_ankle = squeeze( temp_ankle(:,1,:));
    
    temp_saggital_hip_torque = squeeze( temp_hip_torque(:,1,:));
    temp_saggital_knee_torque = squeeze( temp_knee_torque(:,1,:));
    temp_saggital_ankle_torque = squeeze( temp_ankle_torque(:,1,:));
    
    
    
    thigh_mean(s,:) = mean(temp_saggital_thigh');
    knee_mean(s,:) = mean(temp_saggital_knee');
    ankle_mean(s,:) = mean(temp_saggital_ankle');
    
    thigh_torque(s,:) = mean(temp_saggital_hip_torque');
    knee_torque(s,:) = mean(temp_saggital_knee_torque');
    ankle_torque(s,:) = mean(temp_saggital_ankle_torque');
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

thigh_torque = mean(thigh_torque);
knee_torque = mean(knee_torque);
ankle_torque = mean(ankle_torque);
thigh_torque_sd = std(thigh_torque);
knee_torque_sd = std(knee_torque);
ankle_torque_sd = std(ankle_torque);

end