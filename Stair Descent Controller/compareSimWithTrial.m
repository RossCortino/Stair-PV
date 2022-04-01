clearvars -except Streaming Normalized stairAscentTrialData R01 rawR01 estimatedAngles;

close all
addpath("Utility Functions")
addpath("../Data")
if ~exist('Normalized')
    load '../Data/Normalized.mat'
end

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};

trial={'s3'};

incline={'in30'};
[thigh_pos_d30, knee_pos_d30, ankle_pos_d30, thigh_pos_d30sd, knee_pos_d30sd, ankle_pos_d30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline); 
[thigh_vel_d30, knee_vel_d30, ankle_vel_d30, thigh_vel_d30sd, knee_vel_d30sd, ankle_vel_d30sd] = averageJointVelocities_Stair(Normalized, sub, trial, incline);



[trial_file, path] = uigetfile();

load(fullfile(path, trial_file));

trial_sub = {'AB12'};
trial_trial = {'t7'};
trial_stride = {'s1'};
phase_trial = stairAscentTrialData.(trial_sub{1}).(trial_trial{1}).(trial_stride{1}).phaseEstimate';
knee_trial = stairAscentTrialData.(trial_sub{1}).(trial_trial{1}).(trial_stride{1}).knee_act';




