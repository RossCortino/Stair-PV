clearvars -except Streaming Normalized R01 rawR01;
close all
clc;

% load filtKnee20
% sub={'AB01','AB02','AB03', 'AB04', 'AB05', 'AB06', 'AB07', 'AB08', 'AB09', 'AB10'};


sub={'AB01','AB02','AB03', 'AB04', 'AB05', 'AB06', 'AB07', 'AB08', 'AB09', 'AB10'};

% percentGait=linspace(0,1,150);
incline={'i20'};

trial={'s1'};
[thigh_s1,knee_s1,ankle_s1] = averageJointKinematics(Normalized,sub,trial,incline);

trial={'s3'};
[thigh_s3,knee_s3,ankle_s3] = averageJointKinematics(Normalized,sub,trial,incline);

thigh_mean = smooth([thigh_s1 thigh_s3],10)';
knee_mean = smooth([knee_s1 knee_s3],10)';
ankle_mean = smooth([ankle_s1 ankle_s3],10)';
figure
plot(thigh_mean)

figure
plot(knee_mean)


figure
plot(ankle_mean)