clearvars -except Streaming Normalized R01 rawR01 X;
close all
clc;
addpath('../Utility Functions')
X = controllerConstants_StairAscent();
knownInclines = X.inclines;
knee = X.knee;
ankle = X.ankle;

if ~exist('Normalized')
    load '../Data/Normalized.mat'
end

load estimatedAngles.mat

load stairVCTrajectories.mat

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};

trial={'s3'};


knownIncline = -30;

incline={'i20'};

[thigh_20, knee_20, ankle_20, thigh_20sd, knee_20sd, ankle_20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i25'};

[thigh_25, knee_25, ankle_25, thigh_25sd, knee_25sd, ankle_25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i30'};

[thigh_30, knee_30, ankle_30, thigh_30sd, knee_30sd, ankle_30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i35'};

[thigh_35, knee_35, ankle_35, thigh_35sd, knee_35sd, ankle_35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);


incline={'in20'};

[thigh_d20, knee_d20, ankle_d20, thigh_d20sd, knee_d20sd, ankle_d20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in25'};

[thigh_d25, knee_d25, ankle_d25, thigh_d25sd, knee_d25sd, ankle_d25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in30'};

[thigh_d30, knee_d30, ankle_d30, thigh_d30sd, knee_d30sd, ankle_d30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in35'};

[thigh_d35, knee_d35, ankle_d35, thigh_d35sd, knee_d35sd, ankle_d35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);




sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s1'};
incline={'i0'};
[thigh_0, knee_0, ankle_0, thigh_0sd, knee_0sd, ankle_0sd] = averageJointKinematics_Walk(Normalized,sub,trial,incline);






switch knownIncline
    case 35
        incline={'i35'};
        thigh = thigh_35;
        knee = knee_35;
        ankle = ankle_35;
        ascend = 1;
        ind = 1;
    case 30
        incline={'i30'};
        thigh = thigh_30;
        knee = knee_30;
        ankle = ankle_30;
        ascend = 1;
        ind = 2;
    case 25
        incline={'i25'};
        thigh = thigh_25;
        knee = knee_25;
        ankle = ankle_25;
        ascend = 1;
        ind = 3;
    case 20
        incline={'i20'};
        thigh = thigh_20;
        knee = knee_20;
        ankle = ankle_20;
        ascend = 1;
        ind = 4;
    case -20
        incline={'in20'};
        thigh = thigh_d20;
        knee = knee_d20;
        ankle = ankle_d20;
        ascend = 0;
        ind = 6;
    case -25
        incline={'in25'};
        thigh = thigh_d25;
        ascend = 0;
        ind = 7;
    case -30
        incline={'in30'};
        thigh = thigh_d30;
        knee = knee_d30;
        ankle = ankle_d30;
        ascend = 0;
        ind = 8;
    case -35
        incline={'in35'};
        thigh = thigh_d35;
        knee = knee_d35;
        ankle = ankle_d35;
        ascend = 0;
        ind = 9;
    otherwise
        incline={'i35'};
        thigh = thigh_35;
        knee = knee_35;
        ankle = ankle_35;
        ascend = 0;
        ind = 1;
end


phase = estimatedAngles.phase.(incline{1});
knee_est = estimatedAngles.knee.(incline{1});
ankle_est = estimatedAngles.ankle.(incline{1});


t = linspace(0,100,length(thigh));
figure
subplot(221)
plot(t, knee,'Linewidth',2)
hold on
plot(t,knee_est,'--','Linewidth',2)
grid on
legend('Able Bod.','VC Est')
xlabel('Gait Cycle (%)')
ylabel('Knee Angle (^o)')
title(strcat("Knee VC Est: ",num2str(knownIncline),"^o"))
subplot(222)
plot(t, ankle,'Linewidth',2)
hold on
plot(t,ankle_est,'--','Linewidth',2)
grid on
title(strcat("Ankle VC Est: ",num2str(knownIncline),"^o"))
legend('Able Bod.','VC Est')
xlabel('Gait Cycle (%)')
ylabel('Ankle Angle (^o)')

subplot(223)
plot(thigh,knee_est)
title(strcat("Knee VC Est vs. Thigh: ",num2str(knownIncline),"^o"))
xlabel('Thigh Angle (^o)')
ylabel('Knee Angle (^o)')
grid on
subplot(224)
plot(thigh,ankle_est)
title(strcat("Ankle VC Est vs. Thigh: ",num2str(knownIncline),"^o"))
grid on

xlabel('Thigh Angle (^o)')
ylabel('Ankle Angle (^o)')



figure
subplot(311)
plot(t,thigh)
title('Thigh')
subplot(312)
plot(t,knee)
title('Knee')
subplot(313)
plot(t,phase)
title('Phase')
