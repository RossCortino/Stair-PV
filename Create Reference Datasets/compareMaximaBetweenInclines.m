clearvars -except Streaming Normalized R01 rawR01 X;
close all
clc;
addpath('../Utility Functions')
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s3'};


if ~exist('Normalized')
    load '../Data/Normalized.mat'
end


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

trial={'s1'};
incline={'i0'};
[thigh_0, knee_0, ankle_0, thigh_0sd, knee_0sd, ankle_0sd] = averageJointKinematics_Walk(Normalized,sub,trial,incline);

t = linspace(0,100,length(thigh_20));

figure
title('Range of Motion');
plot(35,max(thigh_35)-min(thigh_35),'*');
hold on
plot(30,max(thigh_30)-min(thigh_30),'*');
plot(25,max(thigh_25)-min(thigh_25),'*');
plot(20,max(thigh_20)-min(thigh_20),'*');

plot(0,max(thigh_0)-min(thigh_0),'*');
plot(-20,max(thigh_d20)-min(thigh_d20),'*');
plot(-25,max(thigh_d25)-min(thigh_d25),'*');
plot(-30,max(thigh_d30)-min(thigh_d30),'*');
plot(-35,max(thigh_d35)-min(thigh_d35),'*');
xlabel('Incline')
ylabel('Thigh ROM')
legend('35^o','30^o','25^o','20^o','0^o','-20^o','-25^o','-30^o','-35^o')
grid on

figure
plot(35, thigh_35);
hold on
plot(20, thigh_30);
plot(t, thigh_25);
plot(t, thigh_20);


plot(t,thigh_0);
plot(t, thigh_d20);
plot(t, thigh_d25);
plot(t, thigh_d30);
plot(t, thigh_d35);



legend('35^o','30^o','25^o','20^o','0^o','-20^o','-25^o','-30^o','-35^o')

