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


prompt = 'Leave out 20deg(1), 25deg(2), 30deg(3), 35deg(4): ';
leftOutInc = input(prompt);

incline={'i20'};

[thigh_20, knee_20, ankle_20, thigh_20sd, knee_20sd, ankle_20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i25'};

[thigh_25, knee_25, ankle_25, thigh_25sd, knee_25sd, ankle_25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i30'};

[thigh_30, knee_30, ankle_30, thigh_30sd, knee_30sd, ankle_30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i35'};

[thigh_35, knee_35, ankle_35, thigh_35sd, knee_35sd, ankle_35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s1'};
incline={'i0'};
[thigh_0, knee_0, ankle_0, thigh_0sd, knee_0sd, ankle_0sd] = averageJointKinematics_Walk(Normalized,sub,trial,incline);

switch leftOutInc
    case 1
        rmIndex = 4;
        incline={'i20'}
        thigh_traj = [thigh_0; thigh_25; thigh_30; thigh_35];
        thigh_stream = thigh_20;
        knee_stream = knee_20;
        ankle_stream = ankle_20;
        ankle_sd =ankle_20sd;
        knee_sd =knee_20sd;
        
        strIncline = '20^o';
        
         leg = ["Level Ground Walking", "25^o", "30^o", "35^o", "Unknown (20^o)"];
        
    case 2
        rmIndex = 3;
        incline={'i25'}
        thigh_traj = [thigh_0; thigh_20; thigh_30; thigh_35];
        thigh_stream = thigh_25;
        knee_stream = knee_25;
        ankle_stream = ankle_25;
        ankle_sd =ankle_25sd;
        knee_sd =knee_25sd;
        
        strIncline = '25^o';
        
        leg = ["Level Ground Walking", "20^o", "30^o", "35^o", "Unknown (25^o)"];
    case 3
        rmIndex = 2;
        incline={'i30'}
        thigh_traj = [thigh_0; thigh_20; thigh_25; thigh_35];
        thigh_stream = thigh_30;
        knee_stream = knee_30;
        ankle_stream = ankle_30;
        ankle_sd =ankle_30sd;
        knee_sd =knee_30sd;
        
        strIncline = '30^o';
        
        leg = ["Level Ground Walking", "20^o", "25^o", "35^o", "Unknown (30^o)"];
    case 4
        rmIndex = 1;
        incline={'i35'}
        thigh_traj = [thigh_0; thigh_20; thigh_25; thigh_30];
        thigh_stream = thigh_35;
        knee_stream = knee_35;
        ankle_stream = ankle_35;
        ankle_sd =ankle_35sd;
        knee_sd =knee_35sd;
        
        strIncline = '35^o';
        
        leg = ["Level Ground Walking", "20^o", "25^o", "30^o", "Unknown (35^o)"];
    otherwise
        rmIndex = 4;
        incline={'i20'}
        thigh_traj = [thigh_0; thigh_25; thigh_30; thigh_35];
        thigh_stream = thigh_20;
        knee_stream = knee_20;
        ankle_stream = ankle_20;
        ankle_sd =ankle_20sd;
        knee_sd =knee_20sd;
        
        strIncline = '20^o';
        
        leg = ["Level Ground Walking", "20^o", "30^o", "35^o", "Unknown (25^o)"];
end


phase = estimatedAngles.phase.(incline{1});
knee_org_est = estimatedAngles.knee.(incline{1});
ankle_org_est = estimatedAngles.ankle.(incline{1});
knee(:,rmIndex) = [];
ankle(:,rmIndex) = [];

q_h_max = X.thigh.maxima(rmIndex);
q_h_min = X.thigh.minima(rmIndex);

predeterminedIncline = false;
ascend = 1;
b = generateWeightedMatrix_leftOut(q_h_max, q_h_min, X, predeterminedIncline, ascend, 0,rmIndex);



for i = 1:length(phase)
    hk = knee_VC_func(phase(i));
    ha = ankle_VC_func(phase(i));
    knee_FC = knee*b;
    ankle_FC = ankle*b;
    
    knee_des(i) = hk*knee_FC;
    ankle_des(i) = ha*ankle_FC;
end



knee_vel_d = 0;
ankle_vel_d = 0;

gc = linspace(0,100,length(phase));





figure
sgtitle(strcat("Incline Stair Walking Cross Validation: Leave Out ",num2str(strIncline)))
subplot(311)
plot(gc,thigh_traj(1,:),'linewidth',2)
hold on
plot(gc,thigh_traj(2,:),'linewidth',2)
plot(gc,thigh_traj(3,:),'linewidth',2)
plot(gc,thigh_traj(4,:),'linewidth',2)
plot(gc,thigh_stream,'--','linewidth',2)

grid on
legend(leg)
ylabel('Thigh Angle (^o)')
title('Thigh Joint Position')

% subplot(412)
% yyaxis left
% plot(gc,phase,'k')
% ylabel('Phase Variable')
% yyaxis right
% plot(gc,state)
% ylabel('State')
% title('Phase/State')
% grid on

kfiltcoeff = .3;
afiltcoeff = .5;

subplot(312)


k1 = knee_stream+knee_sd;
k2 = knee_stream-knee_sd;

gc2 = [gc, fliplr(gc)];
inBetween = [k1,fliplr(k2)];
pk1 = fill(gc2,inBetween,[0.3010, 0.7450, 0.9330], 'linestyle','None');
hold on
pk2 = plot(gc, knee_stream,'linewidth',2,'color',[0, 0.4470, 0.7410]);
pk3 = plot(gc, knee_org_est,'linewidth',2, 'color',[0.8500, 0.3250, 0.0980]);
pk4 = plot(gc, knee_des, '--','color','black','linewidth',2);
% xlabel('samples')
ylabel('Knee Angle (^o)')
title('Knee Joint Position')
legend([pk2,pk3,pk4],'Raw','Est(Orig Inc and VC interp)','Est(Leg 2)','Location','Southeast')
grid on
% kneeRMSE_incvc = RMSE(knee_incvc_est,knee_stream)
% kneeRMSE_inc = RMSE(knee_inc_est,knee_stream)

subplot(313)

a1 = ankle_stream+ankle_sd;
a2 = ankle_stream-ankle_sd;

gc2 = [gc, fliplr(gc)];
inBetween = [a1,fliplr(a2)];
pa1 = fill(gc2,inBetween,[0.3010, 0.7450, 0.9330], 'linestyle','None');
hold on
pa2 = plot(gc, ankle_stream,'linewidth',2,'color',[0, 0.4470, 0.7410]);
pa3 = plot(gc, ankle_org_est,'linewidth',2, 'color',[0.8500, 0.3250, 0.0980]);
pa4 = plot(gc, ankle_des,'--','color','black','linewidth',2);
legend([pa2,pa3,pa4],'Able-Bodied','Est(Orig Inc and VC interp)','Est(Leg 2 Inc and VC Interp)','Location','Southeast')
ylabel('Ankle Angle (^o)')
xlabel('Gait Cycle (%)','FontWeight','bold')
title('Ankle Joint Position')
% ankleRMSE_incvc = RMSE(ankle_incvc_est,ankle_stream)
% ankleRMSE_inc = RMSE(ankle_inc_est,ankle_stream)
grid on






function [b] = generateWeightedMatrix_leftOut(q_h_max, q_h_min, X, predeterminedIncline, ascend, inclineAngle,rmIndex)
%CALCSTAIRVC Summary of this function goes here
%   Detailed explanation goes here
% uses pre-defined angle
b = zeros(8,1);
closest_max = zeros(1,2);
closest_min = zeros(1,2);
closest_incline = zeros(1,2);

knownMaxima = X.thigh.maxima;
knownMinima = X.thigh.minima;
knownInclines = X.inclines;




if predeterminedIncline
    switch inclineAngle
        case 35
            b(1) = 1;
        case 30
            b(2) = 1;
        case 25
            b(3) = 1;
        case 20
            b(4) = 1;
        case 0
            b(5) = 1;
        case -20
            b(6) = 1;
        case -25
            b(7) = 1;
        case -30
            b(8) = 1;
        case -35
            b(9) = 1;
        otherwise
            [~,indexOfMin] = min(abs(X.inclines-inclineAngle));
            b(indexOfMin) = 1;
    end
else
    if ascend == 1
        
       
        
        x = q_h_max;
        %bound 1
        usefulMaxima = knownMaxima(1:5);
        usefulMaxima(rmIndex) = [];
        
        usefulInclines = knownInclines(1:5);
        usefulInclines(rmIndex) = [];
        
        [~,min_Index] = min(abs(usefulMaxima-x));
        bounds = min_Index;
        
        if (usefulMaxima(min_Index) <= x || usefulMaxima(min_Index) == usefulMaxima(end)) && usefulMaxima(min_Index) ~= usefulMaxima(1)
            min_Index2 = 1;
        else
            min_Index2 = 2;
        end
        
        if min_Index2 == 1
            %Closest  % 2ndClosest
            bounds = [min_Index-1 min_Index];
            closest_incline =  usefulInclines(bounds);
            closest_max = usefulMaxima(bounds);
            
            x0 = closest_max(2);
            y0 = closest_incline(2);
            
            x1 = closest_max(1);
            y1 = closest_incline(1);
            
            
            
            b1 = (x-x0)/(x1-x0);
            b0 = 1-b1;
            b;
            b(bounds)= [b1 b0];
            
        else
            bounds = [min_Index min_Index+1];
            
            closest_incline =  usefulInclines(bounds);
            closest_max = usefulMaxima(bounds);
            
            x0 = closest_max(1);
            y0 = closest_incline(1);
            
            x1 = closest_max(2);
            y1 = closest_incline(2);
            
            b1 = (x-x0)/(x1-x0);
            b0 = 1-b1;
            b;
            b(bounds)= [b0 b1];
        end
        %         knownInclines*b
        %
        %
        %         knownMaxima*b
        
    else
        x = q_h_min;
        %bound 1
        usefulMinima = knownMinima(6:end);
        [~,min_Index] = min(abs(usefulMinima-x));
        % closest_max(1) = knownMaxima(min_Index);
        % closest_incline(1) = knownInclines(min_Index);
        bounds = min_Index;
        
        if (usefulMinima(min_Index) >= x || usefulMinima(min_Index) == usefulMinima(end)) && usefulMinima(min_Index) ~= usefulMinima(1)
            min_Index2 = 1;
        else
            min_Index2 = 2;
        end
        
        %         boundMatrix = [abs(usefulMinima(min_Index-1)-q_h_min), abs(usefulMinima(min_Index+1)-q_h_min)];
        %
        %         [~,min_Index2] = min(boundMatrix);
        if min_Index2 == 1
            %Closest  % 2ndClosest
            bounds = [min_Index-1 min_Index];
            closest_incline =  knownInclines(bounds);
            closest_min = usefulMinima(bounds);
            
            x0 = closest_min(2);
            y0 = closest_incline(2);
            
            x1 = closest_min(1);
            y1 = closest_incline(1);
            
            
            
            b1 = (x-x0)/(x1-x0);
            b0 = 1-b1;
            %             b
            b(bounds+(length(knownMinima)-length(usefulMinima)))= [b1 b0];
            
        else
            bounds = [min_Index min_Index+1];
            
            closest_incline =  knownInclines(bounds);
            closest_min = usefulMinima(bounds);
            
            x0 = closest_min(1);
            y0 = closest_incline(1);
            
            x1 = closest_min(2);
            y1 = closest_incline(2);
            
            b1 = (x-x0)/(x1-x0);
            b0 = 1-b1;
            %             b
            b(bounds+(length(knownMinima)-length(usefulMinima)))= [b0 b1];
        end
        %         knownInclines*b
        %
        %
        %         knownMinima*b
        
        
    end
end

% estimatedIncline = knownInclines*b;

end

function ha = ankle_VC_func(phase)
%ANKLE_VC_FUNC
%    HA = ANKLE_VC_FUNC(PHASE)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    06-Aug-2021 09:50:01

t2 = phase.*pi.*2.0;
t3 = phase.*pi.*4.0;
t4 = phase.*pi.*6.0;
t5 = phase.*pi.*8.0;
t6 = phase.*pi.*1.0e+1;
t7 = phase.*pi.*1.2e+1;
ha = [1.0./2.0,cos(t2),-sin(t2),cos(t3),-sin(t3),cos(t4),-sin(t4),cos(t5),-sin(t5),cos(t6),-sin(t6),cos(t7),-sin(t7),cos(phase.*pi.*1.4e+1)./2.0];

end
function hk = knee_VC_func(phase)
%KNEE_VC_FUNC
%    HK = KNEE_VC_FUNC(PHASE)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    06-Aug-2021 09:50:01

t2 = phase.*pi.*2.0;
t3 = phase.*pi.*4.0;
t4 = phase.*pi.*6.0;
t5 = phase.*pi.*8.0;
t6 = phase.*pi.*1.0e+1;
t7 = phase.*pi.*1.2e+1;
hk = [1.0./2.0,cos(t2),-sin(t2),cos(t3),-sin(t3),cos(t4),-sin(t4),cos(t5),-sin(t5),cos(t6),-sin(t6),cos(t7),-sin(t7),cos(phase.*pi.*1.4e+1)./2.0];
end
