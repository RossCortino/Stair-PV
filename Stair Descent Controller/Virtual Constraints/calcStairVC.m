function [knee_des, ankle_des, knee_vel_d, ankle_vel_d, estimatedIncline] = calcStairVC(phase, HS, q_h_max, q_h_min, predeterminedIncline, ascend, inclineAngle)
%CALCSTAIRVC Summary of this function goes here
%   Detailed explanation goes here

persistent b

X = controllerConstants_StairAscent();
knownInclines = X.inclines;
knee = X.knee;
ankle = X.ankle;

if isempty(b) || HS == 1
    b = generateWeightedMatrix(q_h_max, q_h_min, X, predeterminedIncline, ascend, inclineAngle);    
end

hk = knee_VC_func(phase);
ha = ankle_VC_func(phase);

knee_FC = knee*b;
ankle_FC = ankle*b;

knee_des = hk*knee_FC;
ankle_des = ha*ankle_FC;

knee_vel_d = 0;
ankle_vel_d = 0;

estimatedIncline = knownInclines*b;
end


