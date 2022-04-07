function [b] = generateWeightedMatrix(q_h_max, q_h_min, X, predeterminedIncline, ascend, inclineAngle)
%CALCSTAIRVC Summary of this function goes here
%   Detailed explanation goes here
% uses pre-defined angle
b = zeros(9,1);
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
        usefulInclines = knownInclines(1:5);
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
        usefulInclines = knownInclines(6:end);
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
            closest_incline =  usefulInclines(bounds);
            closest_min = usefulMinima(bounds);
            
            x0 = closest_min(2);
            y0 = closest_incline(2);
            
            x1 = closest_min(1);
            y1 = closest_incline(1);
            
            
            
            b1 = (x-x0)/(x1-x0);
            b0 = 1-b1;
            %             b
            b(bounds+5)= [b1 b0];
            
        else
            bounds = [min_Index min_Index+1];
            
            closest_incline =  usefulInclines(bounds);
            closest_min = usefulMinima(bounds);
            
            x0 = closest_min(1);
            y0 = closest_incline(1);
            
            x1 = closest_min(2);
            y1 = closest_incline(2);
            
            b1 = (x-x0)/(x1-x0);
            b0 = 1-b1;
            %             b
            b(bounds+5)= [b0 b1];
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