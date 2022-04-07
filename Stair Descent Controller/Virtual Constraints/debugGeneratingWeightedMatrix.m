addpath('Utility Functions')
clearvars -except Streaming Normalized R01 rawR01 stairVCTrajectories X;
close all
clc;


b = zeros(9,1);
closest_max = zeros(1,2);
closest_incline = zeros(1,2);

knownMaxima = X.thigh.maxima;
knownMinima = X.thigh.minima;
knownInclines = X.inclines;
predeterminedIncline = false;
q_h_max = knownMaxima(4);%mean([49.0250 46.0441]);
q_h_min = knownMinima(end);
ascend = 1;





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
        [~,min_Index] = min(abs(usefulMaxima(1:5)-q_h_max));
        bounds = min_Index;
        
        if (usefulMaxima(min_Index) <= q_h_max || usefulMaxima(min_Index) == usefulMaxima(end)) && usefulMaxima(min_Index) ~= usefulMaxima(1)
            min_Index2 = 1;
        else
            min_Index2 = 2;
        end
        
        if min_Index2 == 1
            %Closest  % 2ndClosest
            bounds = [min_Index-1 min_Index];
            closest_incline =  knownInclines(bounds);
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
            
            closest_incline =  knownInclines(bounds);
            closest_max = knownMaxima(bounds);
            
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
        [~,min_Index] = min(abs(usefulMinima-q_h_min));
        % closest_max(1) = knownMaxima(min_Index);
        % closest_incline(1) = knownInclines(min_Index);
        bounds = min_Index;
        
        if (usefulMinima(min_Index) >= q_h_min || usefulMinima(min_Index) == usefulMinima(end)) && usefulMinima(min_Index) ~= usefulMinima(1)
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
    
    
    
    
    % b(min(bounds)) = b0
    % b(max(bounds)) = b1