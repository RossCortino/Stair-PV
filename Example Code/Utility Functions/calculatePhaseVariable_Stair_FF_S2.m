function [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair_FF_S2(thigh, qh_min, qh_max, s_po, c, prevState, prevPV, sm, qhm, mhf, pv_swing_thresh, phaseRateScaler)


    persistent timeForCalculatingPhaseDot dt pushoffPhaseDot s_FC s_to q_to
    
    if isempty(dt)
        
        timeForCalculatingPhaseDot = 0;
        pushoffPhaseDot = 0;
        dt = 1/150;
        s_to = 0;
        q_to = 0;
        s_FC = 0;
    end
    
    maxPhaseRateforFF = 2;    
    
    if prevPV >= pv_swing_thresh && prevPV < 1 % TO to MHF
        FC = 0;
    elseif prevPV >= 0 && prevPV <= .07 % From MHF to HS
        FC = 0;
    else % Stance
        FC = 1;
    end

    descendingPV = (qh_max-thigh)/(qh_max-qh_min)*c; % S1 & S2
    ascendingPV_S4 =  (thigh-q_to)/(qh_max-q_to)*(1-s_to)+s_to; %S4

    if prevState == 1
        currPV = descendingPV;
        if FC == 1
            currState = 2;
            s_FC = prevPV;
        else
            currState = 1;
        end
    elseif prevState == 2
        currPV = descendingPV;
        if (currPV >= s_po)
            currState=3;
            pushoffPhaseDot = clamp((currPV-s_FC)/timeForCalculatingPhaseDot,0,maxPhaseRateforFF);
            timeForCalculatingPhaseDot = 0;
        elseif currPV >= s_FC
            timeForCalculatingPhaseDot = timeForCalculatingPhaseDot+dt;
            currState = 2;
        end
        
    elseif prevState == 3
        
        currPV = min((prevPV+phaseRateScaler*pushoffPhaseDot*dt),.85);
        
        if FC == 0
            currState = 4;
            s_to = currPV;
            q_to = thigh;
        else
            currState = 3;
        end
    else
        currPV = ascendingPV_S4;
        
        if mhf == 1 || FC == 1
            currState = 1;
            mhf = 0;
        end
    end


    if currPV > 1
        currPV =1;

    elseif currPV < 0
        currPV = 0;
    
    end

    if ~exist('currState','var')
        currState = prevState;
    end
    
%     if currState ~= 1
        currPV = filterPhase(currPV,prevPV,1);
%     end

end