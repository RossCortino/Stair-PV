function [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair_FF(thigh, thighd, qh_min, qh_max,qpo, c, prevState,prevPV, sm, qhm,mhf,pv_swing_thresh)


    persistent timeForCalculatingPhaseDot dt pushoffPhaseDot s_po
    
    if isempty(dt)
        
        timeForCalculatingPhaseDot = 0;
        pushoffPhaseDot = 0;
        dt = 1/150;
        s_po = 0;
    end
    s_po = .4;
    s_FC = .3;
    maxPhaseRateforFF = 2;
    minPhaseRateForFF = .4;
    if prevPV >= pv_swing_thresh && prevPV < 1 
        FC = 0;
    else
        FC = 1;
    end

%     FC
%     prevState
    descendingPV = (qh_max-thigh)/(qh_max-qh_min)*c;
    ascendingPV =  1 + (1-sm)*(thigh-qh_max)/(qh_max-qhm);
%     FC
%     prevState

    if prevState == 1
        currPV = descendingPV;
        
        if (currPV >= s_po && FC == 1)
            currState=2;
            pushoffPhaseDot = clamp((s_po-s_FC)/timeForCalculatingPhaseDot,0,maxPhaseRateforFF);
            timeForCalculatingPhaseDot = 0;
        elseif currPV >= s_FC
            timeForCalculatingPhaseDot = timeForCalculatingPhaseDot+dt;
        end
    elseif prevState == 2
%         currPV = descendingPV;
        currPV = min(prevPV+pushoffPhaseDot*dt,pv_swing_thresh);
        if (thighd > 0)
%             currState = 3;
            sm = currPV;
            qhm = thigh;
%             pdot = (sm-s_po)/timeForCalculatingPhaseDot
%             pushoffPhaseDot = clamp((sm-s_po)/timeForCalculatingPhaseDot,0,maxPhaseRateforFF);
% %             timeForCalculatingPhaseDot = 0;
%         else
%             timeForCalculatingPhaseDot = timeForCalculatingPhaseDot+dt;
%             
        end
        
        if FC == 0
            currState = 4;
        end
        
    elseif prevState == 3
        currPV = min(prevPV+pushoffPhaseDot*dt,pv_swing_thresh);
%         currPV = ascendingPV;
        
    else
        currPV = ascendingPV;
        
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
        currPV = filterPhase(currPV,prevPV,.9);
%     end

end