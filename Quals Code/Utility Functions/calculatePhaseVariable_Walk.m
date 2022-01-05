function [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Walk(thigh, thighd, qh_min, qh_max,qpo, c, prevState,prevPV, sm, qhm,mhf,pv_swing_thresh)





    if prevPV >= pv_swing_thresh %&& prevPV < 1
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
        if (thigh < qpo && FC == 1)
            currState=2;
        end
    elseif prevState == 2
        currPV = descendingPV;
        if (thighd > 0)
            currState = 3;
            sm = descendingPV;
            qhm = thigh;
        end
        
    elseif prevState == 3
        currPV = max(prevPV,ascendingPV);
        if FC == 0
            currState = 4;
        end
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



end