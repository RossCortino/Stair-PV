function [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,spo, c, prevState,prevPV, sm, qhm,mhf,pv_swing_thresh)


    if prevPV >= pv_swing_thresh && prevPV < 1 %TO to MHF
        FC = 0;
    elseif prevPV >= 0 && prevPV <= .07 %MHF to HS
        FC = 0;
    else %Stance
        FC = 1;
    end

    descendingPV = (qh_max-thigh)/(qh_max-qh_min)*c;
    ascendingPV =  1 + (1-sm)*(thigh-qh_max)/(qh_max-qhm);

    if prevState == 1 % Late Swing and Early Stance
        currPV = descendingPV;
        if (currPV >= spo && FC == 1)
            currState=2;
        end
    elseif prevState == 2 % Pushoff onset part of Stance
        currPV = descendingPV;
        if (thighd > 0)
            currState = 3;
            sm = descendingPV;
            qhm = thigh;
        end
        
    elseif prevState == 3 % Pushoff and late Stance
        currPV = max(prevPV,ascendingPV); %Phase is not allowed to decrease
        if FC == 0
            currState = 4;
        end
    else % Swing till MHF
        currPV = max(prevPV,ascendingPV); %Phase is not allowed to decrease
        
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