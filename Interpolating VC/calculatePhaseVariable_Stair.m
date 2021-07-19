function [currPV,currState,sm,qhm,mhf] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max, c, prevState,prevPV, sm, qhm,mhf)





    if prevPV >= .68 && prevPV < 1
        FC = 0;
    else
        FC = 1;
    end

%     FC
%     prevState
    if (prevState ==1 && prevPV > .3 && prevPV < .57 && FC == 1)
        currState=2;
    elseif (prevState == 2 && thighd > 0)
        currState = 3;
        sm = prevPV;
        qhm = thigh;
    elseif (prevState == 4 && mhf == 1)
        currState = 1;
        mhf = 0;
    elseif FC == 0
        currState = 4;
    else
        currState = prevState;
    end

    if(currState == 1 || currState == 2)
        currPV = (qh_max-thigh)/(qh_max-qh_min)*c;
    elseif (currState == 3 || currState == 4)
        currPV = 1 + (1-sm)*(thigh-qh_max)/(qh_max-qhm);
    end


    if (currState == 3 && (prevPV > currPV))
        currPV=prevPV;
    end


    if currPV > 1
        currPV =1;

    elseif currPV < 0
        currPV = 0;
    
    end




end