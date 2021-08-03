function [currPV,currState,sm,qhm,mhf,mhe] = calculatePhaseVariable_Stair_MHE(thigh, thighd, vGRF, qh_min, qh_max, qpo, c, prevState,prevPV, sm, qhm,mhf,mhe,FC_thresh)





    if vGRF < FC_thresh
        FC = 0;
    else
        FC = 1;
    end

%     FC
%     prevState
    if (prevState ==1 && thigh < qpo && FC == 1)
        currState=2;
    elseif (prevState == 2 && thighd > 0 && mhe == 1)
        currState = 3;
        sm = prevPV;
        qhm = thigh;
        mhe =0;
    elseif (prevState == 4 && (mhf == 1 || FC == 1))
        currState = 1;
        mhf = 0;
    elseif FC == 0 && prevState == 3
        
        if prevState == 2
            sm = prevPV;
            qhm = thigh;
        end
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