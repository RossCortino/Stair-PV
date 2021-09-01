%%


close all






trial = {'t1'};
strides = {'s1','s2','s3','s4','s5','s6','s7','s8'}
phaseEstimate_all = ControllerOutputs.phaseEstimate';

state = ControllerOutputs.state_out';

thigh_pos
prevState = state(1);

strideNum = 1;
fullcycle = false;

start_ind = 1;

phaseEstimate = [];
zero_flag = true;
des_samples = 500;
for i = 1:length(phaseEstimate_all)
    
    if state(i) == 1 && prevState == 4 
        if fullcycle
            for j = start_ind+1:i-1
                if phaseEstimate_all(j) >0 && zero_flag
                    start_ind = j-1;
                    zero_flag = false;
                end
            end
            pv = phaseEstimate_all(start_ind:i-1);
            while length(pv) ~= des_samples
                pv = smooth(interp1(1:length(pv), pv, 1:length(pv)/des_samples:length(pv)));
            end
            phaseEstimate(strideNum,:) = pv;
            figure
            hold off
            plot(phaseEstimate(strideNum,:))
            
            stairAscentTrialData.(trial{1}).(strides{strideNum}).phaseEstimate = phaseEstimate(strideNum,:)';
            strideNum = strideNum + 1;
            fullcycle = false;
        end
        start_ind = i;
    end
    
    if state(i) == 2
        fullcycle = true;
        zero_flag = true;
    end
    
    prevState = state(i);
end





