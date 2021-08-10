function [temp_traj,qh_max, mhf] = MHFDetection(thigh,temp_traj,qh_max, mhf,thresh,minpeakh,detectionStartThresh)
if thigh > detectionStartThresh %&& i-p > minpeakd
    temp_traj = [temp_traj thigh];
    if length(temp_traj) > 3
        [pks,locs] = findpeaks(temp_traj,'threshold', thresh,'MinPeakHeight',minpeakh);
        if ~isempty(pks)
            temp_traj = [];
%             ypeak = [ypeak pks];
%             xpeak = [xpeak p];
%             temp_max = qh_max;
            qh_max = ypeak;
            mhf = 1;
        end
    end
    
end
end

