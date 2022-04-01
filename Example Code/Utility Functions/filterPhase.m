function phaseFiltered = filterPhase(phase, phasePrev, filter_coeff)

% if (reset)
%     phasePrev = phase;
% end

phaseFiltered = (1 - filter_coeff) * phasePrev + filter_coeff * phase ;

% Store outputs
% phasePrev = phaseFiltered;
end