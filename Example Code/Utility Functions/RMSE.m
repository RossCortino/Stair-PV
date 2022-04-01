function [rmse] = RMSE(est,ref)
%RMSE Summary of this function goes here
%   Detailed explanation goes here


rmse = sqrt(sum((est-ref).^2)./length(est));
end

