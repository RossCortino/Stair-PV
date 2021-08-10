function filterVal = firstOrderLowpass(x,coeff)
    filterVal = x;
    for i = 2:length(x)
        
        filterVal(i) = (1-coeff)*filterVal(i-1)+coeff*x(i);
    end

    
end