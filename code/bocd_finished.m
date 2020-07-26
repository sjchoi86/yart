function ret = bocd_finished(bocd)

% bocd.iter is updated when doing append 
if bocd.iter >= bocd.max_iter
    ret = 1;
else
    ret = 0;
end
