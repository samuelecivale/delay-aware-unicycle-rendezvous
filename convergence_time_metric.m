function tc = convergence_time_metric(time, d, eps_value)
%CONVERGENCE_TIME_METRIC First time after which d remains below eps.
%
% Returns NaN if convergence is not reached.

    tc = NaN;

    for k = 1:length(d)
        if all(d(k:end) < eps_value)
            tc = time(k);
            return;
        end
    end
end
