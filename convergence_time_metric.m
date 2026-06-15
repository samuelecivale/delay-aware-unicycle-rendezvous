function tc = convergence_time_metric(time, disagreement, threshold)
%CONVERGENCE_TIME_METRIC First time after which disagreement stays below threshold.
time = time(:);
disagreement = disagreement(:);
tc = NaN;
for k = 1:length(time)
    if all(disagreement(k:end) <= threshold)
        tc = time(k);
        return;
    end
end
end
