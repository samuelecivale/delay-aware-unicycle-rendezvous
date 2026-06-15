function summary = summarize_consensus_1d(time, X, threshold)
%SUMMARIZE_CONSENSUS_1D Compute final, max disagreement and convergence time.
if nargin < 3
    threshold = 1e-2;
end
d = disagreement_norm_1d(X);
summary = struct();
summary.final_disagreement = d(end);
summary.max_disagreement = max(d);
summary.convergence_time = convergence_time_metric(time, d, threshold);
summary.final_average = mean(X(end,:));
end
