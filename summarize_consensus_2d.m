function summary = summarize_consensus_2d(time, P, threshold)
%SUMMARIZE_CONSENSUS_2D Compute final, max disagreement and convergence time.
if nargin < 3
    threshold = 1e-2;
end
d = disagreement_norm_2d(P);
Pend = squeeze(P(end,:,:));
summary = struct();
summary.final_disagreement = d(end);
summary.max_disagreement = max(d);
summary.convergence_time = convergence_time_metric(time, d, threshold);
summary.final_centroid = mean(Pend, 1);
end
