function summary = summarize_consensus_1d(time, X, eps_value)
%SUMMARIZE_CONSENSUS_1D Return disagreement metrics.

    d = disagreement_norm_1d(X);

    summary.initial_disagreement = d(1);
    summary.final_disagreement = d(end);
    summary.max_disagreement = max(d);
    summary.convergence_time = convergence_time_metric(time, d, eps_value);
end
