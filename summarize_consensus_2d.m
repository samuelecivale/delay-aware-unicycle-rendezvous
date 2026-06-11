function summary = summarize_consensus_2d(time, P, eps_value)
%SUMMARIZE_CONSENSUS_2D Return 2D disagreement metrics.

    d = disagreement_norm_2d(P);

    summary.initial_disagreement = d(1);
    summary.final_disagreement = d(end);
    summary.max_disagreement = max(d);
    summary.convergence_time = convergence_time_metric(time, d, eps_value);
end
