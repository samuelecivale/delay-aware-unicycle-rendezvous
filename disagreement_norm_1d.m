function d = disagreement_norm_1d(X)
%DISAGREEMENT_NORM_1D Compute ||x - mean(x) 1|| over time.

    K = size(X, 1);
    d = zeros(K, 1);

    for k = 1:K
        x = X(k, :)';
        avg = mean(x);
        err = x - avg * ones(length(x), 1);
        d(k) = norm(err);
    end
end
