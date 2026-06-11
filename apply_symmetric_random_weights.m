function A_weighted = apply_symmetric_random_weights(A_binary, w_min, w_max, seed)
%APPLY_SYMMETRIC_RANDOM_WEIGHTS Assign positive symmetric weights to a binary graph.

    if nargin >= 4 && ~isempty(seed)
        rng(seed);
    end

    N = size(A_binary, 1);
    A_weighted = zeros(N, N);

    for i = 1:N
        for j = (i+1):N
            if A_binary(i, j) > 0
                w = w_min + (w_max - w_min) * rand();
                A_weighted(i, j) = w;
                A_weighted(j, i) = w;
            end
        end
    end
end
