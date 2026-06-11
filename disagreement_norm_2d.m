function d = disagreement_norm_2d(P)
%DISAGREEMENT_NORM_2D Compute sqrt(sum_i ||p_i - pbar||^2) over time.
%
% P is K x N x 2.

    K = size(P, 1);
    N = size(P, 2);
    d = zeros(K, 1);

    for k = 1:K
        Pk = squeeze(P(k, :, :));
        pbar = mean(Pk, 1);

        err_sum = 0;
        for i = 1:N
            e = Pk(i, :) - pbar;
            err_sum = err_sum + e * e';
        end

        d(k) = sqrt(err_sum);
    end
end
