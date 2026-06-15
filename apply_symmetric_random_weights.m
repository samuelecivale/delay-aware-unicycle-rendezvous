function A_w = apply_symmetric_random_weights(A, w_min, w_max, seed)
%APPLY_SYMMETRIC_RANDOM_WEIGHTS Replace existing undirected edges with random weights.
if nargin >= 4 && ~isempty(seed)
    rng(seed);
end
A = double(A);
N = size(A,1);
A_w = zeros(N,N);
for i = 1:N
    for j = i+1:N
        if A(i,j) > 0 || A(j,i) > 0
            w = w_min + (w_max - w_min) * rand();
            A_w(i,j) = w;
            A_w(j,i) = w;
        end
    end
end
end
