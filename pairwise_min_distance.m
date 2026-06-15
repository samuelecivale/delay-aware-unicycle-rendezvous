function out = pairwise_min_distance(P)
%PAIRWISE_MIN_DISTANCE Minimum pairwise distance for each time step.
P = double(P);
if ndims(P) == 2
    P = reshape(P, [1, size(P,1), size(P,2)]);
end
K = size(P,1); N = size(P,2);
out = Inf(K,1);
for k = 1:K
    Pk = squeeze(P(k,:,:));
    for i = 1:N
        for j = i+1:N
            out(k) = min(out(k), norm(Pk(i,:) - Pk(j,:)));
        end
    end
end
end
