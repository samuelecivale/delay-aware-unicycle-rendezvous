function d = disagreement_norm_2d(P)
%DISAGREEMENT_NORM_2D Frobenius norm around the instantaneous centroid.
P = double(P);
K = size(P,1);
d = zeros(K,1);
for k = 1:K
    Pk = squeeze(P(k,:,:));
    c = mean(Pk, 1);
    diff = Pk - c;
    d(k) = sqrt(sum(diff(:).^2));
end
end
