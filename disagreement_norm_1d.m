function d = disagreement_norm_1d(X)
%DISAGREEMENT_NORM_1D Euclidean norm around the instantaneous average.
X = double(X);
means = mean(X, 2);
diff = X - means;
d = sqrt(sum(diff.^2, 2));
end
