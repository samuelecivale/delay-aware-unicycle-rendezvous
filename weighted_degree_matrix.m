function D = weighted_degree_matrix(A)
%WEIGHTED_DEGREE_MATRIX Return the diagonal weighted degree matrix of A.
if ~ismatrix(A) || size(A,1) ~= size(A,2)
    error('weighted_degree_matrix:Input', 'A must be a square matrix.');
end
A = double(A);
D = diag(sum(A, 2));
end
