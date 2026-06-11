function D = weighted_degree_matrix(A)
%WEIGHTED_DEGREE_MATRIX Weighted degree matrix.
    D = diag(sum(A, 2));
end
