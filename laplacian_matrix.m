function L = laplacian_matrix(A)
%LAPLACIAN_MATRIX Compute weighted graph Laplacian L = D - A.
    degrees = sum(A, 2);
    D = diag(degrees);
    L = D - A;
end
