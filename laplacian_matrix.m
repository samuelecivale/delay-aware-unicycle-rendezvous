function L = laplacian_matrix(A)
%LAPLACIAN_MATRIX Return L = D - A for a binary or weighted graph.
A = double(A);
L = weighted_degree_matrix(A) - A;
end
