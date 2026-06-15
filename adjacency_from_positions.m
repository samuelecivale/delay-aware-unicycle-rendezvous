function A = adjacency_from_positions(P, radius, weight)
%ADJACENCY_FROM_POSITIONS Build a geometric graph from 2D positions.
if nargin < 3, weight = 1.0; end
P = double(P);
N = size(P,1);
A = zeros(N,N);
for i = 1:N
    for j = i+1:N
        if norm(P(i,:) - P(j,:)) <= radius
            A(i,j) = weight;
            A(j,i) = weight;
        end
    end
end
end
