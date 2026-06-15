function A = adjacency_matrix(graph_type, N)
%ADJACENCY_MATRIX Build common undirected adjacency matrices.
A = zeros(N,N);
graph_type = lower(graph_type);
switch graph_type
    case 'path'
        for i = 1:N-1
            A(i,i+1) = 1; A(i+1,i) = 1;
        end
    case 'ring'
        for i = 1:N
            j = mod(i, N) + 1;
            A(i,j) = 1; A(j,i) = 1;
        end
    case 'star'
        for i = 2:N
            A(1,i) = 1; A(i,1) = 1;
        end
    case 'complete'
        A = ones(N,N) - eye(N);
    case 'disconnected'
        % Two path components of approximately equal size.
        split = floor(N/2);
        for i = 1:split-1
            A(i,i+1) = 1; A(i+1,i) = 1;
        end
        for i = split+1:N-1
            A(i,i+1) = 1; A(i+1,i) = 1;
        end
    otherwise
        error('adjacency_matrix:UnknownType', 'Unknown graph type: %s', graph_type);
end
end
