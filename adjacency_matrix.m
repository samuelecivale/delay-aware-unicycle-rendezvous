function A = adjacency_matrix(graph_type, N)
%ADJACENCY_MATRIX Generate canonical undirected graphs.
%
% graph_type:
%   'path'         chain graph
%   'ring'         cycle graph
%   'star'         node 1 connected to all others
%   'complete'     all-to-all graph
%   'disconnected' two separated path components

    A = zeros(N, N);

    if strcmp(graph_type, 'path')
        for i = 1:(N-1)
            A(i, i+1) = 1;
            A(i+1, i) = 1;
        end

    elseif strcmp(graph_type, 'ring')
        for i = 1:N
            j = mod(i, N) + 1;
            A(i, j) = 1;
            A(j, i) = 1;
        end

    elseif strcmp(graph_type, 'star')
        for i = 2:N
            A(1, i) = 1;
            A(i, 1) = 1;
        end

    elseif strcmp(graph_type, 'complete')
        A = ones(N, N) - eye(N);

    elseif strcmp(graph_type, 'disconnected')
        split = floor(N/2);

        for i = 1:(split-1)
            A(i, i+1) = 1;
            A(i+1, i) = 1;
        end

        for i = split:(N-1)
            if i >= split + 1
                A(i, i+1) = 1;
                A(i+1, i) = 1;
            end
        end

    else
        error('Unknown graph_type.');
    end
end
