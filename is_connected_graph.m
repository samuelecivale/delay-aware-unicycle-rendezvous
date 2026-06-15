function connected = is_connected_graph(A)
%IS_CONNECTED_GRAPH Breadth-first connectivity test for undirected graphs.
A = double(A) > 0;
N = size(A,1);
if N == 0
    connected = false;
    return;
end
visited = false(N,1);
queue = zeros(N,1);
head = 1; tail = 1;
queue(tail) = 1;
visited(1) = true;
while head <= tail
    i = queue(head); head = head + 1;
    neigh = find(A(i,:));
    for j = neigh
        if ~visited(j)
            tail = tail + 1;
            queue(tail) = j;
            visited(j) = true;
        end
    end
end
connected = all(visited);
end
