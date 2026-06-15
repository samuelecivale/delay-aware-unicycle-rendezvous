function out = min_obstacle_clearance(P, obstacles)
%MIN_OBSTACLE_CLEARANCE Minimum distance from obstacle boundary over agents.
P = double(P);
if ndims(P) == 2
    P = reshape(P, [1, size(P,1), size(P,2)]);
end
K = size(P,1); N = size(P,2);
if isempty(obstacles)
    out = NaN(K,1);
    return;
end
out = Inf(K,1);
for k = 1:K
    Pk = squeeze(P(k,:,:));
    for i = 1:N
        for o = 1:length(obstacles)
            c = obstacles(o).center(:)';
            r = obstacles(o).radius;
            out(k) = min(out(k), norm(Pk(i,:) - c) - r);
        end
    end
end
end
