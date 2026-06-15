function D = split_disturbance_connectivity(P_current, t)
%SPLIT_DISTURBANCE_CONNECTIVITY Disturbance that pushes two groups apart for t in [0,8].
D = zeros(size(P_current));
if t >= 0.0 && t <= 8.0
    center_x = mean(P_current(:,1));
    signs = ones(size(P_current,1),1);
    signs(P_current(:,1) < center_x) = -1.0;
    D(:,1) = 0.80 * signs;
end
end
