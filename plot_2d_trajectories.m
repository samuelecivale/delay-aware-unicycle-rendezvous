function plot_2d_trajectories(P, title_str, target)
%PLOT_2D_TRAJECTORIES Plot 2D agent trajectories.
P = double(P);
K = size(P,1); N = size(P,2);
figure; hold on;
for i = 1:N
    plot(P(:,i,1), P(:,i,2), 'LineWidth', 1.4);
    plot(P(1,i,1), P(1,i,2), 'o', 'MarkerSize', 5);
    plot(P(end,i,1), P(end,i,2), 'x', 'MarkerSize', 8, 'LineWidth', 1.2);
end
if nargin >= 3 && ~isempty(target)
    plot(target(1), target(2), 's', 'MarkerSize', 9, 'LineWidth', 1.5);
end
grid on; axis equal;
xlabel('x'); ylabel('y');
title(title_str);
hold off;
end
