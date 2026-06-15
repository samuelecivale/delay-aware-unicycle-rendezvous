function plot_2d_trajectories_with_obstacles(P, title_str, obstacles, target)
%PLOT_2D_TRAJECTORIES_WITH_OBSTACLES Plot trajectories and circular obstacles.
P = double(P); N = size(P,2);
figure; hold on;
for i = 1:N
    plot(P(:,i,1), P(:,i,2), 'LineWidth', 1.4);
    plot(P(1,i,1), P(1,i,2), 'o', 'MarkerSize', 5);
    plot(P(end,i,1), P(end,i,2), 'x', 'MarkerSize', 8, 'LineWidth', 1.2);
end
if nargin >= 3 && ~isempty(obstacles)
    th = linspace(0, 2*pi, 160);
    for o = 1:length(obstacles)
        c = obstacles(o).center(:)'; r = obstacles(o).radius;
        plot(c(1) + r*cos(th), c(2) + r*sin(th), 'LineWidth', 2.0);
        plot(c(1), c(2), 'x', 'MarkerSize', 8);
    end
end
if nargin >= 4 && ~isempty(target)
    plot(target(1), target(2), 's', 'MarkerSize', 9, 'LineWidth', 1.5);
end
grid on; axis equal; xlabel('x'); ylabel('y'); title(title_str); hold off;
end
