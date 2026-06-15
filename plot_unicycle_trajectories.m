function plot_unicycle_trajectories(states, title_str, show_headings, heading_step)
%PLOT_UNICYCLE_TRAJECTORIES Plot unicycle paths and optional headings.
if nargin < 3, show_headings = true; end
if nargin < 4, heading_step = 450; end
states = double(states);
K = size(states,1); N = size(states,2);
figure; hold on;
for i = 1:N
    plot(states(:,i,1), states(:,i,2), 'LineWidth', 1.4);
    plot(states(1,i,1), states(1,i,2), 'o', 'MarkerSize', 5);
    plot(states(end,i,1), states(end,i,2), 'x', 'MarkerSize', 8, 'LineWidth', 1.2);
    if show_headings
        for k = 1:heading_step:K
            x = states(k,i,1); y = states(k,i,2); th = states(k,i,3);
            quiver(x, y, 0.25*cos(th), 0.25*sin(th), 0, 'MaxHeadSize', 1.0);
        end
    end
end
grid on; axis equal;
xlabel('x'); ylabel('y'); title(title_str); hold off;
end
