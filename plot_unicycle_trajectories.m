function plot_unicycle_trajectories(states, title_str, show_headings, heading_step)
%PLOT_UNICYCLE_TRAJECTORIES Plot unicycle trajectories with optional heading arrows.

    K = size(states, 1);
    N = size(states, 2);

    figure('visible', 'off');
    hold on;

    for i = 1:N
        x = squeeze(states(:, i, 1));
        y = squeeze(states(:, i, 2));

        plot(x, y, 'LineWidth', 1.5);
        plot(x(1), y(1), 'o', 'MarkerSize', 6);
        plot(x(end), y(end), 'x', 'MarkerSize', 8, 'LineWidth', 1.5);

        if show_headings
            for k = 1:heading_step:K
                theta = states(k, i, 3);
                len = 0.25;
                quiver(states(k, i, 1), states(k, i, 2), ...
                       len*cos(theta), len*sin(theta), ...
                       0, 'LineWidth', 1.0, 'MaxHeadSize', 1.5);
            end
        end
    end

    grid on;
    axis equal;
    xlabel('x');
    ylabel('y');
    title(title_str);
    hold off;
end
