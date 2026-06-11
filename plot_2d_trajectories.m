function plot_2d_trajectories(P, title_str, target)
%PLOT_2D_TRAJECTORIES Plot planar trajectories.
%
% P is K x N x 2.

    K = size(P, 1);
    N = size(P, 2);

    figure('visible', 'off');
    hold on;

    for i = 1:N
        xi = squeeze(P(:, i, 1));
        yi = squeeze(P(:, i, 2));

        plot(xi, yi, 'LineWidth', 1.5);
        plot(xi(1), yi(1), 'o', 'MarkerSize', 6);
        plot(xi(end), yi(end), 'x', 'MarkerSize', 8, 'LineWidth', 1.5);
    end

    if ~isempty(target)
        plot(target(1), target(2), 's', 'MarkerSize', 9, 'LineWidth', 1.5);
    end

    grid on;
    axis equal;
    xlabel('x');
    ylabel('y');
    title(title_str);
    hold off;
end
