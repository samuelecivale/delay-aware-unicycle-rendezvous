function plot_graph_custom(A, node_positions, title_str)
%PLOT_GRAPH_CUSTOM Plot an undirected graph without external toolboxes.

    N = size(A, 1);

    if isempty(node_positions)
        angles = linspace(0, 2*pi, N+1);
        angles = angles(1:N);
        node_positions = [cos(angles(:)), sin(angles(:))];
    end

    figure('visible', 'off');
    hold on;

    for i = 1:N
        for j = (i+1):N
            if A(i, j) > 0
                plot([node_positions(i,1), node_positions(j,1)], ...
                     [node_positions(i,2), node_positions(j,2)], ...
                     'LineWidth', 1.2);
            end
        end
    end

    scatter(node_positions(:,1), node_positions(:,2), 120, 'filled');

    for i = 1:N
        text(node_positions(i,1), node_positions(i,2), sprintf('%d', i), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'Color', 'w', ...
            'FontWeight', 'bold');
    end

    grid on;
    axis equal;
    xlabel('x');
    ylabel('y');
    title(title_str);
    hold off;
end
