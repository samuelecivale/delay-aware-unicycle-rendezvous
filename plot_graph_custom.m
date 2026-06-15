function plot_graph_custom(A, node_positions, title_str)
%PLOT_GRAPH_CUSTOM Simple graph drawing without toolboxes.
A = double(A);
N = size(A,1);
if nargin < 2 || isempty(node_positions)
    theta = linspace(0, 2*pi, N+1)'; theta(end) = [];
    node_positions = [cos(theta), sin(theta)];
end
if nargin < 3, title_str = 'Graph'; end
figure;
hold on;
for i = 1:N
    for j = i+1:N
        if A(i,j) > 0
            plot(node_positions([i,j],1), node_positions([i,j],2), '-', 'LineWidth', 1.2);
            if abs(A(i,j) - 1) > 1e-9
                mid = mean(node_positions([i,j],:),1);
                text(mid(1), mid(2), sprintf('%.2g', A(i,j)), 'FontSize', 8, 'HorizontalAlignment', 'center');
            end
        end
    end
end
plot(node_positions(:,1), node_positions(:,2), 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
for i = 1:N
    text(node_positions(i,1), node_positions(i,2), sprintf(' %d', i), 'FontSize', 9);
end
grid on; axis equal;
xlabel('x'); ylabel('y'); title(title_str);
hold off;
end
