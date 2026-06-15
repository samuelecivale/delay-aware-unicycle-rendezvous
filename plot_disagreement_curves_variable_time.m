function plot_disagreement_curves_variable_time(time_cell, curves_cell, labels, title_str)
%PLOT_DISAGREEMENT_CURVES_VARIABLE_TIME Plot curves with different time grids.
figure; hold on;
for i = 1:length(time_cell)
    plot(time_cell{i}, curves_cell{i}, 'LineWidth', 1.4);
end
grid on;
xlabel('Time [s]');
ylabel('Disagreement norm');
title(title_str);
if nargin >= 3 && ~isempty(labels)
    legend(labels, 'Location', 'best');
end
hold off;
end
