function plot_disagreement_curves_variable_time(time_cell, curves_cell, labels, title_str)
%PLOT_DISAGREEMENT_CURVES_VARIABLE_TIME Plot curves with different time vectors.

    figure('visible', 'off');
    hold on;

    for i = 1:length(curves_cell)
        t = time_cell{i};
        d = curves_cell{i};
        semilogy(t, d(:) + 1e-14, 'LineWidth', 1.5);
    end

    grid on;
    xlabel('Time [s]');
    ylabel('Disagreement norm');
    title(title_str);
    legend(labels, 'Location', 'best');
    hold off;
end
