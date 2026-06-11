function plot_1d_consensus(time, X, title_str)
%PLOT_1D_CONSENSUS Plot all agent states over time.

    figure('visible', 'off');
    plot(time, X, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Agent states x_i(t)');
    title(title_str);
end
