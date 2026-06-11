function plot_disagreement_curves(time, curves, labels, title_str)
%PLOT_DISAGREEMENT_CURVES Semilog plot of one or more disagreement curves.
%
% curves can be K x M or K x 1.

    figure('visible', 'off');
    hold on;

    if isvector(curves)
        curves = curves(:);
    end

    M = size(curves, 2);

    for i = 1:M
        semilogy(time, curves(:, i) + 1e-14, 'LineWidth', 1.5);
    end

    grid on;
    xlabel('Time [s]');
    ylabel('Disagreement norm');
    title(title_str);
    legend(labels, 'Location', 'best');
    hold off;
end
