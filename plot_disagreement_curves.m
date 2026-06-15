function plot_disagreement_curves(time, curves, labels, title_str)
%PLOT_DISAGREEMENT_CURVES Plot one or more disagreement curves versus time.
figure; hold on;
if isvector(curves)
    curves = curves(:);
end
for i = 1:size(curves,2)
    plot(time, curves(:,i), 'LineWidth', 1.4);
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
