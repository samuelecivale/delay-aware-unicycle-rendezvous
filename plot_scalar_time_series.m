function plot_scalar_time_series(time, values, title_str, ylabel_str, label_str, reference, reference_label)
%PLOT_SCALAR_TIME_SERIES Plot scalar values versus time.
figure; hold on;
plot(time, values, 'LineWidth', 1.5);
if nargin >= 6 && ~isempty(reference)
    yline_custom(reference, '--', 1.2);
end
grid on;
xlabel('Time [s]');
ylabel(ylabel_str);
title(title_str);
if nargin >= 5 && ~isempty(label_str)
    if nargin >= 7 && ~isempty(reference_label)
        legend({label_str, reference_label}, 'Location', 'best');
    else
        legend({label_str}, 'Location', 'best');
    end
elseif nargin >= 7 && ~isempty(reference_label)
    legend({'value', reference_label}, 'Location', 'best');
end
hold off;
end

function yline_custom(y, style, lw)
xl = xlim;
plot(xl, [y y], style, 'LineWidth', lw);
xlim(xl);
end
