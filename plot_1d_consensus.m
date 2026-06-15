function plot_1d_consensus(time, X, title_str)
%PLOT_1D_CONSENSUS Plot agent states versus time.
figure;
plot(time, X, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('State x_i');
title(title_str);
end
