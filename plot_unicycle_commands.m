function plot_unicycle_commands(time, commands)
%PLOT_UNICYCLE_COMMANDS Plot v_i and omega_i commands.

    figure('visible', 'off');

    subplot(2,1,1);
    plot(time, squeeze(commands(:,:,1)), 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('v_i');
    title('Unicycle linear velocity commands');

    subplot(2,1,2);
    plot(time, squeeze(commands(:,:,2)), 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('\omega_i');
    title('Unicycle angular velocity commands');
end
