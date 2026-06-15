function plot_unicycle_commands(time, commands)
%PLOT_UNICYCLE_COMMANDS Plot linear and angular velocity commands versus time.
figure;
subplot(2,1,1);
plot(time, squeeze(commands(:,:,1)), 'LineWidth', 1.1);
grid on; xlabel('Time [s]'); ylabel('Linear velocity v_i'); title('Linear velocity commands');
subplot(2,1,2);
plot(time, squeeze(commands(:,:,2)), 'LineWidth', 1.1);
grid on; xlabel('Time [s]'); ylabel('Angular velocity \omega_i'); title('Angular velocity commands');
end
