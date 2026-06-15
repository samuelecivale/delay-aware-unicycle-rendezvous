function plot_dynamic_unicycle_states(time, states, commands, fig_dir)
%PLOT_DYNAMIC_UNICYCLE_STATES Save dynamic unicycle velocity/acceleration plots.
figure;
plot(time, squeeze(states(:,:,4)), 'LineWidth', 1.1);
grid on; xlabel('Time [s]'); ylabel('Linear velocity v_i'); title('Dynamic unicycle linear velocities');
save_current_figure(fullfile(fig_dir, 'fig_23c_dynamic_unicycle_linear_velocities.png'));
figure;
plot(time, squeeze(states(:,:,5)), 'LineWidth', 1.1);
grid on; xlabel('Time [s]'); ylabel('Angular velocity \omega_i'); title('Dynamic unicycle angular velocities');
save_current_figure(fullfile(fig_dir, 'fig_23d_dynamic_unicycle_angular_velocities.png'));
figure;
plot(time, squeeze(commands(:,:,1)), 'LineWidth', 1.1);
grid on; xlabel('Time [s]'); ylabel('Linear acceleration a_i'); title('Dynamic unicycle acceleration commands');
save_current_figure(fullfile(fig_dir, 'fig_23e_dynamic_unicycle_accelerations.png'));
figure;
plot(time, squeeze(commands(:,:,2)), 'LineWidth', 1.1);
grid on; xlabel('Time [s]'); ylabel('Angular acceleration \alpha_i'); title('Dynamic unicycle angular acceleration commands');
save_current_figure(fullfile(fig_dir, 'fig_23f_dynamic_unicycle_angular_accelerations.png'));
end
