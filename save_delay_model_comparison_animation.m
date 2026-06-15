function save_delay_model_comparison_animation(P_left, P_right, time, filename, left_title, right_title, xlim_vals, ylim_vals, num_frames)
%SAVE_DELAY_MODEL_COMPARISON_ANIMATION Side-by-side 2D trajectory comparison.
P_left = double(P_left); P_right = double(P_right); time = time(:);
N = size(P_left,2); K = min(size(P_left,1), size(P_right,1));
frames = frame_indices_for_animation(K, num_frames);
[writer, frames_dir, use_video_writer] = make_video_writer_or_frames(filename, 15);
fig = figure('Visible','off', 'Position', [100 100 1000 500]);
for f = 1:length(frames)
    k = frames(f);
    clf;
    subplot(1,2,1); hold on;
    for i = 1:N, plot(P_left(1:k,i,1), P_left(1:k,i,2), 'LineWidth', 1.0); end
    plot(P_left(k,:,1), P_left(k,:,2), 'o'); grid on; axis equal; xlim(xlim_vals); ylim(ylim_vals);
    xlabel('x'); ylabel('y'); title(sprintf('%s, t = %.1f s', left_title, time(k)));
    subplot(1,2,2); hold on;
    for i = 1:N, plot(P_right(1:k,i,1), P_right(1:k,i,2), 'LineWidth', 1.0); end
    plot(P_right(k,:,1), P_right(k,:,2), 'o'); grid on; axis equal; xlim(xlim_vals); ylim(ylim_vals);
    xlabel('x'); ylabel('y'); title(sprintf('%s, t = %.1f s', right_title, time(k)));
    drawnow;
    if use_video_writer
        writeVideo(writer, getframe(fig));
    else
        print(fig, fullfile(frames_dir, sprintf('frame_%04d.png', f)), '-dpng', '-r120');
    end
end
close(fig);
finish_video_writer_or_frames(writer, use_video_writer, frames_dir, filename);
end
