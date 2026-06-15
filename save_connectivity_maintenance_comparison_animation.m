function save_connectivity_maintenance_comparison_animation(P_left, P_right, time, filename, left_title, right_title, radius, xlim_vals, ylim_vals, num_frames)
%SAVE_CONNECTIVITY_MAINTENANCE_COMPARISON_ANIMATION Side-by-side switching graph animation.
P_left = double(P_left); P_right = double(P_right); time = time(:);
N = size(P_left,2); K = min(size(P_left,1), size(P_right,1));
frames = frame_indices_for_animation(K, num_frames);
[writer, frames_dir, use_video_writer] = make_video_writer_or_frames(filename, 15);
fig = figure('Visible','off', 'Position', [100 100 1050 520]);
for f = 1:length(frames)
    k = frames(f); clf;
    for side = 1:2
        subplot(1,2,side); hold on;
        if side == 1, P = P_left; ttl = left_title; else, P = P_right; ttl = right_title; end
        for i = 1:N
            plot(P(1:k,i,1), P(1:k,i,2), 'LineWidth', 1.0);
        end
        A = adjacency_from_positions(squeeze(P(k,:,:)), radius, 1.0);
        for i = 1:N
            for j = i+1:N
                if A(i,j) > 0
                    plot([P(k,i,1) P(k,j,1)], [P(k,i,2) P(k,j,2)], ':', 'LineWidth', 0.6);
                end
            end
        end
        plot(P(k,:,1), P(k,:,2), 'o', 'MarkerSize', 7, 'LineWidth', 1.2);
        grid on; axis equal; xlim(xlim_vals); ylim(ylim_vals);
        xlabel('x'); ylabel('y'); title(sprintf('%s, t = %.1f s', ttl, time(k)));
    end
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
