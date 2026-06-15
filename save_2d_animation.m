function save_2d_animation(P, time, filename, title_str, xlim_vals, ylim_vals, num_frames, show_edges)
%SAVE_2D_ANIMATION Save 2D consensus animation.
if nargin < 8, show_edges = false; end
if nargin < 7, num_frames = 100; end
P = double(P); time = time(:); N = size(P,2); K = size(P,1);
frames = frame_indices_for_animation(K, num_frames);
[writer, frames_dir, use_video_writer] = make_video_writer_or_frames(filename, 15);
fig = figure('Visible','off');
for f = 1:length(frames)
    k = frames(f);
    clf; hold on;
    for i = 1:N
        plot(P(1:k,i,1), P(1:k,i,2), 'LineWidth', 1.0);
    end
    if show_edges
        for i = 1:N
            for j = i+1:N
                plot([P(k,i,1) P(k,j,1)], [P(k,i,2) P(k,j,2)], ':', 'LineWidth', 0.5);
            end
        end
    end
    plot(P(k,:,1), P(k,:,2), 'o', 'MarkerSize', 7, 'LineWidth', 1.2);
    grid on; axis equal; xlim(xlim_vals); ylim(ylim_vals);
    xlabel('x'); ylabel('y'); title(sprintf('%s, t = %.1f s', title_str, time(k)));
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
