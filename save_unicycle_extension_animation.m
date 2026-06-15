function save_unicycle_extension_animation(states, time, filename, title_str, xlim_vals, ylim_vals, num_frames, fixed_A, dynamic_radius, obstacles)
%SAVE_UNICYCLE_EXTENSION_ANIMATION Animate unicycle states with optional graph/obstacles.
if nargin < 10, obstacles = []; end
if nargin < 9, dynamic_radius = []; end
if nargin < 8, fixed_A = []; end
states = double(states); time = time(:); N = size(states,2); K = size(states,1);
P = states(:,:,1:2);
frames = frame_indices_for_animation(K, num_frames);
[writer, frames_dir, use_video_writer] = make_video_writer_or_frames(filename, 15);
fig = figure('Visible','off');
for f = 1:length(frames)
    k = frames(f);
    clf; hold on;
    if ~isempty(obstacles)
        th = linspace(0, 2*pi, 160);
        for o = 1:length(obstacles)
            c = obstacles(o).center(:)'; r = obstacles(o).radius;
            plot(c(1) + r*cos(th), c(2) + r*sin(th), 'LineWidth', 1.5);
            plot(c(1), c(2), 'x');
        end
    end
    for i = 1:N
        plot(P(1:k,i,1), P(1:k,i,2), 'LineWidth', 1.0);
    end
    if ~isempty(fixed_A)
        A = fixed_A;
    elseif ~isempty(dynamic_radius)
        A = adjacency_from_positions(squeeze(P(k,:,:)), dynamic_radius, 1.0);
    else
        A = [];
    end
    if ~isempty(A)
        for i = 1:N
            for j = i+1:N
                if A(i,j) > 0
                    plot([P(k,i,1) P(k,j,1)], [P(k,i,2) P(k,j,2)], ':', 'LineWidth', 0.6);
                end
            end
        end
    end
    plot(P(k,:,1), P(k,:,2), 'o', 'MarkerSize', 7, 'LineWidth', 1.2);
    for i = 1:N
        x = states(k,i,1); y = states(k,i,2); th = states(k,i,3);
        quiver(x, y, 0.35*cos(th), 0.35*sin(th), 0, 'LineWidth', 1.1);
    end
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
