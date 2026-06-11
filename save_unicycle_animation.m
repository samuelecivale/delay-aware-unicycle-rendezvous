function output_path = save_unicycle_animation(states, time, output_path, title_str, xlim_vals, ylim_vals, num_frames)
%SAVE_UNICYCLE_ANIMATION Save unicycle animation with heading segments.


    try
        make_videos = evalin('base', 'MAKE_VIDEOS');
    catch
        make_videos = true;
    end

    if ~make_videos
        fprintf('Skipping video generation: MAKE_VIDEOS=false.\n');
        return;
    end

    N = size(states, 2);
    K = length(time);
    frame_indices = round(linspace(1, K, num_frames));

    [folder, name, ext] = fileparts(output_path);
    ensure_dir(folder);

    use_video = (exist('VideoWriter', 'class') == 8) || (exist('VideoWriter', 'file') == 2);

    if use_video
        try
            writer = VideoWriter(output_path, 'MPEG-4');
        catch
            output_path = fullfile(folder, [name '.avi']);
            writer = VideoWriter(output_path, 'Motion JPEG AVI');
        end
        writer.FrameRate = 15;
        open(writer);
    else
        frame_folder = fullfile(folder, [name '_frames']);
        ensure_dir(frame_folder);
    end

    fig = figure('Visible', 'off');

    for f = 1:length(frame_indices)
        k = frame_indices(f);

        clf(fig);
        hold on;

        for i = 1:N
            x = squeeze(states(1:k, i, 1));
            y = squeeze(states(1:k, i, 2));

            plot(x, y, 'LineWidth', 1.2);
            plot(states(k, i, 1), states(k, i, 2), 'o', 'MarkerSize', 6);

            theta = states(k, i, 3);
            len = 0.35;
            plot([states(k, i, 1), states(k, i, 1) + len*cos(theta)], ...
                 [states(k, i, 2), states(k, i, 2) + len*sin(theta)], ...
                 'LineWidth', 1.5);
        end

        grid on;
        axis equal;
        xlim(xlim_vals);
        ylim(ylim_vals);
        xlabel('x');
        ylabel('y');
        title(sprintf('%s, t = %.1f s', title_str, time(k)));
        hold off;
        drawnow;

        frame = getframe(fig);

        if use_video
            writeVideo(writer, frame);
        else
            imwrite(frame.cdata, fullfile(frame_folder, sprintf('frame_%04d.png', f)));
        end
    end

    if use_video
        close(writer);
        fprintf('Saved video: %s\n', output_path);
    else
        fprintf('VideoWriter unavailable. Saved frames in: %s\n', frame_folder);
        output_path = frame_folder;
    end

    close(fig);
end
