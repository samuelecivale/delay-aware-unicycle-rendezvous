function output_path = save_2d_animation(P, time, output_path, title_str, xlim_vals, ylim_vals, num_frames, show_centroid)
%SAVE_2D_ANIMATION Save 2D multi-agent animation.
%
% If VideoWriter is available, saves video.
% Otherwise, saves PNG frames in a folder.


    try
        make_videos = evalin('base', 'MAKE_VIDEOS');
    catch
        make_videos = true;
    end

    if ~make_videos
        fprintf('Skipping video generation: MAKE_VIDEOS=false.\n');
        return;
    end

    N = size(P, 2);
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
            x = squeeze(P(1:k, i, 1));
            y = squeeze(P(1:k, i, 2));

            plot(x, y, 'LineWidth', 1.2);
            plot(P(k, i, 1), P(k, i, 2), 'o', 'MarkerSize', 6);
        end

        if show_centroid
            P0 = squeeze(P(1, :, :));
            c = mean(P0, 1);
            plot(c(1), c(2), 'x', 'MarkerSize', 10, 'LineWidth', 1.5);
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
