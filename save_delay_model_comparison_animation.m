function output_path = save_delay_model_comparison_animation(P_left, P_right, time, output_path, left_title, right_title, xlim_vals, ylim_vals, num_frames)
%SAVE_DELAY_MODEL_COMPARISON_ANIMATION Side-by-side animation.


    try
        make_videos = evalin('base', 'MAKE_VIDEOS');
    catch
        make_videos = true;
    end

    if ~make_videos
        fprintf('Skipping video generation: MAKE_VIDEOS=false.\n');
        return;
    end

    N = size(P_left, 2);
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

    fig = figure('Visible', 'off', 'Position', [100, 100, 1000, 450]);

    for f = 1:length(frame_indices)
        k = frame_indices(f);

        clf(fig);

        subplot(1,2,1);
        hold on;
        for i = 1:N
            plot(squeeze(P_left(1:k,i,1)), squeeze(P_left(1:k,i,2)), 'LineWidth', 1.1);
            plot(P_left(k,i,1), P_left(k,i,2), 'o', 'MarkerSize', 5);
        end
        grid on;
        axis equal;
        xlim(xlim_vals);
        ylim(ylim_vals);
        xlabel('x');
        ylabel('y');
        title(sprintf('%s, t = %.1f s', left_title, time(k)));
        hold off;

        subplot(1,2,2);
        hold on;
        for i = 1:N
            plot(squeeze(P_right(1:k,i,1)), squeeze(P_right(1:k,i,2)), 'LineWidth', 1.1);
            plot(P_right(k,i,1), P_right(k,i,2), 'o', 'MarkerSize', 5);
        end
        grid on;
        axis equal;
        xlim(xlim_vals);
        ylim(ylim_vals);
        xlabel('x');
        ylabel('y');
        title(sprintf('%s, t = %.1f s', right_title, time(k)));
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
