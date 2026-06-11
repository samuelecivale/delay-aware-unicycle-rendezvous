function convert_all_frames_to_mp4(video_dir, fps)
%CONVERT_ALL_FRAMES_TO_MP4 Convert Octave frame folders to MP4 using ffmpeg.
%
% Usage:
%   convert_all_frames_to_mp4('p1_outputs_matlab/videos', 15)
%
% This is useful in GNU Octave, where VideoWriter is often unavailable.
% The animation functions save folders named *_frames containing:
%   frame_0001.png, frame_0002.png, ...

    if nargin < 1 || isempty(video_dir)
        video_dir = fullfile('p1_outputs_matlab', 'videos');
    end

    if nargin < 2
        fps = 15;
    end

    folders = dir(fullfile(video_dir, '*_frames'));

    if isempty(folders)
        fprintf('No *_frames folders found in %s\n', video_dir);
        return;
    end

    for k = 1:length(folders)
        frame_folder = fullfile(video_dir, folders(k).name);
        base_name = strrep(folders(k).name, '_frames', '');
        output_mp4 = fullfile(video_dir, [base_name '.mp4']);

        input_pattern = fullfile(frame_folder, 'frame_%04d.png');

        cmd = sprintf('ffmpeg -y -framerate %d -i "%s" -pix_fmt yuv420p "%s"', ...
            fps, input_pattern, output_mp4);

        fprintf('Running: %s\n', cmd);
        status = system(cmd);

        if status == 0
            fprintf('Created: %s\n', output_mp4);
        else
            fprintf('ffmpeg failed for folder: %s\n', frame_folder);
            fprintf('You can still use the PNG frames directly.\n');
        end
    end
end
