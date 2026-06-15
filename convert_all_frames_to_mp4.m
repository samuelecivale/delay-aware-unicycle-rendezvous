function convert_all_frames_to_mp4(video_dir, fps)
%CONVERT_ALL_FRAMES_TO_MP4 Convert *_frames folders using ffmpeg, if available.
if nargin < 2, fps = 15; end
folders = dir(fullfile(video_dir, '*_frames'));
for i = 1:length(folders)
    frames_dir = fullfile(video_dir, folders(i).name);
    out_name = strrep(folders(i).name, '_frames', '.mp4');
    out_path = fullfile(video_dir, out_name);
    cmd = sprintf('ffmpeg -y -framerate %d -i "%s" "%s"', fps, fullfile(frames_dir, 'frame_%04d.png'), out_path);
    fprintf('Running: %s\n', cmd);
    system(cmd);
end
end
