function [writer, frames_dir, use_video_writer] = make_video_writer_or_frames(filename, fps)
%MAKE_VIDEO_WRITER_OR_FRAMES Create VideoWriter when available, otherwise frame folder.
[path_str, base, ~] = fileparts(filename);
ensure_dir(path_str);
writer = [];
frames_dir = '';
use_video_writer = false;
try
    writer = VideoWriter(filename, 'MPEG-4');
    writer.FrameRate = fps;
    open(writer);
    use_video_writer = true;
catch
    frames_dir = fullfile(path_str, [base '_frames']);
    ensure_dir(frames_dir);
    use_video_writer = false;
end
end
