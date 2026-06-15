function finish_video_writer_or_frames(writer, use_video_writer, frames_dir, filename)
%FINISH_VIDEO_WRITER_OR_FRAMES Close writer or report frame folder.
if use_video_writer
    try
        close(writer);
    catch
    end
else
    fprintf('Saved frames for %s in %s\n', filename, frames_dir);
end
end
