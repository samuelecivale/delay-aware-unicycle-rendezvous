function frames = frame_indices_for_animation(K, num_frames)
%FRAME_INDICES_FOR_ANIMATION Return evenly spaced frame indices.
num_frames = min(num_frames, K);
frames = unique(round(linspace(1, K, num_frames)));
end
