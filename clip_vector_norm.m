function u = clip_vector_norm(u, max_norm)
%CLIP_VECTOR_NORM Saturate a vector by Euclidean norm.
n = norm(u);
if n > max_norm && n > 1e-12
    u = u * (max_norm / n);
end
end
