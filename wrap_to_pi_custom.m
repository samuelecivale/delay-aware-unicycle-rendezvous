function angle = wrap_to_pi_custom(angle)
%WRAP_TO_PI_CUSTOM Wrap angle to [-pi, pi].
    angle = mod(angle + pi, 2*pi) - pi;
end
