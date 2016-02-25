function alpha = pipi(beta)
alpha = beta;
if alpha <= -pi
    alpha = alpha + 2*pi;
elseif alpha >pi
    alpha = alpha - 2*pi;
end
end