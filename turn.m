function theta = turn(Right,alpha,beta)
if Right
    theta = alpha+beta+(pi)/2;
else
    theta = -alpha+beta+(3*pi)/2;
end
end