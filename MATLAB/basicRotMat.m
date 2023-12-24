function R = basicRotMat(axis, theta)
if axis == "x"
    R = [1, 0, 0;
         0, cos(theta), -sin(theta);
         0, sin(theta), cos(theta)];
elseif axis == "y"
    R = [cos(theta), 0, sin(theta);
         0, 1, 0;
         -sin(theta), 0, cos(theta)];
elseif axis == "z"
    R = [cos(theta), -sin(theta), 0;
         sin(theta), cos(theta), 0;
         0, 0, 1];
end
end