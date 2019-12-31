function Rslope = ElementaryRotation_Y(theta_slope)
%ELEMENTARYROTATION_Y Summary of this function goes here
%   Detailed explanation goes here

Rslope = [cos(theta_slope),    0,    sin(theta_slope);...
                   0,          1,            0;...
          -sin(theta_slope),   0,    cos(theta_slope)];

end

