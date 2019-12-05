function result = sinTaylor(angle)
%SINTAYLOR Summary of this function goes here
%   Detailed explanation goes here

result = angle - angle.^3/(3*2) + angle.^5/(5*4*3*2) - angle.^7/(7*6*5*4*3*2);

end

