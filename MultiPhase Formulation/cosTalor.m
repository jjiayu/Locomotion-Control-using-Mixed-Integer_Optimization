function result = cosTalor(angle)
%COSTALOR Summary of this function goes here
%   Detailed explanation goes here

result = 1 - angle.^2/2 + angle.^4/(4*3*2);

end

