function cost = objfunc(x)
%OBJFUNC Summary of this function goes here
%   Detailed explanation goes here

%cost = x'*x;
cost = 5*x(4) + 6*x(5) + 8*x(6) + 10*x(1) - 7*x(3) - 18*log(x(2)+1) - 19.2*log(x(1) - x(2) + 1) + 10;

end

