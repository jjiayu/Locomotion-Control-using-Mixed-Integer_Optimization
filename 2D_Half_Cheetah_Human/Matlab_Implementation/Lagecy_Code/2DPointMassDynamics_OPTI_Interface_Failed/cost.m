function cost = cost(vars, Q)
%COST Quadratic cost function !!!!!May need quadrature as well
%   vars: decision variable vector, vars should be a column vector
%   Q: selection matrix for quadratic form
cost = vars'*Q*vars;
%--------------------------------------------------------------------------
%   Test Code
%disp('cost is ')
%disp(cost)
end

