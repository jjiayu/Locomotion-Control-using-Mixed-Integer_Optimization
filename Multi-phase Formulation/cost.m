function cost = cost(vars,Q)
%                                     xIdx_end,...
%                                     xdotIdx_end,...
%                                     yIdx_end,...
%                                     ydotIdx_end,...
%                                     thetaIdx_end,...
%                                     thetadotIdx_end,...
%                                     x_end,...
%                                     xdot_end,...   
%                                     y_end,...
%                                     ydot_end,...
%                                     theta_end,...
%                                     thetadot_end)
%COST Quadratic cost function !!!!!May need quadrature as well
%   vars: decision variable vector, vars should be a column vector
%   Q: selection matrix for quadratic form

cost = vars'*Q*vars;
%        + (vars(xIdx_end) - x_end)' * (vars(xIdx_end) - x_end) ...
%        + (vars(xdotIdx_end) - xdot_end)' * (vars(xdotIdx_end) - xdot_end)...
%        + (vars(yIdx_end) - y_end)' * (vars(yIdx_end) - y_end) ...
%        + (vars(ydotIdx_end) - ydot_end)' * (vars(ydotIdx_end) - ydot_end)...
%        + (vars(thetaIdx_end) - theta_end)' * (vars(thetaIdx_end) - theta_end)...
%        + (vars(thetadotIdx_end) - thetadot_end)' * (vars(thetadotIdx_end) - thetadot_end);

end

