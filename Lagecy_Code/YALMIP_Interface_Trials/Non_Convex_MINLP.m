% Define variables
x = sdpvar(2,1);

% Define constraints 
%Constraints = [sum(x) <= 10, x(1) == 0, 0.5 <= x(2) <= 1.5];
%for i = 1 : 7
%  Constraints = [Constraints, x(i) + x(i+1) <= x(i+2) + x(i+3)];
%end

Constraints = [];

% Define an objective
Objective = 20 + x(1)^2 + x(2)^2 - 10*(cos(2*pi*x(1)) + cos(2*pi*x(2)));

% Set some options for YALMIP and solver
options = sdpsettings('verbose',1,'solver','ipopt','quadprog.maxiter',100);

% Solve the problem
sol = optimize(Constraints,Objective,options);

% Analyze error flags
if sol.problem == 0
 % Extract and display value
 solution = value(x)
else
 display('Hmm, something went wrong!');
 sol.info
 yalmiperror(sol.problem)
end


% 
% % Objective
% fun = @(x) 20 + x(1)^2 + x(2)^2 - 10*(cos(2*pi*x(1)) + cos(2*pi*x(2)))
% 
% % Bounds
% lb = [5*pi;-20*pi];
% ub = [20*pi;-4*pi];
% 
% % Integer Constraints
% xtype = 'IC';
% 
% %Initial Guess
% x0 = [-5;5];      
% 
% % Options
% opts = optiset('solver','bonmin','display','iter')
% 
% % Create OPTI Object
% Opt = opti('fun',fun,'bounds',lb,ub,'xtype',xtype,'options',opts)
% 
% % Solve the MINLP problem
% [x,fval,exitflag,info] = solve(Opt,x0)    