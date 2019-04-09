% Problem
f = -[6 5]';                %Objective Function (min f'x)
A = [1,4; 6,4; 2, -5];      %Linear Inequality Constraints (Ax <= b)
b = [16;28;6];    
lb = [0;0];                 %Bounds on x (lb <= x <= ub)
ub = [10;10];

% Create OPTI Object
opts = optiset('solver','SCIP');
Opt = opti('f',f,'ineq',A,b,'bounds',lb,ub,'options',opts)

[x,fval,exitflag,info] = solve(Opt)