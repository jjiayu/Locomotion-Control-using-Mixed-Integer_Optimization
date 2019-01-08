% Objective
H = [1 -1; -1  2];          %Objective Function (min 0.5x'Hx + f'x)
f = -[2 6]';                

% Constraints
A = [1,1; -1,2; 2, 1];      %Linear Inequality Constraints (Ax <= b)
b = [2;2;3];    
lb = [0;0];                 %Bounds on x (lb <= x)

% Integer Constraints
xtype = 'IC';

% Create OPTI Object
opts = optiset('solver','BONMIN')
Opt = opti('qp',H,f,'ineq',A,b,'lb',lb,'xtype',xtype,'options',opts)

% Solve the MIQP problem
[x,fval,exitflag,info] = solve(Opt)