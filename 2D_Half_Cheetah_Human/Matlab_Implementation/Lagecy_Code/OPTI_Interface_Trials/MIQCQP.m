% Objective
H = eye(2);                 %Objective Function (min 0.5x'Hx + f'x)
f = -[2 2]';                

% Constraints
A = [-1,1; 1,3];            %Linear Inequality Constraints (Ax <= b)
b = [2;5];    
lb = [0;0];                 %Bounds on x (lb <= x)

% Quadratic Constraint
Q = [1 0; 0 1];             %Quadratic Inequality (x'Qx + l'x <= r)
l = [0;-2];
r = 1;

% Integer Constraints
xtype = 'IC';

% Create OPTI Object
opts = optiset('solver','SCIP')
Opt = opti('qp',H,f,'ineq',A,b,'lb',lb,'qc',Q,l,r,'xtype',xtype,'options',opts)

% Solve the MIQCQP problem
[x,fval,exitflag,info] = solve(Opt)