% A Gurobi optimization template, explaning key implementation components

%  minimize
%        x1 +   x2
%  subject to
%        nothing
%  x1 is continuous variable. x2 is integer varibale

names = {'x1'; 'x2';};

% Building the model
model.obj = [1 -1]; %coefficients for objective function
model.modelsense = 'min';%define optimizaiton type

%define variable type
%Choice 1: Define all varibale as a single type
%model.vtype = 'I'; 

%Choice 2: make a vector to define each varibale type
model.vtype = [repmat('C', 1, 1); repmat('B', 1, 1)];
%model.vtype = [repmat('B', nPlants, 1); repmat('C', nPlants * nWarehouses, 1)];

%Define linear constraints
model.A = sparse([0 0]);
model.rhs = [0];
model.sense = '=';

% Define varibale boundaries (lower bound and upper bound)
% For binary variables, setting lower oundary to 0 or -inf will not affect
% the result
% To define continuous variable in the entire real space, use -inf as the
% lower bound and upper bound is selected as inf by default
model.lb = [-1.4;0];

%Solve the problem
result = gurobi(model);


%Display result
disp([' '])
disp(['optimization result'])
disp(result.x)