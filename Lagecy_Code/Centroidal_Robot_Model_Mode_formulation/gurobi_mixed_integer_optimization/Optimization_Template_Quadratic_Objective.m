% A Gurobi optimization template, explaning key implementation components

%  minimize
%        x1^2
%  subject to
%        x >= 1 if z = 0
%        x <= 1 if z = 1
%  x is continuous variable. z is binary varibale

names = {'x1'};

% Building the model
model.obj = [0 0]; %coefficients for the (linear part) of the objective function
model.Q = sparse([1 0; 0 0]); %Q matrix for the (quadratic part) of the objective function
model.modelsense = 'min';%define optimizaiton type

%define variable type
%Choice 1: Define all varibale as a single type
%model.vtype = 'I'; 

%Choice 2: make a vector to define each varibale type
model.vtype = ['C'; 'B'];
%model.vtype = [repmat('C', 1, 1); repmat('B', 1, 1)];
%model.vtype = [repmat('B', nPlants, 1); repmat('C', nPlants * nWarehouses, 1)];

bigM = 10^3;
smallm = -10^3;

%Define linear constraints
model.A = sparse([1 -smallm; 1 bigM]);
model.rhs = [1; 1+bigM];
model.sense = ['>','<']; %can be =, >, <

%Define boundaries of decision variables
%put model.lb = -inf to define varibale in real-number space
model.lb = [-inf;-inf];

% Set initial guess
model.start = [2,0];
%model.StartNodeLimit = -200;

%Solve the problem
result = gurobi(model);


%Display result
disp([' '])
disp(['optimization result'])
disp(result.x)