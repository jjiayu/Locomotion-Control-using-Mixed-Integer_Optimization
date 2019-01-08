%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


model.A = sparse([0 0 0]);
model.obj = [1 1 1];
model.modelsense = 'min';
model.rhs = [0];
model.sense = '=';
model.vtype = 'I';
result = gurobi(model);

disp([' '])
disp(['optimization result'])
disp(result)
