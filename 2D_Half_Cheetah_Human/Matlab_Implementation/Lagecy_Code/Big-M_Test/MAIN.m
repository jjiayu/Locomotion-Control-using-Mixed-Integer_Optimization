% Test Big-M formulation using BONMIN SCIP

% Objective Function
% cost = x^2

clear;
clc;

objfunc = @(x) x(1)*x(1);

M = 1000;

A = [1,-M;
     1,0];

b = [0;0];

e = [-1;1];

vtype = 'CB';

v0 = [0;0];

lb = [-inf,-inf];
ub = [inf,inf];

opts = optiset('solver','bonmin','display','iter');

Opt = opti('fun',objfunc,'mix',A,b,e,'bounds',lb,ub,'xtype',vtype,'options',opts);

[results,fval,exitflag,info] = solve(Opt,v0)