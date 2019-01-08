clear;
clc;

%cost function
func = @(x) x'*x;
A = [];
b = [];
Aeq = []; beq = [];
lb = [0;-100];
ub = [1;100];
xType = [2;0]; %Set Variable Type to be binary
objFnType = 0;
cFnType = [];
x0 = [10;10];
nlcon = @(x) constFunc(x);

%modify the solver call
x = knitromatlab_mip(func,x0,A,b,Aeq,beq,lb,ub,nlcon,xType,objFnType,cFnType);