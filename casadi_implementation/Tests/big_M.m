clear;
clc;

addpath('/home/jiayu/bin/casadi-linux-matlabR2014b-v3.4.5/')

import casadi.*

% Create NLP: Solve the Rosenbrock problem:
%     minimize    x^2
%     subject to  x<=1 if C = 0
%                 x>=1 if C = 1
%                 equivalent to -(1-c)M<=x-1<=cM --> 
%                               -(1-c)M +1 <= x <= cM + 1 -->
%                               -M + cM +1 <= x <= cM + 1 -->
%                               -M + 1 <= x - cM <= 1
% 
M = 1e5;
x = SX.sym('x');
c = SX.sym('c');
v = [x;c];
f = x^2 + c^2;
g = {};
lbg = [];
ubg = [];
g = {g{:},x + M*c};
lbg = [lbg;0];
ubg = [ubg;inf];
g = {g{:}, x - M*c};
lbg = [lbg;-inf];
ubg = [ubg;0];
discrete = [0,1];
nlp = struct('x', v, 'f', f, 'g', vertcat(g{:}));

% Create IPOPT solver object
solver = nlpsol('solver', 'knitro', nlp,struct('discrete', discrete));

% Solve the NLP
res = solver('x0' , [2.5 1.0],... % solution guess
             'lbx', [-100,0],...           % lower bound on x
             'ubx', [100,1],...           % upper bound on x
             'lbg',  lbg,...           % lower bound on g
             'ubg',  ubg);            % upper bound on g
 
% Print the solution
f_opt = full(res.f)          % >> 0
x_opt = full(res.x)          % >> [0; 1; 0]
lam_x_opt = full(res.lam_x)  % >> [0; 0; 0]
lam_g_opt = full(res.lam_g)  % >> 0