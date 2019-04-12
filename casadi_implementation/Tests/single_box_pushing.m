% Single box Pushing example for CasADi

clear;
clc;

import casadi.*

T = 1; %time interval
N = 40; %number of knots
h = T/N;%time length of a time step

m = 1;

% Build variable vector

x = SX.sym('x',N+1);
xdot = SX.sym('xdot', N+1);

F = SX.sym('F',N);

%Build NLP

%initialize NLP
vars = {x,xdot,F};
vars = vertcat(vars{:});
vars_init = zeros(size(vars));
lbvars = -1e4*ones(size(vars));
ubvars = 1e4*ones(size(vars));
J = 0;
g = {};
lbg = [];
ubg = [];

for k = 1:N
    temp_pos = x(k+1) - x(k) - h*xdot(k);
    temp_vel = xdot(k+1) - xdot(k) -h*F(k)/m;
    
    g = {g{:},temp_pos,temp_vel};
    
    lbg = [lbg;0;0];
    ubg = [ubg;0;0];

    J = J + F(k).^2*h;
end

%reset initial and terminal condition

lbvars(1) = 0;
ubvars(1) = 0;
lbvars(N+2) = 0;
ubvars(N+2) = 0;

lbvars(N+1) = 5;
ubvars(N+1) = 5;

lbvars(N+1+N+1) = 0;
ubvars(N+1+N+1) = 0;



prob = struct('f',J, 'x', vars, 'g', vertcat(g{:}));
solver = nlpsol('solver','ipopt',prob);

sol = solver('x0',vars_init, 'lbx',lbvars ,'ubx', ubvars,...
             'lbg',lbg, 'ubg', ubg);
         
vars_opt = full(sol.x);


         


% %%
% 
% 
% 
% 
% 
% 
% %Declare model variables
% x_cur = SX.sym('x_cur');
% 
% x_next = SX.sym('x_next');
% 
% delta = SX.sym('delta');
% 
% EulerScheme = x_next - x_cur - h*delta;
% 
% %ODE
% dyn = [xdot_r; f/m];
% 
% %Objective Function
% L = f^2;
% 
% %build functions
% 
% Func = Function('func', {r,u}, {dyn,L});
% 
% %start with an empty NLP
% vars = {};
% vars_init = [];
% lbvars = [];
% ubvars = [];
% J = 0;
% g = {};
% lbg = [];
% ubg = [];
% 
% % "Lift" Initial Conditions
% 
% % Xk = MX.sym('X0',2);
% % 
% % vars = {vars{:}, Xk};
% % lbvars = {lbvars;-1e4;-1e4};
% % ubvars = {ubvars;1e4;1e4};
% % vars_init = [vars_init; 0; 0];
% 
% %Formulate the NLP
% for k = 0:N-1
%     
%     %Control input
%     Uk = MX.sym(['U_' num2str(k)]);
%     vars = {vars{:}, Uk};
%     lbvars = [lbvars; -1e4; -1e4];
%     ubvars = [ubvars; 1e4; 1e4];
%     vars_init = [vars_init;0];
%     
%     %State at collocation Points
%     
%     Xk = MX.sym(['X' num2str(k)],2);
%     vars = {vars{:}, Xk};
%     lbvars = [lbvars; -1e4; -1e4];
%     ubvars = [ubvrs; 1e4; 1e4];
%     vars_init = [vars_init; 0; 0];
%     
%     
%     
%     %Append Collocation euqations
%     [robotstate,robotvel]=Func(Xk,Uk);
%     g = {g{:},h*robotstate}
%     
% end