T = 12; %time horizon
N = 40; %Number of control intervals

% Model parameters
c0 = 0.4;
c1 = 0.2;

% Initial condition for x
x0 = [0.5;0.7];

% Bounds on x
lbx = [0;0];
ubx = [2;2];

% Bounds on u
lbu = 0;
ubu = 1;

