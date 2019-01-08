% Half-Cheetah Motion Optimization based on Mixed-integer Optimization

%% Clear Environment
clear;
clc;

%% Parameter setting

m = 33; %kg
I = 2.9; %kg m^2
g = 9.8;

% Time parameters
TimeStep = 0.025; %in seconds
EndTime = 10; %in seconds
%missing checking the endtime is the nulriple of the time step
TimeSeries = 0:TimeStep:EndTime;
TimeSeriesLength = length(TimeSeries);

% Modeling/Define Variables
% States q = [x,z,theta,xdot,zdot,thetadot,PFx,PFz,PHx,PHz], the same length as TimeSeries
% x: horizontal position
% z: vertical position
% theta: orientation
% PFx: x position of front leg in ROBOT FRAME
% PFz: z position of front leg in ROBOT FRAME
% PHx: x position of hind leg in ROBOT FRAME
% PHz: z position of hind leg in ROBOT FRAME
%
% Initialize variable label containers
x_label = strings(1,TimeSeriesLength);
xdot_label = strings(1,TimeSeriesLength);

for i = 1:TimeSeriesLength
    x_label(i) = strcat('x',num2str(i-1));
    xdot_label(i) = strcat('xdot',num2str(i-1));
end

xLength = length(x_label);
xdotLength = length(x_label);

% Control Inputs
% u = [FFx,FFz,FHx,FHz]
% TimeSeriesLength-1
% FFx: Front leg tangential force
% FFz: Front leg normal force
% FHx: Hind leg tangential force
% FHz: Hind leg normal force

F_label = strings(1,TimeSeriesLength);

for i = 1:TimeSeriesLength
    F_label(i) = strcat('F',num2str(i-1));
end

FLength = length(F_label);

% Full Decision Variable name list
names = [x_label,xdot_label,F_label];

namesLength = length(names);

LengthList = [xLength,xdotLength,FLength];
     
% Setup Optimization Problem
% Objective Function
% minimize u^2
% coefficiant for the linear part of the objective function
model.obj = zeros(1,namesLength);

% Q matrix for the quadratic part of the objective function
% Quadratic form:
% v = [x,z,theta,xdot,zdot,thetadot,PFx,PFz,PHx,PHz, FFx,FFz,FHx,FHz,ModeFly,ModeDouble,ModeFront,ModeHind] ->length = 10+4+4 = 18
% inx: 1 2   3    4    5      6      7   8   9   10  11   12  13  14    15       16           17      18

Q = {};
for i = 1:3
    for j = 1:3
        if (i==3 && j==3) %|| (i == 1 && j == 1) || (i == 2 && j == 2)
            Q{i,j} = eye(LengthList(i),LengthList(j));
        else    
            Q{i,j} = zeros(LengthList(i),LengthList(j));
        end
    end
end

model.Q = sparse(cell2mat(Q));

%define optimization type
model.modelsense = 'min';
model.vtype = repmat('C',xLength+xdotLength+FLength,1);

model.lb = repmat(-inf,namesLength,1);

%Dynamic Constraints
% x(k+1)-x(k)-h*xdot(k) = 0
Ax = zeros(xLength-1,namesLength);
for i = 1:xLength-1
    Ax(i,find(names==strcat('x',num2str(i)))) = 1;
    Ax(i,find(names==strcat('x',num2str(i-1)))) = -1;
    Ax(i,find(names==strcat('xdot',num2str(i)))) = -TimeStep/2;
    Ax(i,find(names==strcat('xdot',num2str(i-1)))) = -TimeStep/2;
end
Ax_rhs = zeros(size(Ax,1),1);
Ax_sense = repmat('=',1,size(Ax,1));

Axdot = zeros(xdotLength-1,namesLength);
for i = 1:xdotLength-1
    Axdot(i,find(names==strcat('xdot',num2str(i))))=1;
    Axdot(i,find(names==strcat('xdot',num2str(i-1))))=-1;
    Axdot(i,find(names==strcat('F',num2str(i))))=-TimeStep/2/m;
    Axdot(i,find(names==strcat('F',num2str(i-1))))=-TimeStep/2/m;
end
Axdot_rhs = zeros(size(Axdot,1),1);
Axdot_sense = repmat('=',1,size(Axdot,1));

% Athetadot = zeros(zdotLength-1,namesLength);
% for i = 1:zdotLength-1
%     Athetadot(i,find(names==strcat('thetadot',num2str(i))))=1;
%     Athetadot(i,find(names==strcat('thetadot',num2str(i-1))))=-1;
%     Athetadot(i,find(names==strcat('FFz',num2str(i-1))))=-TimeStep/m;
%     Athetadot(i,find(names==strcat('FHz',num2str(i-1))))=-TimeStep/m;
% end
% Athetadot_rhs = -g*ones(size(Athetadot,1),1);
% Athetadot_sense = repmat('=',1,size(Athetadot,1));

Ainit = zeros(2,namesLength);
%Ainit = zeros(1,namesLength);
Ainit(1,find(names == x_label(1))) = 1;
Ainit(2,find(names == xdot_label(1))) = 1;
Ainit_rhs = [0;0];
Ainit_sense = ['=','='];
%Ainit_rhs = [0];
%Ainit_sense = ['='];

Aterminal = zeros(2,namesLength);
%Aterminal = zeros(1,namesLength);
Aterminal(1,find(names == x_label(end))) = 1;
Aterminal(2,find(names == xdot_label(end))) = 1;
Aterminal_rhs = [10;0];
Aterminal_sense = ['=','='];
%Aterminal_rhs = [10];
%Aterminal_sense = ['='];

% model.A = sparse([Ainit;Aterminal]);
% model.rhs = [Ainit_rhs;Aterminal_rhs];
% model.sense = [Ainit_sense,Aterminal_sense];

model.A = sparse([Ainit;Aterminal;Ax;Axdot]);
model.rhs = [Ainit_rhs;Aterminal_rhs;Ax_rhs;Axdot_rhs];
model.sense = [Ainit_sense,Aterminal_sense,Ax_sense,Axdot_sense];
%model.sense = ['>','<']


%model.start = [2*ones(1,length(x_label)),100*ones(1,length(u_label)),2*zeros(1,length(z_label))];
result = gurobi(model);


% Q matrix for the quadratic part of the objective function
% Quadratic form:
% v = [x, u ,z]'
% Q = [Q11 Q12 Q13;
%      Q21 Q22 Q23;
%      Q31 Q32 Q33]
% v'*Q*v = x'*Q11*x + x'*Q12*u + x'*Q13*z +
%          u'*Q21*x + u'*Q22*u + u'*Q23*z +
%          z'*Q31*x + z'*Q32*u + z'*Q33*z
%Q11 = zeros(length(x_label),length(x_label));
%Q12 = zeros(length(x_label),length(u_label));
%Q13 = zeros(length(x_label),length(z_label));
%Q21 = zeros(length(u_label),length(x_label));
%Q22 = eye(length(u_label),length(u_label));
%Q23 = zeros(length(u_label),length(z_label));
%Q31 = zeros(length(z_label),length(x_label));
%Q32 = zeros(length(z_label),length(u_label));
%Q33 = zeros(length(z_label),length(z_label));

%model.Q = sparse([Q11, Q12, Q13;
%                  Q21, Q22, Q23;
%                  Q31, Q32, Q33]);

%define optimization type
%model.modelsense = 'min';
%model.vtype = [repmat('C',length(x_label),1);repmat('C',length(u_label),1);repmat('B',length(z_label),1)];

%model.lb = repmat(-0.4,length(names),1);

%model.A = sparse(zeros(1,length(names)));
%model.rhs = 0;


%model.start = [2*ones(1,length(x_label)),100*ones(1,length(u_label)),2*zeros(1,length(z_label))];
%result = gurobi(model);

