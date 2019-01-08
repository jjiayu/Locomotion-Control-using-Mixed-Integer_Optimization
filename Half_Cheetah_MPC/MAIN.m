% Half-Cheetah Motion Optimization based on Mixed-integer Optimization
% Mode selection described as foot-ground contact changes

clear;
clc;

%==========================================================
%Inertia Parameters (Information from MIT Cheetah)
m = 33; %kg
I = 2.9; %kg m^2
g = 9.80665; %m/s^2
%==========================================================

%==========================================================
%Time parameters
TimeStep = 0.125; %in seconds
EndTime = TimeStep*20; %in seconds
% missing checking the endtime is the multiple of the time step
TimeSeries = 0:TimeStep:EndTime;
TimeSeriesLength = length(TimeSeries);
%=========================================================

%==========================================================
%Define/Create Modeling Variables
%----------------------------------------------------------
%   States: r = [x,y,theta,xdot,ydot,thetadot]
%       x: horizontal position
%       y: vertical position
%       theta: orientation
%       xdot: horizontal velocity
%       ydot: vertical velocity
%       thetadot: angular velocity
%----------------------------------------------------------
%   Initialize Variable Name Containers
x_label = strings(1,TimeSeriesLength);
y_label = strings(1,TimeSeriesLength);
theta_label = strings(1,TimeSeriesLength);
xdot_label = strings(1,TimeSeriesLength);
ydot_label = strings(1,TimeSeriesLength);
thetadot_label = strings(1,TimeSeriesLength); 
%   Assign Variable Names
for i = 1:TimeSeriesLength
    x_label(i) = strcat('x',num2str((i-1)));
    y_label(i) = strcat('y',num2str((i-1)));
    theta_label(i) = strcat('theta',num2str((i-1)));
    xdot_label(i) = strcat('xdot',num2str((i-1)));
    ydot_label(i) = strcat('ydot',num2str((i-1)));
    thetadot_label(i) = strcat('thetadot',num2str((i-1)));
end
%   Save List Length
xLength = 
yLength = 
thetaLength = 
xdotLength = 
ydotLength = 
thetadotLength = 
%----------------------------------------------------------
%   Footstep Locations(IN WORLD FRAME):
%       Front Leg: PF = [PFx, PFy]
%       Hind Leg:  PH = [PHx, PHy]
%----------------------------------------------------------
%   Initialize Variable Name Containers
PFx_label = strings(1,TimeSeriesLength);
PFy_label = strings(1,TimeSeriesLength);
PHx_label = strings(1,TimeSeriesLength);
PHy_label = strings(1,TimeSeriesLength);
%   Assign Variable Names
for i = 1:TimeSeriesLength
    PFx_label(i) = strcat('PFx',num2str((i-1)));
    PFy_label(i) = strcat('PFy',num2str((i-1)));
    PHx_label(i) = strcat('PHx',num2str((i-1)));
    PHy_label(i) = strcat('PHy',num2str((i-1)));
end
%----------------------------------------------------------
%   Foot-Ground Reaction Forces(IN WORLD FRAME):
%       Front Leg Forces: FF = [FFx, FFy]
%       Hind Leg Forces:  FH = [FHx, FHy]
%----------------------------------------------------------
%   Initialize Variable Name Containers
FFx_label = strings(1,TimeSeriesLength);
FFy_label = strings(1,TimeSeriesLength);
FHx_label = strings(1,TimeSeriesLength);
FHy_label = strings(1,TimeSeriesLength);
%   Assign Variable Names
for i = 1:TimeSeriesLength
    FFx_label(i) = strcat('FFx',num2str((i-1)));
    FFy_label(i) = strcat('FFy',num2str((i-1)));
    FHx_label(i) = strcat('FHx',num2str((i-1))); 
    FHy_label(i) = strcat('FHy',num2str((i-1)));
end
%----------------------------------------------------------
%   Mode Selection (Defined as the contact configuration for each leg):
%       Leg Contact Configuration: C = [CF, CH]; CF,CH = 0/1 (Binary Varibale)
%           Front Leg Contact On/Off: CF
%           Hind Leg Contact On/Off:  CH
%   Initialize Variable Name Containers
CF_label = strings(1,TimeSeriesLength);
CH_label = strings(1,TimeSeriesLength);
%   Assign Variable Names
for i = 1:TimeSeriesLength
    CF_label(i) = strcat('CF',num2str((i-1)));
    CH_label(i) = strcat('CH',num2str((i-1)));
end
%==========================================================



for i = 1:TimeSeriesLength
    x_label(i) = strcat('x',num2str(i-1));
    z_label(i) = strcat('z',num2str(i-1));
    theta_label(i) = strcat('theta',num2str(i-1));
    xdot_label(i) = strcat('xdot',num2str(i-1));
    zdot_label(i) = strcat('zdot',num2str(i-1));
    thetadot_label(i) = strcat('thetadot',num2str(i-1));
    PFx_label(i) = strcat('PFx',num2str(i-1));
    PFz_label(i) = strcat('PFz',num2str(i-1));
    PHx_label(i) = strcat('PHx',num2str(i-1));
    PHz_label(i) = strcat('PHz',num2str(i-1));
end

xLength = length(x_label);
zLength = length(z_label);
thetaLength = length(theta_label);
xdotLength = length(xdot_label);
zdotLength = length(zdot_label);
thetadotLength = length(thetadot_label);
PFxLength = length(PFx_label);
PFzLength = length(PFz_label);
PHxLength = length(PHx_label);
PHzLength = length(PHz_label);

% Control Inputs
% u = [FFx,FFz,FHx,FHz]
% TimeSeriesLength-1
% FFx: Front leg tangential force
% FFz: Front leg normal force
% FHx: Hind leg tangential force
% FHz: Hind leg normal force

FFx_label = strings(1,TimeSeriesLength);
FFz_label = strings(1,TimeSeriesLength);
FHx_label = strings(1,TimeSeriesLength);
FHz_label = strings(1,TimeSeriesLength);

for i = 1:TimeSeriesLength
    FFx_label(i) = strcat('FFx',num2str(i-1));
    FFz_label(i) = strcat('FFz',num2str(i-1));
    FHx_label(i) = strcat('FHx',num2str(i-1));
    FHz_label(i) = strcat('FHz',num2str(i-1));
end

FFxLength = length(FFx_label);
FFzLength = length(FFz_label);
FHxLength = length(FHx_label);
FHzLength = length(FHz_label);

% Mode Selection
ModeFly_label = strings(1,TimeSeriesLength);
ModeDouble_label = strings(1,TimeSeriesLength);
ModeFront_label = strings(1,TimeSeriesLength);
ModeHind_label = strings(1,TimeSeriesLength);

for i = 1:TimeSeriesLength
    ModeFly_label(i) = strcat('ModeFly',num2str(i-1));
    ModeDouble_label(i) = strcat('ModeDouble',num2str(i-1));
    ModeFront_label(i) = strcat('ModeFront',num2str(i-1));
    ModeHind_label(i) = strcat('ModeHind',num2str(i-1));
end

ModeFlyLength = length(ModeFly_label);
ModeDoubleLength = length(ModeDouble_label);
ModeFrontLength = length(ModeFront_label);
ModeHindLength = length(ModeHind_label);

% Full Decision Variable name list
names = [x_label,z_label,theta_label,xdot_label,zdot_label,thetadot_label,PFx_label,PFz_label,PHx_label,PHz_label,...
         FFx_label,FFz_label,FHx_label,FHz_label,...
         ModeFly_label,ModeDouble_label,ModeFront_label,ModeHind_label];

namesLength = length(names);

LengthList = [xLength,zLength,thetaLength,xdotLength,zdotLength,thetadotLength, PFxLength, PFzLength, PHxLength, PHzLength,...
              FFxLength,FFzLength,FHxLength,FHzLength,...
              ModeFlyLength,ModeDoubleLength,ModeFrontLength,ModeHindLength];
     
% Setup Optimization Problem
% Objective Function
% minimize u^2
% coefficiant for the linear part of the objective function
model.obj = zeros(1,namesLength);

% Q matrix for the quadratic part of the objective function
% Quadratic form:
% v = [x,z,theta,xdot,zdot,thetadot,PFx,PFz,PHx,PHz, FFx,FFz,FHx,FHz,ModeFly,ModeDouble,ModeFront,ModeHind] ->length = 10+4+4 = 18
% inx: 1 2   3    4    5      6      7   8   9   10  11   12  13  14    15       16           17      18
% v'*Qv = sigma(i,j) v1*Q11*v1+..v1*Q12*v2+..+v2*Q21*v1....

Q = {};
for i = 1:18
    for j = 1:18
        if (i == 11 && j == 11) || (i == 12 && j == 12) || (i == 13 && j == 13) || (i == 14 && j == 14)
            Q{i,j} = eye(LengthList(i),LengthList(j));
        else    
            Q{i,j} = zeros(LengthList(i),LengthList(j));
        end
    end
end

model.Q = sparse(cell2mat(Q));

%define optimization type
model.modelsense = 'min';
model.vtype = [repmat('C',xLength+zLength+thetaLength+xdotLength+zdotLength+thetadotLength+PFxLength+PFzLength+PHxLength+PHzLength+FFxLength+FFzLength+FHxLength+FHzLength,1);repmat('B',ModeFlyLength+ModeDoubleLength+ModeFrontLength+ModeHindLength,1)];

model.lb = repmat(-inf,namesLength,1);

%Dynamic Constraints
% see tropzoidal quadrature
Ax = zeros(xLength-1,namesLength);
for i = 1:xLength-1
    Ax(i,find(names==strcat('x',num2str(i)))) = 1;
    Ax(i,find(names==strcat('x',num2str(i-1)))) = -1;
    Ax(i,find(names==strcat('xdot',num2str(i)))) = -TimeStep/2;
    Ax(i,find(names==strcat('xdot',num2str(i-1)))) = -TimeStep/2;
end
Ax_rhs = zeros(size(Ax,1),1);
Ax_sense = repmat('=',1,size(Ax,1));

Az = zeros(zLength-1,namesLength);
for i = 1:zLength-1
    Az(i,find(names==strcat('z',num2str(i)))) = 1;
    Az(i,find(names==strcat('z',num2str(i-1)))) = -1;
    Az(i,find(names==strcat('zdot',num2str(i)))) = -TimeStep/2;
    Az(i,find(names==strcat('zdot',num2str(i-1)))) = -TimeStep/2;
end
Az_rhs = zeros(size(Az,1),1);
Az_sense = repmat('=',1,size(Az,1));

Atheta = zeros(thetaLength-1,namesLength);
for i = 1:thetaLength-1
    Atheta(i,find(names==strcat('theta',num2str(i)))) = 1;
    Atheta(i,find(names==strcat('theta',num2str(i-1)))) = -1;
    Atheta(i,find(names==strcat('thetadot',num2str(i)))) = -TimeStep/2;
    Atheta(i,find(names==strcat('thetadot',num2str(i-1)))) = -TimeStep/2;
end
Atheta_rhs = zeros(size(Atheta,1),1);
Atheta_sense = repmat('=',1,size(Atheta,1));

%Horizntal dynamics
%ModeFly
%Define Big-M
MFly = 10^5;

%Lower equality constraint
AxdotModeFlyLeq = zeros(xdotLength-1,namesLength); 
for i = 1:xdotLength-1
    AxdotModeFlyLeq(i,find(names==strcat('xdot',num2str(i))))=1;
    AxdotModeFlyLeq(i,find(names==strcat('xdot',num2str(i-1))))=-1;
    AxdotModeFlyLeq(i,find(names==strcat('FFx',num2str(i))))=0;
    AxdotModeFlyLeq(i,find(names==strcat('FHx',num2str(i))))=0;
    AxdotModeFlyLeq(i,find(names==strcat('FFx',num2str(i-1))))=0;
    AxdotModeFlyLeq(i,find(names==strcat('FHx',num2str(i-1))))=0;
    AxdotModeFlyLeq(i,find(names==strcat('ModeFly',num2str(i-1))))=MFly;
end
AxdotModeFlyLeq_rhs = MFly*ones(size(AxdotModeFlyLeq,1),1);
AxdotModeFlyLeq_sense = repmat('<',1,size(AxdotModeFlyLeq,1));

%Greater equality constraint
AxdotModeFlyGeq = zeros(xdotLength-1,namesLength);
for i = 1:xdotLength-1
    AxdotModeFlyGeq(i,find(names==strcat('xdot',num2str(i))))=1;
    AxdotModeFlyGeq(i,find(names==strcat('xdot',num2str(i-1))))=-1;
    AxdotModeFlyGeq(i,find(names==strcat('FFx',num2str(i))))=0;
    AxdotModeFlyGeq(i,find(names==strcat('FHx',num2str(i))))=0;
    AxdotModeFlyGeq(i,find(names==strcat('FFx',num2str(i-1))))=0;
    AxdotModeFlyGeq(i,find(names==strcat('FHx',num2str(i-1))))=0;
    AxdotModeFlyGeq(i,find(names==strcat('ModeFly',num2str(i-1))))=-MFly;
end
AxdotModeFlyGeq_rhs = -MFly*ones(size(AxdotModeFlyGeq,1),1);
AxdotModeFlyGeq_sense = repmat('>',1,size(AxdotModeFlyGeq,1));

%ModeDouble
%Define Big-M
MDouble = 10^5;

%Lower equality constraint
AxdotModeDoubleLeq = zeros(xdotLength-1,namesLength); 
for i = 1:xdotLength-1
    AxdotModeDoubleLeq(i,find(names==strcat('xdot',num2str(i))))=1;
    AxdotModeDoubleLeq(i,find(names==strcat('xdot',num2str(i-1))))=-1;
    AxdotModeDoubleLeq(i,find(names==strcat('FFx',num2str(i))))=-TimeStep/2/m;
    AxdotModeDoubleLeq(i,find(names==strcat('FHx',num2str(i))))=-TimeStep/2/m;
    AxdotModeDoubleLeq(i,find(names==strcat('FFx',num2str(i-1))))=-TimeStep/2/m;
    AxdotModeDoubleLeq(i,find(names==strcat('FHx',num2str(i-1))))=-TimeStep/2/m;
    AxdotModeDoubleLeq(i,find(names==strcat('ModeDouble',num2str(i-1))))=MDouble;
end
AxdotModeDoubleLeq_rhs = MDouble*ones(size(AxdotModeDoubleLeq,1),1);
AxdotModeDoubleLeq_sense = repmat('<',1,size(AxdotModeDoubleLeq,1));

%Greater equality constraint
AxdotModeDoubleGeq = zeros(xdotLength-1,namesLength);
for i = 1:xdotLength-1
    AxdotModeDoubleGeq(i,find(names==strcat('xdot',num2str(i))))=1;
    AxdotModeDoubleGeq(i,find(names==strcat('xdot',num2str(i-1))))=-1;
    AxdotModeDoubleGeq(i,find(names==strcat('FFx',num2str(i))))=-TimeStep/2/m;
    AxdotModeDoubleGeq(i,find(names==strcat('FHx',num2str(i))))=-TimeStep/2/m;
    AxdotModeDoubleGeq(i,find(names==strcat('FFx',num2str(i-1))))=-TimeStep/2/m;
    AxdotModeDoubleGeq(i,find(names==strcat('FHx',num2str(i-1))))=-TimeStep/2/m;
    AxdotModeDoubleGeq(i,find(names==strcat('ModeDouble',num2str(i-1))))=-MDouble;
end
AxdotModeDoubleGeq_rhs = -MDouble*ones(size(AxdotModeDoubleGeq,1),1);
AxdotModeDoubleGeq_sense = repmat('>',1,size(AxdotModeDoubleGeq,1));

%ModeFront
%Define Big-M
MFront = 10^5;

%Lower equality constraint
AxdotModeFrontLeq = zeros(xdotLength-1,namesLength); 
for i = 1:xdotLength-1
    AxdotModeFrontLeq(i,find(names==strcat('xdot',num2str(i))))=1;
    AxdotModeFrontLeq(i,find(names==strcat('xdot',num2str(i-1))))=-1;
    AxdotModeFrontLeq(i,find(names==strcat('FFx',num2str(i))))=-TimeStep/2/m;
    AxdotModeFrontLeq(i,find(names==strcat('FHx',num2str(i))))=0;
    AxdotModeFrontLeq(i,find(names==strcat('FFx',num2str(i-1))))=-TimeStep/2/m;
    AxdotModeFrontLeq(i,find(names==strcat('FHx',num2str(i-1))))=0;
    AxdotModeFrontLeq(i,find(names==strcat('ModeFront',num2str(i-1))))=MFront;
end
AxdotModeFrontLeq_rhs = MFront*ones(size(AxdotModeFrontLeq,1),1);
AxdotModeFrontLeq_sense = repmat('<',1,size(AxdotModeFrontLeq,1));

%Greater equality constraint
AxdotModeFrontGeq = zeros(xdotLength-1,namesLength);
for i = 1:xdotLength-1
    AxdotModeFrontGeq(i,find(names==strcat('xdot',num2str(i))))=1;
    AxdotModeFrontGeq(i,find(names==strcat('xdot',num2str(i-1))))=-1;
    AxdotModeFrontGeq(i,find(names==strcat('FFx',num2str(i))))=-TimeStep/2/m;
    AxdotModeFrontGeq(i,find(names==strcat('FHx',num2str(i))))=0;
    AxdotModeFrontGeq(i,find(names==strcat('FFx',num2str(i-1))))=-TimeStep/2/m;
    AxdotModeFrontGeq(i,find(names==strcat('FHx',num2str(i-1))))=0;
    AxdotModeFrontGeq(i,find(names==strcat('ModeFront',num2str(i-1))))=-MFront;
end
AxdotModeFrontGeq_rhs = -MFront*ones(size(AxdotModeFrontGeq,1),1);
AxdotModeFrontGeq_sense = repmat('>',1,size(AxdotModeFrontGeq,1));

%ModeHind
%Define Big-M
MHind = 10^5;

%Lower equality constraint
AxdotModeHindLeq = zeros(xdotLength-1,namesLength); 
for i = 1:xdotLength-1
    AxdotModeHindLeq(i,find(names==strcat('xdot',num2str(i))))=1;
    AxdotModeHindLeq(i,find(names==strcat('xdot',num2str(i-1))))=-1;
    AxdotModeHindLeq(i,find(names==strcat('FFx',num2str(i))))=0;
    AxdotModeHindLeq(i,find(names==strcat('FHx',num2str(i))))=-TimeStep/2/m;
    AxdotModeHindLeq(i,find(names==strcat('FFx',num2str(i-1))))=0;
    AxdotModeHindLeq(i,find(names==strcat('FHx',num2str(i-1))))=-TimeStep/2/m;
    AxdotModeHindLeq(i,find(names==strcat('ModeHind',num2str(i-1))))=MHind;
end
AxdotModeHindLeq_rhs = MHind*ones(size(AxdotModeHindLeq,1),1);
AxdotModeHindLeq_sense = repmat('<',1,size(AxdotModeHindLeq,1));

%Greater equality constraint
AxdotModeHindGeq = zeros(xdotLength-1,namesLength);
for i = 1:xdotLength-1
    AxdotModeHindGeq(i,find(names==strcat('xdot',num2str(i))))=1;
    AxdotModeHindGeq(i,find(names==strcat('xdot',num2str(i-1))))=-1;
    AxdotModeHindGeq(i,find(names==strcat('FFx',num2str(i))))=0;
    AxdotModeHindGeq(i,find(names==strcat('FHx',num2str(i))))=-TimeStep/2/m;
    AxdotModeHindGeq(i,find(names==strcat('FFx',num2str(i-1))))=0;
    AxdotModeHindGeq(i,find(names==strcat('FHx',num2str(i-1))))=-TimeStep/2/m;
    AxdotModeHindGeq(i,find(names==strcat('ModeHind',num2str(i-1))))=-MHind;
end
AxdotModeHindGeq_rhs = -MHind*ones(size(AxdotModeHindGeq,1),1);
AxdotModeHindGeq_sense = repmat('>',1,size(AxdotModeHindGeq,1));

Axdot = [AxdotModeFlyLeq;AxdotModeFlyGeq;AxdotModeDoubleLeq;AxdotModeDoubleGeq;AxdotModeFrontLeq;AxdotModeFrontGeq;AxdotModeHindLeq;AxdotModeHindGeq];
Axdot_rhs = [AxdotModeFlyLeq_rhs;AxdotModeFlyGeq_rhs;AxdotModeDoubleLeq_rhs;AxdotModeDoubleGeq_rhs;AxdotModeFrontLeq_rhs;AxdotModeFrontGeq_rhs;AxdotModeHindLeq_rhs;AxdotModeHindGeq_rhs];
Axdot_sense = [AxdotModeFlyLeq_sense,AxdotModeFlyGeq_sense,AxdotModeDoubleLeq_sense,AxdotModeDoubleGeq_sense,AxdotModeFrontLeq_sense,AxdotModeFrontGeq_sense,AxdotModeHindLeq_sense,AxdotModeHindGeq_sense];

%Axdot = zeros(xdotLength-1,namesLength);
%for i = 1:xdotLength-1
%    Axdot(i,find(names==strcat('xdot',num2str(i))))=1;
%    Axdot(i,find(names==strcat('xdot',num2str(i-1))))=-1;
%    Axdot(i,find(names==strcat('FFx',num2str(i))))=-TimeStep/2/m;
%    Axdot(i,find(names==strcat('FHx',num2str(i))))=-TimeStep/2/m;
%    Axdot(i,find(names==strcat('FFx',num2str(i-1))))=-TimeStep/2/m;
%    Axdot(i,find(names==strcat('FHx',num2str(i-1))))=-TimeStep/2/m;
%end
%Axdot_rhs = zeros(size(Axdot,1),1);
%Axdot_sense = repmat('=',1,size(Axdot,1));

%Vertical Dynamics
%ModeFly

%Lower equality constraint
AzdotModeFlyLeq = zeros(zdotLength-1,namesLength); 
for i = 1:zdotLength-1
    AzdotModeFlyLeq(i,find(names==strcat('zdot',num2str(i))))=1;
    AzdotModeFlyLeq(i,find(names==strcat('zdot',num2str(i-1))))=-1;
    AzdotModeFlyLeq(i,find(names==strcat('FFz',num2str(i))))=0;
    AzdotModeFlyLeq(i,find(names==strcat('FHz',num2str(i))))=0;
    AzdotModeFlyLeq(i,find(names==strcat('FFz',num2str(i-1))))=0;
    AzdotModeFlyLeq(i,find(names==strcat('FHz',num2str(i-1))))=0;
    AzdotModeFlyLeq(i,find(names==strcat('ModeFly',num2str(i-1))))=MFly;
end
AzdotModeFlyLeq_rhs = (-TimeStep*g+MFly)*ones(size(AzdotModeFlyLeq,1),1);
AzdotModeFlyLeq_sense = repmat('<',1,size(AzdotModeFlyLeq,1));

%Greater equality constraint
AzdotModeFlyGeq = zeros(zdotLength-1,namesLength);
for i = 1:zdotLength-1
    AzdotModeFlyGeq(i,find(names==strcat('zdot',num2str(i))))=1;
    AzdotModeFlyGeq(i,find(names==strcat('zdot',num2str(i-1))))=-1;
    AzdotModeFlyGeq(i,find(names==strcat('FFz',num2str(i))))=0;
    AzdotModeFlyGeq(i,find(names==strcat('FHz',num2str(i))))=0;
    AzdotModeFlyGeq(i,find(names==strcat('FFz',num2str(i-1))))=0;
    AzdotModeFlyGeq(i,find(names==strcat('FHz',num2str(i-1))))=0;
    AzdotModeFlyGeq(i,find(names==strcat('ModeFly',num2str(i-1))))=-MFly;
end
AzdotModeFlyGeq_rhs = (-TimeStep*g-MFly)*ones(size(AzdotModeFlyGeq,1),1);
AzdotModeFlyGeq_sense = repmat('>',1,size(AzdotModeFlyGeq,1));

%ModeDouble
%Define Big-M

%Lower equality constraint
AzdotModeDoubleLeq = zeros(zdotLength-1,namesLength); 
for i = 1:zdotLength-1
    AzdotModeDoubleLeq(i,find(names==strcat('zdot',num2str(i))))=1;
    AzdotModeDoubleLeq(i,find(names==strcat('zdot',num2str(i-1))))=-1;
    AzdotModeDoubleLeq(i,find(names==strcat('FFz',num2str(i))))=-TimeStep/2/m;
    AzdotModeDoubleLeq(i,find(names==strcat('FHz',num2str(i))))=-TimeStep/2/m;
    AzdotModeDoubleLeq(i,find(names==strcat('FFz',num2str(i-1))))=-TimeStep/2/m;
    AzdotModeDoubleLeq(i,find(names==strcat('FHz',num2str(i-1))))=-TimeStep/2/m;
    AzdotModeDoubleLeq(i,find(names==strcat('ModeDouble',num2str(i-1))))=MDouble;
end
AzdotModeDoubleLeq_rhs = (-TimeStep*g+MDouble)*ones(size(AzdotModeDoubleLeq,1),1);
AzdotModeDoubleLeq_sense = repmat('<',1,size(AzdotModeDoubleLeq,1));

%Greater equality constraint
AzdotModeDoubleGeq = zeros(zdotLength-1,namesLength);
for i = 1:zdotLength-1
    AzdotModeDoubleGeq(i,find(names==strcat('zdot',num2str(i))))=1;
    AzdotModeDoubleGeq(i,find(names==strcat('zdot',num2str(i-1))))=-1;
    AzdotModeDoubleGeq(i,find(names==strcat('FFz',num2str(i))))=-TimeStep/2/m;
    AzdotModeDoubleGeq(i,find(names==strcat('FHz',num2str(i))))=-TimeStep/2/m;
    AzdotModeDoubleGeq(i,find(names==strcat('FFz',num2str(i-1))))=-TimeStep/2/m;
    AzdotModeDoubleGeq(i,find(names==strcat('FHz',num2str(i-1))))=-TimeStep/2/m;
    AzdotModeDoubleGeq(i,find(names==strcat('ModeDouble',num2str(i-1))))=-MDouble;
end
AzdotModeDoubleGeq_rhs = (-TimeStep*g-MDouble)*ones(size(AzdotModeDoubleGeq,1),1);
AzdotModeDoubleGeq_sense = repmat('>',1,size(AzdotModeDoubleGeq,1));

%ModeFront
%Define Big-M

%Lower equality constraint
AzdotModeFrontLeq = zeros(zdotLength-1,namesLength); 
for i = 1:zdotLength-1
    AzdotModeFrontLeq(i,find(names==strcat('zdot',num2str(i))))=1;
    AzdotModeFrontLeq(i,find(names==strcat('zdot',num2str(i-1))))=-1;
    AzdotModeFrontLeq(i,find(names==strcat('FFz',num2str(i))))=-TimeStep/2/m;
    AzdotModeFrontLeq(i,find(names==strcat('FHz',num2str(i))))=0;
    AzdotModeFrontLeq(i,find(names==strcat('FFz',num2str(i-1))))=-TimeStep/2/m;
    AzdotModeFrontLeq(i,find(names==strcat('FHz',num2str(i-1))))=0;
    AzdotModeFrontLeq(i,find(names==strcat('ModeFront',num2str(i-1))))=MFront;
end
AzdotModeFrontLeq_rhs = (-TimeStep*g+MFront)*ones(size(AzdotModeFrontLeq,1),1);
AzdotModeFrontLeq_sense = repmat('<',1,size(AzdotModeFrontLeq,1));

%Greater equality constraint
AzdotModeFrontGeq = zeros(zdotLength-1,namesLength);
for i = 1:zdotLength-1
    AzdotModeFrontGeq(i,find(names==strcat('zdot',num2str(i))))=1;
    AzdotModeFrontGeq(i,find(names==strcat('zdot',num2str(i-1))))=-1;
    AzdotModeFrontGeq(i,find(names==strcat('FFz',num2str(i))))=-TimeStep/2/m;
    AzdotModeFrontGeq(i,find(names==strcat('FHz',num2str(i))))=0;
    AzdotModeFrontGeq(i,find(names==strcat('FFz',num2str(i-1))))=-TimeStep/2/m;
    AzdotModeFrontGeq(i,find(names==strcat('FHz',num2str(i-1))))=0;
    AzdotModeFrontGeq(i,find(names==strcat('ModeFront',num2str(i-1))))=-MFront;
end
AzdotModeFrontGeq_rhs = (-TimeStep*g-MFront)*ones(size(AzdotModeFrontGeq,1),1);
AzdotModeFrontGeq_sense = repmat('>',1,size(AzdotModeFrontGeq,1));

%ModeHind
%Define Big-M

%Lower equality constraint
AzdotModeHindLeq = zeros(zdotLength-1,namesLength); 
for i = 1:xdotLength-1
    AzdotModeHindLeq(i,find(names==strcat('zdot',num2str(i))))=1;
    AzdotModeHindLeq(i,find(names==strcat('zdot',num2str(i-1))))=-1;
    AzdotModeHindLeq(i,find(names==strcat('FFz',num2str(i))))=-TimeStep/2/m;
    AzdotModeHindLeq(i,find(names==strcat('FHz',num2str(i))))=0;
    AzdotModeHindLeq(i,find(names==strcat('FFz',num2str(i-1))))=-TimeStep/2/m;
    AzdotModeHindLeq(i,find(names==strcat('FHz',num2str(i-1))))=0;
    AzdotModeHindLeq(i,find(names==strcat('ModeHind',num2str(i-1))))=MHind;
end
AzdotModeHindLeq_rhs = (-TimeStep*g+MHind)*ones(size(AzdotModeHindLeq,1),1);
AzdotModeHindLeq_sense = repmat('<',1,size(AzdotModeHindLeq,1));

%Greater equality constraint
AzdotModeHindGeq = zeros(zdotLength-1,namesLength);
for i = 1:zdotLength-1
    AzdotModeHindGeq(i,find(names==strcat('zdot',num2str(i))))=1;
    AzdotModeHindGeq(i,find(names==strcat('zdot',num2str(i-1))))=-1;
    AzdotModeHindGeq(i,find(names==strcat('FFz',num2str(i))))=-TimeStep/2/m;
    AzdotModeHindGeq(i,find(names==strcat('FHz',num2str(i))))=0;
    AzdotModeHindGeq(i,find(names==strcat('FFz',num2str(i-1))))=-TimeStep/2/m;
    AzdotModeHindGeq(i,find(names==strcat('FHz',num2str(i-1))))=0;
    AzdotModeHindGeq(i,find(names==strcat('ModeHind',num2str(i-1))))=-MHind;
end
AzdotModeHindGeq_rhs = (-TimeStep*g-MHind)*ones(size(AzdotModeHindGeq,1),1);
AzdotModeHindGeq_sense = repmat('>',1,size(AzdotModeHindGeq,1));

Azdot = [AzdotModeFlyLeq;AzdotModeFlyGeq;AzdotModeDoubleLeq;AzdotModeDoubleGeq;AzdotModeFrontLeq;AzdotModeFrontGeq;AzdotModeHindLeq;AzdotModeHindGeq];
Azdot_rhs = [AzdotModeFlyLeq_rhs;AzdotModeFlyGeq_rhs;AzdotModeDoubleLeq_rhs;AzdotModeDoubleGeq_rhs;AzdotModeFrontLeq_rhs;AzdotModeFrontGeq_rhs;AzdotModeHindLeq_rhs;AzdotModeHindGeq_rhs];
Azdot_sense = [AzdotModeFlyLeq_sense,AzdotModeFlyGeq_sense,AzdotModeDoubleLeq_sense,AzdotModeDoubleGeq_sense,AzdotModeFrontLeq_sense,AzdotModeFrontGeq_sense,AzdotModeHindLeq_sense,AzdotModeHindGeq_sense];

% Azdot = zeros(zdotLength-1,namesLength);
% for i = 1:zdotLength-1
%    Azdot(i,find(names==strcat('zdot',num2str(i))))=1;
%    Azdot(i,find(names==strcat('zdot',num2str(i-1))))=-1;
%    Azdot(i,find(names==strcat('FFz',num2str(i))))=-TimeStep/2/m;
%    Azdot(i,find(names==strcat('FHz',num2str(i))))=-TimeStep/2/m;
%    Azdot(i,find(names==strcat('FFz',num2str(i-1))))=-TimeStep/2/m;
%    Azdot(i,find(names==strcat('FHz',num2str(i-1))))=-TimeStep/2/m;
% end
% Azdot_rhs = -TimeStep*g*ones(size(Azdot,1),1);
% Azdot_sense = repmat('=',1,size(Azdot,1));

% Athetadot = zeros(zdotLength-1,namesLength);
% for i = 1:zdotLength-1
%     Athetadot(i,find(names==strcat('thetadot',num2str(i))))=1;
%     Athetadot(i,find(names==strcat('thetadot',num2str(i-1))))=-1;
%     Athetadot(i,find(names==strcat('FFz',num2str(i-1))))=-TimeStep/m;
%     Athetadot(i,find(names==strcat('FHz',num2str(i-1))))=-TimeStep/m;
% end
% Athetadot_rhs = -g*ones(size(Athetadot,1),1);
% Athetadot_sense = repmat('=',1,size(Athetadot,1));

Ainit = zeros(6,namesLength);
%Ainit = zeros(3,namesLength);
Ainit(1,find(names == x_label(1))) = 1;
Ainit(2,find(names == z_label(1))) = 1;
Ainit(3,find(names == theta_label(1))) = 1;
Ainit(4,find(names == xdot_label(1))) = 1;
Ainit(5,find(names == zdot_label(1))) = 1;
Ainit(6,find(names == thetadot_label(1))) = 1;
Ainit_rhs = [0;0;0;0;0;0];
Ainit_sense = ['=','=','=','=','=','='];
%Ainit_rhs = [0;0;0];
%Ainit_sense = ['=','=','='];

Aterminal = zeros(6,namesLength);
%Aterminal = zeros(3,namesLength);
Aterminal(1,find(names == x_label(end))) = 1;
Aterminal(2,find(names == z_label(end))) = 1;
Aterminal(3,find(names == theta_label(end))) = 1;
Aterminal(4,find(names == xdot_label(end))) = 1;
Aterminal(5,find(names == zdot_label(end))) = 1;
Aterminal(6,find(names == thetadot_label(end))) = 1;
Aterminal_rhs = [1;1;0;0;0;0];
Aterminal_sense = ['=','=','=','=','=','='];
%Aterminal_rhs = [10;0;0];
%Aterminal_sense = ['=','=','='];

% Mode Selection constraint
AModeSelection = zeros(length(TimeSeries),namesLength);
for i = 1:length(TimeSeries)
     AModeSelection(i,find(names == strcat('ModeFly',num2str(i-1)))) = 1;
     AModeSelection(i,find(names == strcat('ModeDouble',num2str(i-1)))) = 1;
     AModeSelection(i,find(names == strcat('ModeFront',num2str(i-1)))) = 1;
     AModeSelection(i,find(names == strcat('ModeHind',num2str(i-1)))) = 1;
end
AModeSelection_rhs = ones(length(TimeSeries),1);
AModeSelection_sense = repmat('=',1,size(AModeSelection,1));

% model.A = sparse([Ainit;Aterminal]);
% model.rhs = [Ainit_rhs;Aterminal_rhs];
% model.sense = [Ainit_sense,Aterminal_sense];

model.A = sparse([Ainit;Aterminal;AModeSelection;Ax;Az;Atheta;Axdot;Azdot]);
model.rhs = [Ainit_rhs;Aterminal_rhs;AModeSelection_rhs;Ax_rhs;Az_rhs;Atheta_rhs;Axdot_rhs;Azdot_rhs];
model.sense = [Ainit_sense,Aterminal_sense,AModeSelection_sense,Ax_sense,Az_sense,Atheta_sense,Axdot_sense,Azdot_sense];
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

