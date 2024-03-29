% Mixed-Integer Nonlinear Optimization in 2D Case (Planar Cheetah)
% Mode Selection described as foot-ground contact changes 0/1

clear;
clc;

%==========================================================
%Add solvers' Path
%Knitro
addpath('C:\Program Files\Artelys\Knitro 11.1.0\knitromatlab')
%Gurobi
addpath('C:\gurobi810')
%==========================================================
%Choose Solver
solver = "knitro";
%solver = "gurobi";

%==========================================================
%Inertia Parameters(Information from MIT Cheetah 3)
m = 45; %kg
I = 2.1; %kg m^2
g = 9.80665; %m/s^2
%==========================================================

%==========================================================
%Time parameters
NumTimeSteps = 20; %number of time steps
h = 0.05; %Time Step in seconds
EndTime = h*NumTimeSteps; %in seconds
% missing checking the endtime is the multiple of the time step
TimeSeries = 0:h:EndTime;
TimeSeriesLength = length(TimeSeries);
%=========================================================

%=========================================================
%Initial Conditions
x_init = 0;
y_init = 0.3; %0.2
xdot_init = 0;
ydot_init = 0;
%---------------------------------------------------------
%Terminal Conditions
x_end = 0.7; %20
y_end = 0.3; %0.2
xdot_end = 0;
ydot_end = 0;
%=========================================================

%==========================================================
%Define/Create Modeling Variables
%----------------------------------------------------------
%   States: r = [x,y,theta,xdot,ydot,thetadot]
%       x: horizontal position
%       y: vertical position
%       xdot: horizontal velocity
%       ydot: vertical velocity
%----------------------------------------------------------
%   Initialize Variable Name Containers
x_label = strings(1,TimeSeriesLength);
y_label = strings(1,TimeSeriesLength);
xdot_label = strings(1,TimeSeriesLength);
ydot_label = strings(1,TimeSeriesLength);
%   Assign Variable Names
for i = 1:TimeSeriesLength
    x_label(i) = strcat('x',num2str((i-1)));
    y_label(i) = strcat('y',num2str((i-1)));
    xdot_label(i) = strcat('xdot',num2str((i-1)));
    ydot_label(i) = strcat('ydot',num2str((i-1)));
end
%   Save List Length
xLength = length(x_label);
yLength = length(y_label);
xdotLength = length(xdot_label);
ydotLength = length(ydot_label);
%----------------------------------------------------------
%   Footstep Locations(IN WORLD FRAME):
%       Front Leg: PF = [PFx, PFy, PFxdot, PFydot]
%       Hind Leg:  PH = [PHx, PHy, PHxdot, PHydot]
%----------------------------------------------------------
%   Initialize Variable Name Containers
PFx_label = strings(1,TimeSeriesLength);
PFy_label = strings(1,TimeSeriesLength);
PFxdot_label = strings(1,TimeSeriesLength);
PFydot_label = strings(1,TimeSeriesLength);
PHx_label = strings(1,TimeSeriesLength);
PHy_label = strings(1,TimeSeriesLength);
PHxdot_label = strings(1,TimeSeriesLength);
PHydot_label = strings(1,TimeSeriesLength);
%   Assign Variable Names
for i = 1:TimeSeriesLength
    PFx_label(i) = strcat('PFx',num2str((i-1)));
    PFy_label(i) = strcat('PFy',num2str((i-1)));
    PFxdot_label(i) = strcat('PFxdot',num2str(i-1));
    PFydot_label(i) = strcat('PFydot',num2str(i-1));
    PHx_label(i) = strcat('PHx',num2str((i-1)));
    PHy_label(i) = strcat('PHy',num2str((i-1)));
    PHxdot_label(i) = strcat('PHxdot',num2str(i-1));
    PHydot_label(i) = strcat('PHydot',num2str(i-1));
end
%   Save List Length
PFxLength = length(PFx_label);
PFyLength = length(PFy_label);
PFxdotLength = length(PFxdot_label);
PFydotLength = length(PFydot_label);
PHxLength = length(PHx_label);
PHyLength = length(PHy_label);
PHxdotLength = length(PHxdot_label);
PHydotLength = length(PHydot_label);
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
%   Save List Length
FFxLength = length(FFx_label);
FFyLength = length(FFy_label);
FHxLength = length(FHx_label);
FHyLength = length(FHy_label);
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
%   Save List Length
CFLength = length(CF_label);
CHLength = length(CH_label);
%----------------------------------------------------------
%   Full Decision Variable Name List
names = [x_label, y_label, xdot_label, ydot_label, PFx_label, PFy_label, PFxdot_label, PFydot_label, PHx_label, PHy_label, PHxdot_label, PHydot_label, FFx_label, FFy_label, FHx_label, FHy_label, CF_label, CH_label];
%   Length of the name list
namesLength = length(names);
%   Length list of all variable list
LengthList = [xLength, yLength, xdotLength, ydotLength, PFxLength, PFyLength, PFxdotLength, PFydotLength, PHxLength, PHyLength, PHxdotLength, PHydotLength, FFxLength, FFyLength, FHxLength, FHyLength, CFLength, CHLength];
%----------------------------------------------------------
%   List for all decision variable names
varList = ["x", "y", "xdot", "ydot", "PFx", "PFy", "PFxdot", "PFydot", "PHx", "PHy", "PHxdot", "PHydot", "FFx", "FFy", "FHx", "FHy", "CF", "CH"];
%   Length of the decision varibale name list
varListLength = length(varList);
%==========================================================

%==========================================================
%Setup Optimization Problem
%----------------------------------------------------------
%   Objective Function
%       minimize F^2 -> FFx(t)^2 + FFy(t)^2 + FHx(t)^2 + FHy(t)^2
%   Quadratic form:
%       minimize F'*Q*F
%   v = [x;   y;   xdot;   ydot;   PFx;   PFy;  PFxdot;  PFydot;  PHx;  PHy;  PHxdot;  PHydot;   FFx;    FFy;    FHx;    FHy;   CF;   CH]
%   idx: 1    2     3        4      5      6      7         8      9     10     11       12       13      14      15     16     17    18
%   v'*Q*v = sigma(i,j) v1*Q11*v1 + v1*Q12*v2 + .... + v2*Q21*v1 + v2*Q22*v2 ...
%-----------------------------------------------------------
%   Define Q
QCell = {};
%   Assign Q Matrix, True Code need to be activated when running optimization
for i = 1:varListLength
    for j = 1:varListLength
        if (varList(i) == "FFx" && varList{j} == "FFx") || (varList{i} == "FFy" && varList{j} == "FFy") || (varList{i} == "FHx" && varList{j} == "FHx") || (varList{i} == "FHy" && varList{j} == "FHy")
            QCell{i,j} = eye(LengthList(i),LengthList(j));
%         elseif varList(i) == "CF" && varList(j) == "CF"
%             QCell{i,j} = 10000*eye(LengthList(i),LengthList(j));
%         elseif varList(i) == "CH" && varList(j) == "CH"
%             QCell{i,j} = 50000*eye(LengthList(i),LengthList(j));
        else
            QCell{i,j} = zeros(LengthList(i),LengthList(j));
        end
    end
end
%
Q = cell2mat(QCell);

% Q(find(names == 'y15'),find(names == 'y15')) = 1e30;
% Q(find(names == 'y14'),find(names == 'y14')) = 1e30;
% Q(find(names == 'y13'),find(names == 'y13')) = 1e30;
% Q(find(names == 'y16'),find(names == 'y16')) = 1e30;
% Q(find(names == 'y17'),find(names == 'y17')) = 1e30;
% Q(find(names == 'y18'),find(names == 'y18')) = 1e30;

Q_backup = Q;
Q = sparse(Q);
%------------------------------------------------------------
%   Add Constraints
%       System Dynamics (equality constraints)
%-------------------------------------------------------------
%           x-axis position dynamics
%               Build A matrix
Ax_pos_dyn = zeros(xLength-1,namesLength); 
for k = 0:TimeSeriesLength-2 %NOTE: minus 2, 0 to (length -1) -1
    Ax_pos_dyn(k+1,find(names == strcat('x',num2str(k+1)))) = 1;
    Ax_pos_dyn(k+1,find(names == strcat('x',num2str(k))))   = -1;
    %Ax_pos_dyn(k+1,find(names == strcat('xdot',num2str(k+1))))= -1/2*h;
    %Ax_pos_dyn(k+1,find(names == strcat('xdot',num2str(k))))= -1/2*h;
    Ax_pos_dyn(k+1,find(names == strcat('xdot',num2str(k))))= -h; %Euler Integration
end
%               Build b vector
bx_pos_dyn = zeros(size(Ax_pos_dyn,1),1);
%--------------------------------------------------------------
%           x-axis velocity dynamics
%               Build A matrix
Ax_vel_dyn = zeros(xdotLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    Ax_vel_dyn(k+1,find(names == strcat('xdot',num2str(k+1)))) = 1;
    Ax_vel_dyn(k+1,find(names == strcat('xdot',num2str(k))))   = -1;
%     Ax_vel_dyn(k+1,find(names == strcat('FFx',num2str(k+1))))  = -1/2*h/m;
%     Ax_vel_dyn(k+1,find(names == strcat('FHx',num2str(k+1))))  = -1/2*h/m;
%     Ax_vel_dyn(k+1,find(names == strcat('FFx',num2str(k))))  = -1/2*h/m;
%     Ax_vel_dyn(k+1,find(names == strcat('FHx',num2str(k))))  = -1/2*h/m;
    Ax_vel_dyn(k+1,find(names == strcat('FFx',num2str(k))))  = -h/m;%Euler Integration
    Ax_vel_dyn(k+1,find(names == strcat('FHx',num2str(k))))  = -h/m;%Euler Integration
end
%               Build b vector
bx_vel_dyn = zeros(size(Ax_vel_dyn,1),1);
%--------------------------------------------------------------
%           y-axis position dynamics
%               Build A matrix
Ay_pos_dyn = zeros(yLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    Ay_pos_dyn(k+1,find(names == strcat('y',num2str(k+1)))) = 1;
    Ay_pos_dyn(k+1,find(names == strcat('y',num2str(k))))   =-1;
%     Ay_pos_dyn(k+1,find(names == strcat('ydot',num2str(k+1)))) = -1/2*h;
%     Ay_pos_dyn(k+1,find(names == strcat('ydot',num2str(k)))) = -1/2*h;  
    Ay_pos_dyn(k+1,find(names == strcat('ydot',num2str(k)))) = -h; %Euler Integration
end
%               Build b vector
by_pos_dyn = zeros(size(Ay_pos_dyn,1),1);
%---------------------------------------------------------------
%           y-axis velocity dynamics
%               Build A matrix
Ay_vel_dyn = zeros(ydotLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    Ay_vel_dyn(k+1,find(names == strcat('ydot',num2str(k+1)))) = 1;
    Ay_vel_dyn(k+1,find(names == strcat('ydot',num2str(k))))   =-1;
%     Ay_vel_dyn(k+1,find(names == strcat('FFy',num2str(k+1))))  =-1/2*h/m;
%     Ay_vel_dyn(k+1,find(names == strcat('FHy',num2str(k+1))))  =-1/2*h/m;
%     Ay_vel_dyn(k+1,find(names == strcat('FFy',num2str(k))))    =-1/2*h/m;
%     Ay_vel_dyn(k+1,find(names == strcat('FHy',num2str(k))))    =-1/2*h/m;
    Ay_vel_dyn(k+1,find(names == strcat('FFy',num2str(k))))    =-h/m; %Euler Integration
    Ay_vel_dyn(k+1,find(names == strcat('FHy',num2str(k))))    =-h/m; %Euler Integration
end
%               Build b vector
by_vel_dyn = repmat(-h*g,size(Ay_vel_dyn,1),1);
%---------------------------------------------------------------
%   Foot/End-Effector Dynamics
%---------------------------------------------------------------
%       Front Leg PF
%           x-axis dynamics (only velocities)
%               Build A matrix
APFx_dyn = zeros(PFxLength-1, namesLength);
for k = 0:TimeSeriesLength - 2
    APFx_dyn(k+1,find(names == strcat('PFx',num2str(k+1)))) = 1;
    APFx_dyn(k+1,find(names == strcat('PFx',num2str(k))))   =-1;
    %APFx_dyn(k+1,find(names == strcat('PFxdot',num2str(k+1)))) = -1/2*h;
    %APFx_dyn(k+1,find(names == strcat('PFxdot',num2str(k))))   =-1/2*h;
    APFx_dyn(k+1,find(names == strcat('PFxdot',num2str(k))))   =-h; %euler integration
end
%               Build b vector
bPFx_dyn = zeros(size(APFx_dyn,1),1);
%---------------------------------------------------------------
%           y-axis dynamics (only velocities)
%               Build A matrix
APFy_dyn = zeros(PFyLength-1, namesLength);
for k = 0:TimeSeriesLength - 2
    APFy_dyn(k+1,find(names == strcat('PFy',num2str(k+1)))) = 1;
    APFy_dyn(k+1,find(names == strcat('PFy',num2str(k))))   =-1;
    %APFy_dyn(k+1,find(names == strcat('PFydot',num2str(k+1)))) = -1/2*h;
    %APFy_dyn(k+1,find(names == strcat('PFydot',num2str(k))))   =-1/2*h;
    APFy_dyn(k+1,find(names == strcat('PFydot',num2str(k))))   =-h; %Euler Integration
end
%               Build b vector
bPFy_dyn = zeros(size(APFy_dyn,1),1);
%---------------------------------------------------------------
%       Hind Leg PH
%           x-axis dynamics (only velocities)
%               Build A matrix
APHx_dyn = zeros(PHxLength-1, namesLength);
for k = 0:TimeSeriesLength - 2
    APHx_dyn(k+1,find(names == strcat('PHx',num2str(k+1)))) = 1;
    APHx_dyn(k+1,find(names == strcat('PHx',num2str(k))))   =-1;
    %APHx_dyn(k+1,find(names == strcat('PHxdot',num2str(k+1)))) = -1/2*h;
    %APHx_dyn(k+1,find(names == strcat('PHxdot',num2str(k))))   =-1/2*h;
    APHx_dyn(k+1,find(names == strcat('PHxdot',num2str(k))))   =-h; %Euler Integration
end
%               Build b vector
bPHx_dyn = zeros(size(APHx_dyn,1),1);
%---------------------------------------------------------------
%           y-axis dynamics (only velocities)
%               Build A matrix
APHy_dyn = zeros(PHyLength-1, namesLength);
for k = 0:TimeSeriesLength - 2
    APHy_dyn(k+1,find(names == strcat('PHy',num2str(k+1)))) = 1;
    APHy_dyn(k+1,find(names == strcat('PHy',num2str(k))))   =-1;
    %APHy_dyn(k+1,find(names == strcat('PHydot',num2str(k+1)))) = -1/2*h;
    %APHy_dyn(k+1,find(names == strcat('PHydot',num2str(k))))   =-1/2*h;
    APHy_dyn(k+1,find(names == strcat('PHydot',num2str(k))))   =-h; %Euler Integration
end
%               Build b vector
bPHy_dyn = zeros(size(APHy_dyn,1),1);
%---------------------------------------------------------------
%           Collect System Dynamic Constraints (Centroidal + End-effector Dynamics)
%---------------------------------------------------------------
Adyn = [Ax_pos_dyn;Ax_vel_dyn;Ay_pos_dyn;Ay_vel_dyn;APFx_dyn;APFy_dyn;APHx_dyn;APHy_dyn];
bdyn = [bx_pos_dyn;bx_vel_dyn;by_pos_dyn;by_vel_dyn;bPFx_dyn;bPFy_dyn;bPHx_dyn;bPHy_dyn];
%---------------------------------------------------------------
%       Complementarity Constraints
%---------------------------------------------------------------
%           Parameter settings
%---------------------------------------------------------------
height = 0; %terrain height
Mpos_y = 50; %big-M for Foot position in y-axis
Mvel = 5; %big-M for Foot velocity in both x and y axis
Mfx = 10000; %big-M for foot-ground reaction forces for x-axis
Mfy = 10000; %big-M for foot-ground reaction forces for y-axis
%---------------------------------------------------------------
%           Front Leg
%---------------------------------------------------------------
%               Foot/End-Effector Position (Pairs)
%---------------------------------------------------------------
%                   1st Constraint: PFy <= height + Mpos_y*(1-CF) -> PFy + Mpos_y*CF <=height + Mpos_y
%                   Build A matrix
APFy_Con1 = zeros(CFLength,namesLength);
for k = 1:CFLength
    APFy_Con1(k,find(names == strcat('PFy',num2str(k-1)))) = 1;
    APFy_Con1(k,find(names == strcat('CF',num2str(k-1)))) = Mpos_y;
end
%                   Build B matrix
bPFy_Con1 = repmat(height + Mpos_y, size(APFy_Con1,1),1);
%                   Setup constraint sense
TypePFy_Con1 = repmat(-1,size(APFy_Con1,1),1);%-1 <=, 0 ==, 1 >=
% %---------------------------------------------------------------
% %                   2nd Constraint: PFy >= 0 (height)
% %                   Build A matrix
APFy_Con2 = zeros(CFLength,namesLength);
for k = 1:CFLength
    APFy_Con2(k,find(names == strcat('PFy',num2str(k-1)))) = 1;
end
%                   Build B matrix
bPFy_Con2 = repmat(height,size(APFy_Con2,1),1);
%                   Setup Constraint sense
TypePFy_Con2 = repmat(1,size(APFy_Con2,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%               Foot/End-Effector Velocity
%                   x-axis
%---------------------------------------------------------------
%                       1st Constraint: PFxdot <= 0 + M_vel*(1-CF) -> PFxdot + Mvel*CF <= 0 + Mvel
%                       Build A matrix
APFxdot_Con1 = zeros(CFLength,namesLength);
for k = 1:CFLength
    APFxdot_Con1(k,find(names == strcat('PFxdot',num2str(k-1)))) = 1;
    APFxdot_Con1(k,find(names == strcat('CF',num2str(k-1)))) = Mvel;
end
%                       Build b vector
bPFxdot_Con1 = repmat(Mvel,size(APFxdot_Con1,1),1);
%                       Setup Constraint Sense
TypePFxdot_Con1 = repmat(-1,size(APFxdot_Con1,1),1); %-1 <=, 0 ==, 1 >=
%----------------------------------------------------------------
%                       2nd Constraint: PFxdot >= 0 - Mvel(1-CF) -> PFxdot - Mvel*CF >= 0 - Mvel
%                       Build A matrix
APFxdot_Con2 = zeros(CFLength,namesLength);
for k = 1:CFLength
    APFxdot_Con2(k,find(names == strcat('PFxdot',num2str(k-1)))) = 1;
    APFxdot_Con2(k,find(names == strcat('CF',num2str(k-1)))) = -Mvel;
end
%                       Build b vector
bPFxdot_Con2 = repmat(-Mvel,size(APFxdot_Con2,1),1);
%                       Setup Constraint type
TypePFxdot_Con2 = repmat(1,size(APFxdot_Con2,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                   y-axis
%---------------------------------------------------------------
%                       1st Constraint: PFydot <= 0 + M_vel*(1-CF) -> PFydot + Mvel*CF <= 0 + Mvel
%                       Build A matrix
APFydot_Con1 = zeros(CFLength,namesLength);
for k = 1:CFLength
    APFydot_Con1(k,find(names == strcat('PFydot',num2str(k-1)))) = 1;
    APFydot_Con1(k,find(names == strcat('CF',num2str(k-1)))) = Mvel;
end
%                       Build b vector
bPFydot_Con1 = repmat(Mvel,size(APFydot_Con1,1),1);
%                       Setup Constraint Sense
TypePFydot_Con1 = repmat(-1,size(APFydot_Con1,1),1); %-1 <=, 0 ==, 1 >=
%----------------------------------------------------------------
%                       2nd Constraint: PFydot >= 0 - Mvel(1-CF) -> PFydot - Mvel*CF >= 0 - Mvel
%                       Build A matrix
APFydot_Con2 = zeros(CFLength,namesLength);
for k = 1:CFLength
    APFydot_Con2(k,find(names == strcat('PFydot',num2str(k-1)))) = 1;
    APFydot_Con2(k,find(names == strcat('CF',num2str(k-1)))) = -Mvel;
end
%                       Build b vector
bPFydot_Con2 = repmat(-Mvel,size(APFydot_Con2,1),1);
%                       Setup Constraint Sense
TypePFydot_Con2 = repmat(1,size(APFydot_Con2,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%               Foot-Ground Reaction Forces
%---------------------------------------------------------------
%                   x-axis
%---------------------------------------------------------------
%                       1st Constraint: FFx <= 0 + Mfx*CF -> FFx - Mfx*CF <= 0
%                       Build A matrix
AFFx_Con1 = zeros(CFLength,namesLength);
for k = 1:CFLength
    AFFx_Con1(k,find(names == strcat('FFx',num2str(k-1)))) = 1;
    AFFx_Con1(k,find(names == strcat('CF',num2str(k-1))))  = -Mfx;
end
%                       Build b vector
bFFx_Con1 = zeros(size(AFFx_Con1,1),1);
%                       Setup Constraint Type
TypeFFx_Con1 = repmat(-1,size(AFFx_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                       2nd Constraint: FFx >= 0 - Mfx*Cf -> FFx + Mfx*CF >= 0
%                       Build A matrix
AFFx_Con2 = zeros(CFLength,namesLength);
for k = 1:CFLength
    AFFx_Con2(k,find(names == strcat('FFx',num2str(k-1)))) = 1;
    AFFx_Con2(k,find(names == strcat('CF',num2str(k-1))))  = Mfx;
end
%                       Build b vector
bFFx_Con2 = zeros(size(AFFx_Con2,1),1);
%                       Setup Constraint Type
TypeFFx_Con2 = repmat(1,size(AFFx_Con2,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                   y-axis
%---------------------------------------------------------------
%                       1st Constraint: FFy <= 0 + Mfy*CF -> FFy - Mfy*CF <= 0
%                       Build A matrix
AFFy_Con1 = zeros(CFLength,namesLength);
for k = 1:CFLength
    AFFy_Con1(k,find(names == strcat('FFy',num2str(k-1)))) = 1;
    AFFy_Con1(k,find(names == strcat('CF',num2str(k-1)))) = -Mfy;
end
%                       Build b vector
bFFy_Con1 = zeros(size(AFFy_Con1,1),1);
%                       Setup Constraint Type
TypeFFy_Con1 = repmat(-1,size(AFFy_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                       2nd Constraint: FFy >= 0
%                       Build A matrix
AFFy_Con2 = zeros(CFLength,namesLength);
for k = 1:CFLength
    AFFy_Con2(k,find(names == strcat('FFy',num2str(k-1)))) = 1;
end
%                       Build b vector
bFFy_Con2 = zeros(size(AFFy_Con2,1),1);
%                       Setup Constraint Type
TypeFFy_Con2 = repmat(1,size(AFFy_Con2,1),1);
%---------------------------------------------------------------
%           Hind Leg
%---------------------------------------------------------------
%               Foot/End-Effector Position (Pairs)
%---------------------------------------------------------------
%                   1st Constraint: PHy <= height + Mpos_y*(1-CH) -> PHy + Mpos_y*CH <=height + Mpos_y
%                   Build A matrix
APHy_Con1 = zeros(CHLength,namesLength);
for k = 1:CHLength
    APHy_Con1(k,find(names == strcat('PHy',num2str(k-1)))) = 1;
    APHy_Con1(k,find(names == strcat('CH',num2str(k-1)))) = Mpos_y;
end
%                   Build B matrix
bPHy_Con1 = repmat(height + Mpos_y, size(APHy_Con1,1),1);
%                   Setup constraint sense
TypePHy_Con1 = repmat(-1,size(APHy_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                   2nd Constraint: PHy >= 0
%                   Build A matrix
APHy_Con2 = zeros(CHLength,namesLength);
for k = 1:CHLength
    APHy_Con2(k,find(names == strcat('PHy',num2str(k-1)))) = 1;
end
%                   Build B matrix
bPHy_Con2 = repmat(height,size(APHy_Con2,1),1);
%                   Setup Constraint sense
TypePHy_Con2 = repmat(1,size(APHy_Con2,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%               Foot/End-Effector Velocity
%                   x-axis
%---------------------------------------------------------------
%                       1st Constraint: PHxdot <= 0 + M_vel*(1-CH) -> PHxdot + Mvel*CH <= 0 + Mvel
%                       Build A matrix
APHxdot_Con1 = zeros(CHLength,namesLength);
for k = 1:CHLength
    APHxdot_Con1(k,find(names == strcat('PHxdot',num2str(k-1)))) = 1;
    APHxdot_Con1(k,find(names == strcat('CH',num2str(k-1)))) = Mvel;
end
%                       Build b vector
bPHxdot_Con1 = repmat(Mvel,size(APHxdot_Con1,1),1);
%                       Setup Constraint Sense
TypePHxdot_Con1 = repmat(-1,size(APHxdot_Con1,1),1); %-1 <=, 0 ==, 1 >=
%----------------------------------------------------------------
%                       2nd Constraint: PHxdot >= 0 - Mvel(1-CF) -> PHxdot - Mvel*CH >= 0 - Mvel
%                       Build A matrix
APHxdot_Con2 = zeros(CHLength,namesLength);
for k = 1:CHLength
    APHxdot_Con2(k,find(names == strcat('PHxdot',num2str(k-1)))) = 1;
    APHxdot_Con2(k,find(names == strcat('CH',num2str(k-1)))) = -Mvel;
end
%                       Build b vector
bPHxdot_Con2 = repmat(-Mvel,size(APHxdot_Con2,1),1);
%                       Setup Constraint type
TypePHxdot_Con2 = repmat(1,size(APHxdot_Con2,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                   y-axis
%---------------------------------------------------------------
%                       1st Constraint: PHydot <= 0 + M_vel*(1-CF) -> PHydot + Mvel*CH <= 0 + Mvel
%                       Build A matrix
APHydot_Con1 = zeros(CHLength,namesLength);
for k = 1:CHLength
    APHydot_Con1(k,find(names == strcat('PHydot',num2str(k-1)))) = 1;
    APHydot_Con1(k,find(names == strcat('CH',num2str(k-1)))) = Mvel;
end
%                       Build b vector
bPHydot_Con1 = repmat(Mvel,size(APHydot_Con1,1),1);
%                       Setup Constraint Sense
TypePHydot_Con1 = repmat(-1,size(APHydot_Con1,1),1); %-1 <=, 0 ==, 1 >=
%----------------------------------------------------------------
%                       2nd Constraint: PHydot >= 0 - Mvel(1-CH) -> PHydot - Mvel*CH >= 0 - Mvel
%                       Build A matrix
APHydot_Con2 = zeros(CHLength,namesLength);
for k = 1:CHLength
    APHydot_Con2(k,find(names == strcat('PHydot',num2str(k-1)))) = 1;
    APHydot_Con2(k,find(names == strcat('CH',num2str(k-1)))) = -Mvel;
end
%                       Build b vector
bPHydot_Con2 = repmat(-Mvel,size(APHydot_Con2,1),1);
%                       Setup Constraint Sense
TypePHydot_Con2 = repmat(1,size(APHydot_Con2,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%               Foot-Ground Reaction Forces
%---------------------------------------------------------------
%                   x-axis
%---------------------------------------------------------------
%                       1st Constraint: FHx <= 0 + Mfx*CH -> FHx - Mfx*CH <= 0
%                       Build A matrix
AFHx_Con1 = zeros(CHLength,namesLength);
for k = 1:CHLength
    AFHx_Con1(k,find(names == strcat('FHx',num2str(k-1)))) = 1;
    AFHx_Con1(k,find(names == strcat('CH',num2str(k-1))))  = -Mfx;
end
%                       Build b vector
bFHx_Con1 = zeros(size(AFHx_Con1,1),1);
%                       Setup Constraint Type
TypeFHx_Con1 = repmat(-1,size(AFHx_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                       2nd Constraint: FHx >= 0 - Mfx*CH -> FHx + Mfx*CH >= 0
%                       Build A matrix
AFHx_Con2 = zeros(CHLength,namesLength);
for k = 1:CHLength
    AFHx_Con2(k,find(names == strcat('FHx',num2str(k-1)))) = 1;
    AFHx_Con2(k,find(names == strcat('CH',num2str(k-1))))  = Mfx;
end
%                       Build b vector
bFHx_Con2 = zeros(size(AFHx_Con2,1),1);
%                       Setup Constraint Type
TypeFHx_Con2 = repmat(1,size(AFHx_Con2,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                   y-axis
%---------------------------------------------------------------
%                       1st Constraint: FHy <= 0 + Mfy*CH -> FHy - Mfy*CH <= 0
%                       Build A matrix
AFHy_Con1 = zeros(CHLength,namesLength);
for k = 1:CHLength
    AFHy_Con1(k,find(names == strcat('FHy',num2str(k-1)))) = 1;
    AFHy_Con1(k,find(names == strcat('CH',num2str(k-1)))) = -Mfy;
end
%                       Build b vector
bFHy_Con1 = zeros(size(AFHy_Con1,1),1);
%                       Setup Constraint Type
TypeFHy_Con1 = repmat(-1,size(AFHy_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                       2nd Constraint: FHy >= 0
%                       Build A matrix
AFHy_Con2 = zeros(CHLength,namesLength);
for k = 1:CHLength
    AFHy_Con2(k,find(names == strcat('FHy',num2str(k-1)))) = 1;
end
%                       Build b vector
bFHy_Con2 = zeros(size(AFHy_Con2,1),1);
%                       Setup Constraint Type
TypeFHy_Con2 = repmat(1,size(AFHy_Con2,1),1);
%---------------------------------------------------------------
%           Collect Complementarity Constraints
%           Question: Should we include velocity constraint on y-axis of
%           the end-effectors
Acomplementarity = [APFy_Con1;APFy_Con2;...
                    APFxdot_Con1;APFxdot_Con2;...
                    APFydot_Con1;APFydot_Con2;...
                    AFFx_Con1;AFFx_Con2;...
                    AFFy_Con1;AFFy_Con2;...
                    APHy_Con1;APHy_Con2;...
                    APHxdot_Con1;APHxdot_Con2;...
                    APHydot_Con1;APHydot_Con2;...
                    AFHx_Con1;AFHx_Con2;...
                    AFHy_Con1;AFHy_Con2
                    ];

bcomplementarity = [bPFy_Con1;bPFy_Con2;...
                    bPFxdot_Con1;bPFxdot_Con2;...
                    bPFydot_Con1;bPFydot_Con2;...
                    bFFx_Con1;bFFx_Con2;...
                    bFFy_Con1;bFFy_Con2;...
                    bPHy_Con1;bPHy_Con2;...
                    bPHxdot_Con1;bPHxdot_Con2;...
                    bPHydot_Con1;bPHydot_Con2;...
                    bFHx_Con1;bFHx_Con2;...
                    bFHy_Con1;bFHy_Con2
                    ];

Typecomplementarity = [TypePFy_Con1;TypePFy_Con2;...
                       TypePFxdot_Con1;TypePFxdot_Con2;...
                       TypePFydot_Con1;TypePFydot_Con2;...
                       TypeFFx_Con1;TypeFFx_Con2;...
                       TypeFFy_Con1;TypeFFy_Con2;...
                       TypePHy_Con1;TypePHy_Con2;...
                       TypePHxdot_Con1;TypePHxdot_Con2;...
                       TypePHydot_Con1;TypePHydot_Con2;...
                       TypeFHx_Con1;TypeFHx_Con2;...
                       TypeFHy_Con1;TypeFHy_Con2
                       ];
%---------------------------------------------------------------
%       Kinematics Constraint
%---------------------------------------------------------------
%           Set up parameters
BodyHeight = 0.2; %m
BodyLength = 0.6;%m
minLegX = 0.1; %m 
maxLegX = 0.4; %m %0.4
minLegY = 0.1; %m %0.1
maxLegY = 0.5; %m %0.5
%---------------------------------------------------------------
%           Hind Foot
%---------------------------------------------------------------
%               y-axis
%---------------------------------------------------------------
%               First Constraint: y - PHy >= BodyHeight/2 + minLegY
%               Build A matrix:
APHy_Kine_Con1 = zeros(PHyLength, namesLength);
for k = 1:PHyLength
    APHy_Kine_Con1(k,find(names == strcat('y',num2str(k-1))))   =  1;
    APHy_Kine_Con1(k,find(names == strcat('PHy',num2str(k-1)))) = -1;
end
%               Build b vector
bPHy_Kine_Con1 = repmat(BodyHeight/2 + minLegY,size(APHy_Kine_Con1,1),1);
%               Setup Constraint Type
Type_PHy_Kine_Con1 = repmat(1,size(APHy_Kine_Con1,1),1);
%---------------------------------------------------------------
%               Second Constraint: y - PHy <= BodyHeight/2 + minLegY + maxLegY
%               Build A matrix
APHy_Kine_Con2 = zeros(PHyLength, namesLength);
for k = 1:PHyLength
    APHy_Kine_Con2(k,find(names == strcat('y',num2str(k-1)))) = 1;
    APHy_Kine_Con2(k,find(names == strcat('PHy',num2str(k-1)))) = -1;
end
%               Build b vector
bPHy_Kine_Con2 = repmat(BodyHeight/2 + minLegY + maxLegY,size(APHy_Kine_Con2,1),1);
%               Setup Constraint Type
Type_PHy_Kine_Con2 = repmat(-1,size(APHy_Kine_Con2,1),1);
%---------------------------------------------------------------
%               x-axis
%---------------------------------------------------------------
%               First Constraint: x - PHx >= minLegX
%               Build A matrix
APHx_Kine_Con1 = zeros(PHxLength, namesLength);
for k = 1:PHxLength
    APHx_Kine_Con1(k,find(names == strcat('x',num2str(k-1)))) = 1;
    APHx_Kine_Con1(k,find(names == strcat('PHx',num2str(k-1)))) = -1;
end
%               Build b vector
bPHx_Kine_Con1 = repmat(minLegX,size(APHx_Kine_Con1,1),1);
%               Setup Constraint Type
Type_PHx_Kine_Con1 = repmat(1,size(APHx_Kine_Con1,1),1);
%---------------------------------------------------------------
%               Second Constraint: x - PHx <= minLegX + maxLegX
%               Build A matrix
APHx_Kine_Con2 = zeros(PHxLength, namesLength);
for k =1:PHxLength
    APHx_Kine_Con2(k,find(names == strcat('x',num2str(k-1)))) = 1;
    APHx_Kine_Con2(k,find(names == strcat('PHx',num2str(k-1)))) = -1;
end
%               Build b vector
bPHx_Kine_Con2 = repmat(minLegX+maxLegX, size(APHx_Kine_Con2,1),1);
%               Setup Constraint Type
Type_PHx_Kine_Con2 = repmat(-1,size(APHx_Kine_Con2,1),1);
%---------------------------------------------------------------
%           Front Foot
%---------------------------------------------------------------
%               y-axis
%---------------------------------------------------------------
%               First Constraint: y - PFy >= BodyHeight/2 + minLegY
%               Build A matrix
APFy_Kine_Con1 = zeros(PFyLength, namesLength);
for k = 1:PFyLength
    APFy_Kine_Con1(k,find(names == strcat('y',num2str(k-1)))) = 1;
    APFy_Kine_Con1(k,find(names == strcat('PFy',num2str(k-1)))) = -1;
end
%               Build b vector
bPFy_Kine_Con1 = repmat(BodyHeight/2 + minLegY, size(APFy_Kine_Con1,1),1);
%               Setup Constraint Type
Type_PFy_Kine_Con1 = repmat(1,size(APFy_Kine_Con1,1),1);
%---------------------------------------------------------------
%               Second Constraint: y - PFy <= BodyHeight/2 + minLegY + maxLegY
%               Build A matrix
APFy_Kine_Con2 = zeros(PFyLength, namesLength);
for k = 1:PFyLength
    APFy_Kine_Con1(k,find(names == strcat('y',num2str(k-1)))) = 1;
    APFy_Kine_Con1(k,find(names == strcat('PFy',num2str(k-1)))) = -1;
end
%               Build b vector
bPFy_Kine_Con2 = repmat(BodyHeight/2 + minLegY + maxLegY, size(APFy_Kine_Con1,1),1);
%               Setup Constraint Type
Type_PFy_Kine_Con2 = repmat(-1, size(APFy_Kine_Con1,1),1);
%---------------------------------------------------------------
%               x-axis
%---------------------------------------------------------------
%               First Constraint: PFx - x >= minLegX
%               Build A matrix
APFx_Kine_Con1 = zeros(PFxLength, namesLength);
for k = 1:PFxLength
    APFx_Kine_Con1(k,find(names == strcat('PFx',num2str(k-1)))) = 1;
    APFx_Kine_Con1(k,find(names == strcat('x',num2str(k-1)))) = -1;
end
%               Build b vector
bPFx_Kine_Con1 = repmat(minLegX,size(APFx_Kine_Con1,1),1);
%               Setup Constraint Type
Type_PFx_Kine_Con1 = repmat(1, size(APFx_Kine_Con1,1),1);
%---------------------------------------------------------------
%               Second Constraint: PFx - x <= minLegX + maxLegX
%               Build A matrix
APFx_Kine_Con2 = zeros(PFxLength, namesLength);
for k = 1:PFxLength
    APFx_Kine_Con2(k,find(names == strcat('PFx',num2str(k-1)))) = 1;
    APFx_Kine_Con2(k,find(names == strcat('x',num2str(k-1)))) = -1;
end
%               Build b vector
bPFx_Kine_Con2 = repmat(minLegX + maxLegX, size(APFx_Kine_Con2,1),1);
%               Setup Constraint Type
Type_PFx_Kine_Con2 = repmat(-1,size(APFx_Kine_Con2,1),1);
%---------------------------------------------------------------
%           Collect Kinematics Constraint
%---------------------------------------------------------------
AKinematics = [APHy_Kine_Con1;APHy_Kine_Con2;
               APHx_Kine_Con1;APHx_Kine_Con2;
               APFy_Kine_Con1;APFy_Kine_Con2;
               APFx_Kine_Con1;APFx_Kine_Con2;
              ];
     
bKinematics = [bPHy_Kine_Con1;bPHy_Kine_Con2;
               bPHx_Kine_Con1;bPHx_Kine_Con2;
               bPFy_Kine_Con1;bPFy_Kine_Con2;
               bPFx_Kine_Con1;bPFx_Kine_Con2;
              ];
    
TypeKinematics = [Type_PHy_Kine_Con1;Type_PHy_Kine_Con2;
                  Type_PHx_Kine_Con1;Type_PHx_Kine_Con2;
                  Type_PFy_Kine_Con1;Type_PFy_Kine_Con2;
                  Type_PFx_Kine_Con1;Type_PFx_Kine_Con2;
                 ];          
%---------------------------------------------------------------
%   Set Boundary Conditions
%---------------------------------------------------------------
%       Initial State
%           Build A matrix
Ax_init = zeros(1,namesLength);
Ax_init(find(names == 'x0')) = 1;
Ay_init = zeros(1,namesLength);
Ay_init(find(names == 'y0')) = 1;
Axdot_init = zeros(1,namesLength);
Axdot_init(find(names == 'xdot0')) = 1;
Aydot_init = zeros(1,namesLength);
Aydot_init(find(names == 'ydot0')) = 1;
%               Collect into an A matrix
Ainit = [Ax_init;
         Ay_init;
         Axdot_init;
         Aydot_init
        ];
%           Build b vectors
binit = [x_init;
         y_init;
         xdot_init;
         ydot_init
        ];
%---------------------------------------------------------------
%       Terminal State
%           Build A matrix
Ax_end = zeros(1,namesLength);
Ax_end(find(names == x_label(end))) = 1;
Ay_end = zeros(1,namesLength);
Ay_end(find(names == y_label(end))) = 1;
Axdot_end = zeros(1,namesLength);
Axdot_end(find(names == xdot_label(end))) = 1;
Aydot_end = zeros(1,namesLength);
Aydot_end(find(names == ydot_label(end))) = 1;
%               Collect in to an A matrix
Aend = [Ax_end;
        Ay_end;
        Axdot_end;
        Aydot_end
       ];
%           Build b vectors
bend = [x_end;
        y_end;
        xdot_end;
        ydot_end
       ];
%---------------------------------------------------------------
%===============================================================
%Build Constraints
%===============================================================
%   Collect all Aeq matrices and beq vectors for linear equality
%   constraints
Aeq = [Ainit;Aend;Adyn]; 
beq = [binit;bend;bdyn];
%---------------------------------------------------------------
%   Collect all A, b and Type for linear inequality constraints
A = [Acomplementarity;
     AKinematics
     ];
b = [bcomplementarity;
     bKinematics
     ];
IneqType = [Typecomplementarity;
            TypeKinematics
            ];
%------------------------------------------------------------
%   Variable Lower and Upper Boundaries
lb = [repmat(-inf,1, sum(LengthList(find(varList == "x"):find(varList == "FHy")))),repmat(0,1,CFLength + CHLength)];
ub = [repmat( inf,1, sum(LengthList(find(varList == "x"):find(varList == "FHy")))),repmat(1,1,CFLength + CHLength)];
%   Variable Type
vtype = [repmat('C', 1, sum(LengthList(find(varList == "x"):find(varList == "FHy")))), repmat('B',1, CFLength + CHLength)];
%=============================================================
%Call Solvers
if solver == "knitro"
    %Display Info
    disp("Use Knitro")
    disp(" ")
    %Set up problem
        %Build Objective Function
    objfunc = @(vars)cost(vars,Q); %!!!!Cost function may need quadrature as well
        %nonlinear constraints
    nlcon = @(vars) nlconstraint(vars);
        %other parameters
    objFnType = 0;
    cFnType = [];
    x0 = rand(namesLength,1);
    %flip constraints, Knitro only takes < constraints
    A(find(IneqType == 1),:) = -A(find(IneqType == 1),:);
    b(find(IneqType == 1)) = -b(find(IneqType == 1));
    %transpose lower and upper bounds, Knitro requires
    lb = lb';
    ub = ub';
    %reset vType vector, Knitro requires
    vtype = [repmat(0, sum(LengthList(find(varList == "x"):find(varList == "FHy"))), 1); repmat(2, CFLength + CHLength, 1)]; %0 -> continuous variable, 2 -> discrete variable
    %call knitro
    options = optimset('Display','iter');
    [result.x,result.objval,result.exitflag,optInfo] = knitromatlab_mip(objfunc,x0,A,b,Aeq,beq,lb,ub,nlcon,vtype,objFnType,cFnType,[],options,'mipoptions.opt');
    
elseif solver == "gurobi"
    %Dispaly Info
    disp("Use Gurobi");
    disp(" ");
    %Set up problem
    model.Q = Q;
    model.modelsense = 'min';
    model.vtype = vtype;
    model.lb = lb;
    model.ub = ub;
    model.A = sparse([A;Aeq]);
    model.rhs = [b;beq];
    model.sense = IneqType;
    model.sense(find(model.sense == -1)) ='<';
    model.sense(find(model.sense == 1)) ='>';
    model.sense = [model.sense;repmat('=',length(beq),1)];
    %Call Gurobi
    result = gurobi(model);
    gurobi_write(model, 'MILP.mps');
    
else
    disp("Unknown solver");
    disp{" "}
end

%========================================================================
%Extract Input Results
FFx_result = result.x(find(names == 'FFx0'):find(names == FFx_label(end)));
FFy_result = result.x(find(names == 'FFy0'):find(names == FFy_label(end)));
FHx_result = result.x(find(names == 'FHx0'):find(names == FHx_label(end)));
FHy_result = result.x(find(names == 'FHy0'):find(names == FHy_label(end)));

NetForceX = FFx_result + FHx_result;
NetForceY = FFy_result + FHy_result;
%% Plot Figures

%Plot CoM trajectory in 2D Space
figure(1)
plot(result.x(find(names == 'x0'):find(names == x_label(end))),result.x(find(names == 'y0'):find(names == y_label(end))),'LineWidth',2)
title('CoM Trajectory')
xlabel('x-axis Position')
ylabel('y-axis Position')
set(gca,'FontSize',20)

%Plot CoM Velocity in x and y axis
figure(2)
subplot(1,2,1)
plot(TimeSeries,result.x(find(names == 'xdot0'):find(names == xdot_label(end))),'LineWidth',2)
hold on
title('CoM Velocity in x-axis')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
set(gca,'FontSize',20)

subplot(1,2,2)
plot(TimeSeries,result.x(find(names == 'ydot0'):find(names == ydot_label(end))),'LineWidth',2)
title('CoM Velocity in y-axis')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
set(gca,'FontSize',20)

%Plot Contact Configuration and Foot-Ground Reaction Forces
figure(3)
suptitle('Contact Configuration and Foot-Ground Reaction Forces')

subplot(3,2,1)
stairs(TimeSeries,result.x(find(names == 'CF0'):find(names == CF_label(end))),'LineWidth',2)
title('Contact Configuration of Front Leg, 0/1: on/off')
xlabel('Time (s)')
set(gca,'FontSize',20)

subplot(3,2,2)
stairs(TimeSeries,result.x(find(names == 'CH0'):find(names == CH_label(end))),'LineWidth',2)
title('Contact Configuration of Hind Leg, 0/1: on/off')
xlabel('Time (s)')
set(gca,'FontSize',20)

subplot(3,2,3)
stairs(TimeSeries,result.x(find(names == 'FFx0'):find(names == FFx_label(end))),'LineWidth',2)
title('Horizontal Forces (x-axis) from Front Foot')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',20)

subplot(3,2,4)
stairs(TimeSeries,result.x(find(names == 'FHx0'):find(names == FHx_label(end))),'LineWidth',2)
title('Horizontal Forces (x-axis) from Hind Foot')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',20)

subplot(3,2,5)
stairs(TimeSeries,result.x(find(names == 'FFy0'):find(names == FFy_label(end))),'LineWidth',2)
title('Vertical Forces (y-axis) from Front Foot')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',20)

subplot(3,2,6)
stairs(TimeSeries,result.x(find(names == 'FHy0'):find(names == FHy_label(end))),'LineWidth',2)
title('Vertical Forces (y-axis) from Hind Foot')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',20)

%Plot Contact Configuration and End-Effector Positions
figure(4)
suptitle('Contact Configuration and Vertical End-Effector Positions')

subplot(3,2,1)
stairs(TimeSeries,result.x(find(names == 'CF0'):find(names == CF_label(end))),'LineWidth',2)
title('Contact Configuration of Front Foot, 0/1: on/off')
xlabel('Time (s)')
set(gca,'FontSize',20)

subplot(3,2,2)
stairs(TimeSeries,result.x(find(names == 'CH0'):find(names == CH_label(end))),'LineWidth',2)
title('Contact Configuration of Hind Foot, 0/1: on/off')
xlabel('Time (s)')
set(gca,'FontSize',20)

subplot(3,2,3)
plot(TimeSeries,result.x(find(names == 'PFx0'):find(names == PFx_label(end))),'LineWidth',2)
title('Horizontal Position (x-axis) of Front Foot')
xlabel('Time (s)')
ylabel('Position (m)')
set(gca,'FontSize',20)

subplot(3,2,4)
plot(TimeSeries,result.x(find(names == 'PHx0'):find(names == PHx_label(end))),'LineWidth',2)
title('Horizontal Position (x-axis) of Hind Foot')
xlabel('Time (s)')
ylabel('Position (m)')
set(gca,'FontSize',20)

subplot(3,2,5)
plot(TimeSeries,result.x(find(names == 'PFy0'):find(names == PFy_label(end))),'LineWidth',2)
title('Verticle Position of Front Foot')
xlabel('Time (s)')
ylabel('Position (m)')
set(gca,'FontSize',20)

subplot(3,2,6)
plot(TimeSeries,result.x(find(names == 'PHy0'):find(names == PHy_label(end))),'LineWidth',2)
title('Verticle Position of Hind Foot')
xlabel('Time (s)')
ylabel('Position (m)')
set(gca,'FontSize',20)

%Plot Contact Configuration and End-Effector Velocities
figure(5)
suptitle('Contact Configuration and End-Effector Velocities')

subplot(3,2,1)
stairs(TimeSeries,result.x(find(names == 'CF0'):find(names == CF_label(end))),'LineWidth',2)
title('Contact Configuration of Front Foot, 0/1: on/off')
xlabel('Time (s)')
set(gca,'FontSize',20)

subplot(3,2,2)
stairs(TimeSeries,result.x(find(names == 'CH0'):find(names == CH_label(end))),'LineWidth',2)
title('Contact Configuration of Hind Foot, 0/1: on/off')
xlabel('Time (s)')
set(gca,'FontSize',20)

subplot(3,2,3)
stairs(TimeSeries,result.x(find(names == 'PFxdot0'):find(names == PFxdot_label(end))),'LineWidth',2)
title('Horizontal Velocity (x-axis) of Front Foot')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
set(gca,'FontSize',20)

subplot(3,2,4)
stairs(TimeSeries,result.x(find(names == 'PHxdot0'):find(names == PHxdot_label(end))),'LineWidth',2)
title('Horizontal Velocity (x-axis) of Hind Foot')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
set(gca,'FontSize',20)

subplot(3,2,5)
stairs(TimeSeries,result.x(find(names == 'PFydot0'):find(names == PFydot_label(end))),'LineWidth',2)
title('Vertical Velocity (y-axis) of Front Foot')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
set(gca,'FontSize',20)

subplot(3,2,6)
stairs(TimeSeries,result.x(find(names == 'PHydot0'):find(names == PHydot_label(end))),'LineWidth',2)
title('Vertical Velocity (y-axis) of Hind Foot')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
set(gca,'FontSize',20)

%%
figure(6)
hold on
plot(result.x(find(names == 'x0'):find(names == x_label(end))),result.x(find(names == 'y0'):find(names == y_label(end))),'o','LineWidth',2)
plot(result.x(find(names == 'PFx0'):find(names == PFx_label(end))),result.x(find(names == 'PFy0'):find(names == PFy_label(end))),'o')
plot(result.x(find(names == 'PHx0'):find(names == PHx_label(end))),result.x(find(names == 'PHy0'):find(names == PHy_label(end))),'o')