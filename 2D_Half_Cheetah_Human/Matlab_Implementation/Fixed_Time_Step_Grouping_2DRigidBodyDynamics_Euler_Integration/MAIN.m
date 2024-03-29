% Mixed-Integer Nonlinear Optimization in 2D Case (Planar Cheetah)
% Mode Selection described as foot-ground contact changes 0/1

%Check Readme for notes and future improvements

clear;
clc;

%========================================================
%Command Line Logging
diary off
diary_filename = strcat('log-', datestr(datetime('now'),30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
diary(diary_filename);

%=========================================================
%Display Script Information
disp('Date and Time:');
disp(datetime('now'));
disp(['Correspond Log File Name: ', diary_filename]);
disp(' ');
disp('Locomotion Control with Mixed-integer Nonlinear Optimization in 2D')
disp('Time Step Grouping Script - All the Phases have the same Number of Time Steps')
disp(' ')

%==========================================================
%Add solvers' Path
%Knitro
addpath('C:\Program Files\Artelys\Knitro 11.1.0\knitromatlab')
%Gurobiaddpath('C:\gurobi810')
%==========================================================
%Choose Solver
solver = "knitro";
%Knitro Option File
option_file = 'mipoptions_default.opt';
%solver = "gurobi";
%==========================================================

%==========================================================
%Inertia Parameters(Information from MIT Cheetah 3)
m = 45; %kg
I = 2.1; %kg m^2 Izz
g = 9.80665; %m/s^2
%==========================================================

%==========================================================
%Environment Information
TerrainHeight = 0; %terrain height
TerrainNorm = [0,1];
miu = 0.5; %friction coefficient
%==========================================================

%==========================================================
%Time Step Parameter Settings
%h = 0.05; %Time Step in seconds
NumTimeSteps = input('Input Number of Time Steps (e.g. 20): \n');
disp(' ');
h = input('Input Size of Time Steps (e.g. 0.05): \n');
disp(' ');
NumPhases = input('Input Number of Phases (type the same value with number of time steps if we want to allow mode change per time step): \n');
disp(' ');
%NumTimeSteps = 20; %number of time steps
%NumPhases = 4; %Number of Phases
if mod(NumTimeSteps,NumPhases) ~= 0
    ME_NumLocalTimeSteps = MException('Initialization:NumofLocalTimeSteps','Number of Time Steps for Each Phase is not an Integer');
    throw(ME_NumLocalTimeSteps)
end
NumLocalTimeSteps = NumTimeSteps/NumPhases; %Number of local time steps for each phase
EndTime = h*NumTimeSteps; %in seconds
% missing checking the endtime is the multiple of the time step
TimeSeries = 0:h:EndTime;
TimeSeriesLength = length(TimeSeries);
%=========================================================

%=========================================================
%Parameter Setting
%---------------------------------------------------------
%   Big-M parameters for complementarity constraints
%---------------------------------------------------------
height = TerrainHeight;
Mpos_y = 50; %big-M for Foot position in y-axis
Mvel = 2; %big-M for Foot velocity in both x and y axis
Mfx = 10000; %big-M for foot-ground reaction forces for x-axis
Mfy = 10000; %big-M for foot-ground reaction forces for y-axis
%---------------------------------------------------------
%   Kinematics Constraint Parameters
%---------------------------------------------------------
%       Body Size
BodyLength = 0.6; %m
BodyHeight = 0.2; %m
%       Default foot positions in local robot frame
Default_Leg_Length = 0.45; %default leg lenght, distance from the default leg y to torso
PFcenterX = 1/2*BodyLength;
PFcenterY = -(1/2*BodyHeight + Default_Leg_Length);
PHcenterX = -1/2*BodyLength;
PHcenterY = -(1/2*BodyHeight + Default_Leg_Length);
%PF_center = [1/2*BodyLength,  -(1/2*BodyHeight + Default_Leg_Length)]';
%PH_center = [-1/2*BodyLength, -(1/2*BodyHeight + Default_Leg_Length)]';
%       Kinematics Bounding Box Constraint
BoundingBox_Width = 0.3;
BoundingBox_Height= 0.3;
BoundingBox = [BoundingBox_Width,BoundingBox_Height]';
%==========================================================

%=========================================================
%Initial Conditions
x_init = 0;
y_init = 1/2*BodyHeight + Default_Leg_Length; %0.4
theta_init = 0;
xdot_init = 0;
ydot_init = 0;
thetadot_init = 0;
%---------------------------------------------------------
%Terminal Conditionsconsta
x_end = input('Input Goal State (Travel Distance e.g. 0.5): \n');
disp(' ');
%x_end = 0.5; %20
y_end = 1/2*BodyHeight + Default_Leg_Length; %0.4
theta_end = 0;
xdot_end = 0;
ydot_end = 0;
thetadot_end = 0;
%---------------------------------------------------------
%Test if initial and terminal conditions meet kinematics constraint
%robot height should set in a way that the highest foot position is not
%under the terrain height, otherwise conflicts with complementarity
%constraint
if (y_init - 1/2*BodyHeight - Default_Leg_Length + BoundingBox_Height/2) <= 0
    ME_InitHeight = MException('Initialization:ProblematicInitialHeight','Initial Hight Error (y_init), Increase Initial Height');
    throw(ME_InitHeight)
end

if (y_end - 1/2*BodyHeight - Default_Leg_Length + BoundingBox_Height/2) <= 0
    ME_TerminalHeight = MException('Initialization:ProblematicTerminalHeight','Terminal Hight Error (y_end), Increase Terminal Height');
    throw(ME_TerminalHeight)
end
%=========================================================

%==========================================================
%Settings for Matrix Sparsity
%----------------------------------------------------------
Q_Sparsity = 1; %on/off (1/0) of Q matrix Sparsity
A_Sparsity = 1; %on/off (1/0) of A matrix Sparsity
%----------------------------------------------------------
%Display Settings
if Q_Sparsity == 1
    disp('Q matrix in the Cost Function is Sparse')
elseif Q_Sparsity == 0
    disp('Q matrix in the Cost Function is Non-Sparse')
else
    ME_QSparsity = MException('Initialization:QMatrixSparsity','Wrong Setings of Q Matrix Sparsity');
    throw(ME_QSparsity)
end

if A_Sparsity == 1
    disp('A matrix for Constraints is Sparse')
elseif A_Sparsity == 0
    disp('A matrix for Constraints is Non-Sparse')
else
    ME_ASparsity = MException('Initialization:AMatrixSparsity','Wrong Setings of A Matrix Sparsity');
    throw(ME_ASparsity)
end
disp(' ')
%----------------------------------------------------------
%Effects of Sparsity
%   Non-sparse matrix (Q) gives rise to faster computing speed, but seems
%   like take more memory
%   Sparse Matrix (Q) seems like take less memory
%   Non-Sparse/Sparse Matrix has nothing to do with feasibility. Try
%   multiple times if the solver reports the problem is infeasible to solve
%==========================================================

%==========================================================
%Settings for Soft/Hard Constraint of Terminal Conditions
%----------------------------------------------------------
SoftTerminalConstraint = input('Set How the Terminal Conditions will be handled; 0: Hard Constraint; 1: Soft Constraint in Cost Function \n'); %on/off (1/0) of putting Terminal conditions into cost function (soft constraint) or hard constraints
%----------------------------------------------------------
%Display Settings
if SoftTerminalConstraint == 1
    disp('Terminal Condition is Set as Soft Constraints in the Cost Function')
elseif SoftTerminalConstraint == 0
    disp('Terminal Condition is Set as Hard Constraints')
else
    ME_TerminalCondition = MException('Initialization:TerminalCondition','Wrong Settings of on/off of Soft/Hard Constraint Formulation of Terminal Conditions');
    throw(ME_TerminalCondition)
end
disp(' ')
%==========================================================

%==========================================================
%Define/Create Modeling Variables
%----------------------------------------------------------
%   States: r = [x,y,theta,xdot,ydot,thetadot]
%       x: horizontal position
%       y: vertical position
%       theta: torso orientation
%       xdot: horizontal velocity
%       ydot: vertical velocity
%       thetadot: torso angular acceleration
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
    theta_label(i) = strcat('theta',num2str(i-1));
    xdot_label(i) = strcat('xdot',num2str((i-1)));
    ydot_label(i) = strcat('ydot',num2str((i-1)));
    thetadot_label(i) = strcat('thetadot',num2str(i-1));
end
%   Save List Length
xLength = length(x_label);
yLength = length(y_label);
thetaLength = length(theta_label);
xdotLength = length(xdot_label);
ydotLength = length(ydot_label);
thetadotLength = length(thetadot_label);
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
CF_label = strings(1,NumPhases);
CH_label = strings(1,NumPhases);
%   Assign Variable Names
for i = 1:NumPhases
    CF_label(i) = strcat('CF',num2str(i));
    CH_label(i) = strcat('CH',num2str(i));
end
%   Save List Length
CFLength = length(CF_label);
CHLength = length(CH_label);
%----------------------------------------------------------
%   Full Decision Variable Name List
names = [x_label, y_label, theta_label, xdot_label, ydot_label, thetadot_label, PFx_label, PFy_label, PFxdot_label, PFydot_label, PHx_label, PHy_label, PHxdot_label, PHydot_label, FFx_label, FFy_label, FHx_label, FHy_label, CF_label, CH_label];
%   Length of the name list
namesLength = length(names);
%   Length list of all variable list
LengthList = [xLength, yLength, thetaLength, xdotLength, ydotLength, thetadotLength, PFxLength, PFyLength, PFxdotLength, PFydotLength, PHxLength, PHyLength, PHxdotLength, PHydotLength, FFxLength, FFyLength, FHxLength, FHyLength, CFLength, CHLength];
%----------------------------------------------------------
%   List for all decision variable names
varList = ["x", "y", "theta", "xdot", "ydot", "thetadot", "PFx", "PFy", "PFxdot", "PFydot", "PHx", "PHy", "PHxdot", "PHydot", "FFx", "FFy", "FHx", "FHy", "CF", "CH"];
%   Length of the decision varibale name list
varListLength = length(varList);
%----------------------------------------------------------
%   Extract Variable Index
xIdx_init = find(names == 'x0');
xIdx_end  = find(names == x_label(end));
xdotIdx_init = find(names == 'xdot0');
xdotIdx_end = find(names == xdot_label(end));
yIdx_init = find(names == 'y0');
yIdx_end  = find(names == y_label(end));
ydotIdx_init = find(names == 'ydot0');
ydotIdx_end = find(names == ydot_label(end));
thetaIdx_init = find(names == 'theta0');
thetaIdx_end  = find(names == theta_label(end));
thetadotIdx_init = find(names == 'thetadot0');
thetadotIdx_end  = find(names == thetadot_label(end));
PFxIdx_init = find(names == 'PFx0');
PFxIdx_end  = find(names == PFx_label(end));
PFyIdx_init = find(names == 'PFy0');
PFyIdx_end  = find(names == PFy_label(end));
PHxIdx_init = find(names == 'PHx0');
PHxIdx_end  = find(names == PHx_label(end));
PHyIdx_init = find(names == 'PHy0');
PHyIdx_end  = find(names == PHy_label(end));
FFxIdx_init = find(names == 'FFx0');
FFxIdx_end  = find(names == FFx_label(end));
FFyIdx_init = find(names == 'FFy0');
FFyIdx_end  = find(names == FFy_label(end));
FHxIdx_init = find(names == 'FHx0');
FHxIdx_end  = find(names == FHx_label(end));
FHyIdx_init = find(names == 'FHy0');
FHyIdx_end  = find(names == FHy_label(end));
%==========================================================


%==========================================================
%Setup Optimization Problem
%----------------------------------------------------------
%   Objective Function
%       minimize F^2 -> FFx(t)^2 + FFy(t)^2 + FHx(t)^2 + FHy(t)^2
%   Quadratic form:
%       minimize F'*Q*F
%   v = [x;   y;   theta;   xdot;   ydot;   thetadot;  PFx;   PFy;  PFxdot;  PFydot;  PHx;  PHy;  PHxdot;  PHydot;   FFx;    FFy;    FHx;    FHy;   CF;   CH]
%   idx: 1    2     3        4        5         6       7      8      9        10      11    12     13      14        15     16       17      18     19   20
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
Q_backup = Q;
%------------------------------------------------------------
%Sparse Q Transformation
if Q_Sparsity == 1
    Q = sparse(Q);
end
%------------------------------------------------------------

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
%           theta position dynamics --> theta(k+1)- theta(k) =h*thetadot(k) --> theta(k+1) - theta(k) - h*thetadot(k) = 0
%               Build A matrix
Atheta_pos_dyn = zeros(thetadotLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    Atheta_pos_dyn(k+1,find(names == strcat('theta',num2str(k+1)))) = 1;
    Atheta_pos_dyn(k+1,find(names == strcat('theta',num2str(k))))   = -1;
    Atheta_pos_dyn(k+1,find(names == strcat('thetadot',num2str(k)))) = -h;
end
%               Build b vector
btheta_pos_dyn = zeros(size(Atheta_pos_dyn,1),1);
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
Adyn = [Ax_pos_dyn;Ax_vel_dyn;Ay_pos_dyn;Ay_vel_dyn;Atheta_pos_dyn;APFx_dyn;APFy_dyn;APHx_dyn;APHy_dyn];
bdyn = [bx_pos_dyn;bx_vel_dyn;by_pos_dyn;by_vel_dyn;btheta_pos_dyn;bPFx_dyn;bPFy_dyn;bPHx_dyn;bPHy_dyn];
%---------------------------------------------------------------
%       Complementarity Constraints
%---------------------------------------------------------------
%           Parameter settings
%---------------------------------------------------------------
% height = 0; %terrain height
% Mpos_y = 50; %big-M for Foot position in y-axis
% Mvel = 25; %big-M for Foot velocity in both x and y axis
% Mfx = 1000; %big-M for foot-ground reaction forces for x-axis
% Mfy = 1000; %big-M for foot-ground reaction forces for y-axis
%---------------------------------------------------------------
%           Front Leg
%---------------------------------------------------------------
%               Foot/End-Effector Position (Pairs)
%---------------------------------------------------------------
%                   1st Constraint: PFy <= height + Mpos_y*(1-CF) -> PFy + Mpos_y*CF <=height + Mpos_y
%                   Build A matrix
APFy_Con1 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APFy_Con1(k+1,find(names == strcat('PFy',num2str(k)))) = 1;
    APFy_Con1(k+1,find(names == strcat('CF',num2str(floor(k/NumLocalTimeSteps)+1)))) = Mpos_y;
end
%                   Build B matrix
bPFy_Con1 = repmat(height + Mpos_y, size(APFy_Con1,1),1);
%                   Setup constraint sense
TypePFy_Con1 = repmat(-1,size(APFy_Con1,1),1);%-1 <=, 0 ==, 1 >=
% %---------------------------------------------------------------
% %                   2nd Constraint: PFy >= 0 (height)
% %                   Build A matrix
APFy_Con2 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APFy_Con2(k+1,find(names == strcat('PFy',num2str(k)))) = 1;
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
APFxdot_Con1 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APFxdot_Con1(k+1,find(names == strcat('PFxdot',num2str(k)))) = 1;
    APFxdot_Con1(k+1,find(names == strcat('CF',num2str(floor(k/NumLocalTimeSteps)+1)))) = Mvel;
end
%                       Build b vector
bPFxdot_Con1 = repmat(Mvel,size(APFxdot_Con1,1),1);
%                       Setup Constraint Sense
TypePFxdot_Con1 = repmat(-1,size(APFxdot_Con1,1),1); %-1 <=, 0 ==, 1 >=
%----------------------------------------------------------------
%                       2nd Constraint: PFxdot >= 0 - Mvel(1-CF) -> PFxdot - Mvel*CF >= 0 - Mvel
%                       Build A matrix
APFxdot_Con2 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APFxdot_Con2(k+1,find(names == strcat('PFxdot',num2str(k)))) = 1;
    APFxdot_Con2(k+1,find(names == strcat('CF',num2str(floor(k/NumLocalTimeSteps)+1)))) = -Mvel;
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
APFydot_Con1 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APFydot_Con1(k+1,find(names == strcat('PFydot',num2str(k)))) = 1;
    APFydot_Con1(k+1,find(names == strcat('CF', num2str(floor(k/NumLocalTimeSteps)+1)))) = Mvel;
end
%                       Build b vector
bPFydot_Con1 = repmat(Mvel,size(APFydot_Con1,1),1);
%                       Setup Constraint Sense
TypePFydot_Con1 = repmat(-1,size(APFydot_Con1,1),1); %-1 <=, 0 ==, 1 >=
%----------------------------------------------------------------
%                       2nd Constraint: PFydot >= 0 - Mvel(1-CF) -> PFydot - Mvel*CF >= 0 - Mvel
%                       Build A matrix
APFydot_Con2 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APFydot_Con2(k+1,find(names == strcat('PFydot',num2str(k)))) = 1;
    APFydot_Con2(k+1,find(names == strcat('CF', num2str(floor(k/NumLocalTimeSteps)+1)))) = -Mvel;
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
AFFx_Con1 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    AFFx_Con1(k+1,find(names == strcat('FFx',num2str(k)))) = 1;
    AFFx_Con1(k+1,find(names == strcat('CF', num2str(floor(k/NumLocalTimeSteps)+1))))  = -Mfx;
end
%                       Build b vector
bFFx_Con1 = zeros(size(AFFx_Con1,1),1);
%                       Setup Constraint Type
TypeFFx_Con1 = repmat(-1,size(AFFx_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                       2nd Constraint: FFx >= 0 - Mfx*Cf -> FFx + Mfx*CF >= 0
%                       Build A matrix
AFFx_Con2 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    AFFx_Con2(k+1,find(names == strcat('FFx',num2str(k)))) = 1;
    AFFx_Con2(k+1,find(names == strcat('CF', num2str(floor(k/NumLocalTimeSteps)+1))))  = Mfx;
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
AFFy_Con1 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    AFFy_Con1(k+1,find(names == strcat('FFy',num2str(k)))) = 1;
    AFFy_Con1(k+1,find(names == strcat('CF', num2str(floor(k/NumLocalTimeSteps)+1)))) = -Mfy;
end
%                       Build b vector
bFFy_Con1 = zeros(size(AFFy_Con1,1),1);
%                       Setup Constraint Type
TypeFFy_Con1 = repmat(-1,size(AFFy_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                       2nd Constraint: FFy >= 0 / [FFx, FFy]'*[TerrainNormx,TerrainNormy]
%                       Build A matrix
AFFy_Con2 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    AFFy_Con2(k+1,find(names == strcat('FFx',num2str(k)))) = TerrainNorm(1);
    AFFy_Con2(k+1,find(names == strcat('FFy',num2str(k)))) = TerrainNorm(2);
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
APHy_Con1 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APHy_Con1(k+1,find(names == strcat('PHy',num2str(k)))) = 1;
    APHy_Con1(k+1,find(names == strcat('CH', num2str(floor(k/NumLocalTimeSteps)+1)))) = Mpos_y;
end
%                   Build B matrix
bPHy_Con1 = repmat(height + Mpos_y, size(APHy_Con1,1),1);
%                   Setup constraint sense
TypePHy_Con1 = repmat(-1,size(APHy_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                   2nd Constraint: PHy >= 0
%                   Build A matrix
APHy_Con2 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APHy_Con2(k+1,find(names == strcat('PHy',num2str(k)))) = 1;
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
APHxdot_Con1 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APHxdot_Con1(k+1,find(names == strcat('PHxdot',num2str(k)))) = 1;
    APHxdot_Con1(k+1,find(names == strcat('CH', num2str(floor(k/NumLocalTimeSteps)+1)))) = Mvel;
end
%                       Build b vector
bPHxdot_Con1 = repmat(Mvel,size(APHxdot_Con1,1),1);
%                       Setup Constraint Sense
TypePHxdot_Con1 = repmat(-1,size(APHxdot_Con1,1),1); %-1 <=, 0 ==, 1 >=
%----------------------------------------------------------------
%                       2nd Constraint: PHxdot >= 0 - Mvel(1-CF) -> PHxdot - Mvel*CH >= 0 - Mvel
%                       Build A matrix
APHxdot_Con2 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APHxdot_Con2(k+1,find(names == strcat('PHxdot',num2str(k)))) = 1;
    APHxdot_Con2(k+1,find(names == strcat('CH', num2str(floor(k/NumLocalTimeSteps)+1)))) = -Mvel;
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
APHydot_Con1 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APHydot_Con1(k+1,find(names == strcat('PHydot',num2str(k)))) = 1;
    APHydot_Con1(k+1,find(names == strcat('CH', num2str(floor(k/NumLocalTimeSteps)+1)))) = Mvel;
end
%                       Build b vector
bPHydot_Con1 = repmat(Mvel,size(APHydot_Con1,1),1);
%                       Setup Constraint Sense
TypePHydot_Con1 = repmat(-1,size(APHydot_Con1,1),1); %-1 <=, 0 ==, 1 >=
%----------------------------------------------------------------
%                       2nd Constraint: PHydot >= 0 - Mvel(1-CH) -> PHydot - Mvel*CH >= 0 - Mvel
%                       Build A matrix
APHydot_Con2 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    APHydot_Con2(k+1,find(names == strcat('PHydot',num2str(k)))) = 1;
    APHydot_Con2(k+1,find(names == strcat('CH', num2str(floor(k/NumLocalTimeSteps)+1)))) = -Mvel;
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
AFHx_Con1 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    AFHx_Con1(k+1,find(names == strcat('FHx',num2str(k)))) = 1;
    AFHx_Con1(k+1,find(names == strcat('CH', num2str(floor(k/NumLocalTimeSteps)+1)))) = -Mfx;
end
%                       Build b vector
bFHx_Con1 = zeros(size(AFHx_Con1,1),1);
%                       Setup Constraint Type
TypeFHx_Con1 = repmat(-1,size(AFHx_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                       2nd Constraint: FHx >= 0 - Mfx*CH -> FHx + Mfx*CH >= 0
%                       Build A matrix
AFHx_Con2 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    AFHx_Con2(k+1,find(names == strcat('FHx',num2str(k)))) = 1;
    AFHx_Con2(k+1,find(names == strcat('CH', num2str(floor(k/NumLocalTimeSteps)+1)))) = Mfx;
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
AFHy_Con1 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    AFHy_Con1(k+1,find(names == strcat('FHy',num2str(k)))) = 1;
    AFHy_Con1(k+1,find(names == strcat('CH', num2str(floor(k/NumLocalTimeSteps)+1)))) = -Mfy;
end
%                       Build b vector
bFHy_Con1 = zeros(size(AFHy_Con1,1),1);
%                       Setup Constraint Type
TypeFHy_Con1 = repmat(-1,size(AFHy_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                       2nd Constraint: FHy >= 0 /
%                       [Fhx,Fhy]'*[TerrainNormx,TerrainNormy] >= 0
%                       Build A matrix
AFHy_Con2 = zeros(TimeSeriesLength-1,namesLength);
for k = 0:TimeSeriesLength-2
    AFHy_Con2(k+1,find(names == strcat('FHx',num2str(k)))) = TerrainNorm(1);
    AFHy_Con2(k+1,find(names == strcat('FHy',num2str(k)))) = TerrainNorm(2);
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
Atheta_init = zeros(1,namesLength);
Atheta_init(find(names == 'theta0')) = 1;
Athetadot_init = zeros(1,namesLength);
Athetadot_init(find(names == 'thetadot0')) = 1;
%               Collect into an A matrix
Ainit = [Ax_init;
         Ay_init;
         Axdot_init;
         Aydot_init;
         Atheta_init;
         Athetadot_init
        ];
%           Build b vectors
binit = [x_init;
         y_init;
         xdot_init;
         ydot_init;
         theta_init;
         thetadot_init
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
Atheta_end = zeros(1,namesLength);
Atheta_end(find(names == theta_label(end))) = 1;
Athetadot_end = zeros(1,namesLength);
Athetadot_end(find(names == thetadot_label(end))) = 1;
%               Collect in to an A matrix
Aend = [Ax_end;
        Ay_end;
        Axdot_end;
        Aydot_end;
        Atheta_end;
        Athetadot_end
       ];
%           Build b vectors
bend = [x_end;
        y_end;
        xdot_end;
        ydot_end
        theta_end;
        thetadot_end
       ];
%---------------------------------------------------------------
%===============================================================
%Build Constraints
%===============================================================
%   Collect all Aeq matrices and beq vectors for linear equality
%   constraints
Aeq = [Ainit;Adyn]; 
beq = [binit;bdyn];
%   Add Terminal Conditions into hard constraint
if SoftTerminalConstraint == 0
    Aeq = [Aeq;Aend];
    beq = [beq;bend];
end
%---------------------------------------------------------------
%   Collect all A, b and Type for linear inequality constraints
A = [Acomplementarity;
%     AKinematics
     ];
b = [bcomplementarity;
%     bKinematics
     ];
IneqType = [Typecomplementarity;
%            TypeKinematics
            ];
%------------------------------------------------------------
%   Variable Lower and Upper Boundaries
lb = [repmat(-inf,1, sum(LengthList(find(varList == "x"):find(varList == "FHy")))),repmat(0,1,CFLength + CHLength)];
ub = [repmat( inf,1, sum(LengthList(find(varList == "x"):find(varList == "FHy")))),repmat(1,1,CFLength + CHLength)];
%   Variable Type
vtype = [repmat('C', 1, sum(LengthList(find(varList == "x"):find(varList == "FHy")))), repmat('B',1, CFLength + CHLength)];
%=============================================================
%Call Solvers
%-------------------------------------------------------------
if solver == "knitro"
    %Display Info
    disp("Use Knitro")
    disp(" ")
%-------------------------------------------------------------
    %Set up problem
%-------------------------------------------------------------
        %Build Objective Function
    if SoftTerminalConstraint == 0 %Set Terminal Condition as hard Constraint
        objfunc = @(vars)cost(vars,Q); %!!!!Cost function may need quadrature as well
    elseif SoftTerminalConstraint == 1 %Set Terminal Condition as soft constraint, put into cost function
        objfunc = @(vars)cost_softconstraint(vars,...
                                             Q,...
                                             xIdx_end,...
                                             xdotIdx_end,...
                                             yIdx_end,...
                                             ydotIdx_end,...
                                             thetaIdx_end,...
                                             thetadotIdx_end,...
                                             x_end,...
                                             xdot_end,...   
                                             y_end,...
                                             ydot_end,...
                                             theta_end,...
                                             thetadot_end);
    end

%-------------------------------------------------------------
%       Nonlinear Constraints
    nlcon = @(vars) nlconstraint(vars,...
                                 h,...
                                 I,...
                                 xIdx_init,xIdx_end,...
                                 yIdx_init,yIdx_end,...
                                 thetaIdx_init,thetaIdx_end,...
                                 thetadotIdx_init,thetadotIdx_end,...
                                 PFxIdx_init,PFxIdx_end,...
                                 PFyIdx_init,PFyIdx_end,...
                                 PHxIdx_init,PHxIdx_end,...
                                 PHyIdx_init,PHyIdx_end,...
                                 FFxIdx_init,FFxIdx_end,...
                                 FFyIdx_init,FFyIdx_end,...
                                 FHxIdx_init,FHxIdx_end,...
                                 FHyIdx_init,FHyIdx_end,...
                                 PFcenterX,PFcenterY,...
                                 PHcenterX,PHcenterY,...
                                 BoundingBox_Width,BoundingBox_Height,...
                                 TerrainNorm,miu);
%     This nonlinear constraint function is slightly slower                             
%     nlcon = @(vars)nlconstraint_current_future_vectors(vars,...
%                                                        h,...
%                                                        I,...
%                                                        xIdx_init,xIdx_end,...
%                                                        yIdx_init,yIdx_end,...
%                                                        thetaIdx_init,thetaIdx_end,...
%                                                        xdotIdx_init,xdotIdx_end,...
%                                                        ydotIdx_init,ydotIdx_end,...
%                                                        thetadotIdx_init,thetadotIdx_end,...
%                                                        PFxIdx_init,PFxIdx_end,...
%                                                        PFyIdx_init,PFyIdx_end,...
%                                                        PHxIdx_init,PHxIdx_end,...
%                                                        PHyIdx_init,PHyIdx_end,...
%                                                        FFxIdx_init,FFxIdx_end,...
%                                                        FFyIdx_init,FFyIdx_end,...
%                                                        FHxIdx_init,FHxIdx_end,...
%                                                        FHyIdx_init,FHyIdx_end,...
%                                                        PFcenterX,PFcenterY,...
%                                                        PHcenterX,PHcenterY,...
%                                                        BoundingBox_Width,BoundingBox_Height,...
%                                                        TerrainNorm,miu);
%-------------------------------------------------------------
    %other parameters
    objFnType = 0;
    cFnType = [];
    x0 = rand(namesLength,1);
%-------------------------------------------------------------
    %flip inequality constraints, Knitro only takes < constraints
    A(find(IneqType == 1),:) = -A(find(IneqType == 1),:);
    b(find(IneqType == 1)) = -b(find(IneqType == 1));
%-------------------------------------------------------------
    %Sparsify A matrix and b vectors
    if A_Sparsity == 1
        A = sparse(A);
        b = sparse(b);
        Aeq = sparse(Aeq);
        beq = sparse(beq);
    end
%-------------------------------------------------------------
    %transpose lower and upper bounds, Knitro requires
    lb = lb';
    ub = ub';
%-------------------------------------------------------------
    %reset vType vector, Knitro requires
    vtype = [repmat(0, sum(LengthList(find(varList == "x"):find(varList == "FHy"))), 1); repmat(2, CFLength + CHLength, 1)]; %0 -> continuous variable, 2 -> discrete variable
%-------------------------------------------------------------
    %call knitro
    options = optimset('Display','iter');
    [result.x,result.objval,result.exitflag,optInfo] = knitromatlab_mip(objfunc,x0,A,b,Aeq,beq,lb,ub,nlcon,vtype,objFnType,cFnType,[],options,option_file);
%-------------------------------------------------------------
else
    disp("Unknown solver");
    disp{" "}
end

%Close command line logging
diary off
%%
%========================================================================
%Extract Input Results
%========================================================================
%robot state (Position)
x_result = result.x(find(names == 'x0'):find(names == x_label(end)));
y_result = result.x(find(names == 'y0'):find(names == y_label(end)));
theta_result = result.x(find(names == 'theta0'):find(names == theta_label(end)));

%robot state (Velocity)
xdot_result = result.x(find(names == 'xdot0'):find(names == xdot_label(end)));
ydot_result = result.x(find(names == 'ydot0'):find(names == ydot_label(end)));
thetadot_result = result.x(find(names == 'thetadot0'):find(names == thetadot_label(end)));

%Contact Configuration
CF_result = result.x(find(names == 'CF0'):find(names == CF_label(end)));
CH_result = result.x(find(names == 'CH0'):find(names == CH_label(end)));

%end-effector locations
PFx_result = result.x(find(names == 'PFx0'):find(names == PFx_label(end)));
PFy_result = result.x(find(names == 'PFy0'):find(names == PFy_label(end)));
PHx_result = result.x(find(names == 'PHx0'):find(names == PHx_label(end)));
PHy_result = result.x(find(names == 'PHy0'):find(names == PHy_label(end)));

%end-effector velocities
PFxdot_result = result.x(find(names == 'PFxdot0'):find(names == PFxdot_label(end)));
PFydot_result = result.x(find(names == 'PFydot0'):find(names == PFydot_label(end)));
PHxdot_result = result.x(find(names == 'PHxdot0'):find(names == PHxdot_label(end)));
PHydot_result = result.x(find(names == 'PHydot0'):find(names == PHydot_label(end)));

%contact force result
FFx_result = result.x(find(names == 'FFx0'):find(names == FFx_label(end)));
FFy_result = result.x(find(names == 'FFy0'):find(names == FFy_label(end)));
FHx_result = result.x(find(names == 'FHx0'):find(names == FHx_label(end)));
FHy_result = result.x(find(names == 'FHy0'):find(names == FHy_label(end)));

NetForceX = FFx_result + FHx_result;
NetForceY = FFy_result + FHy_result;

%Torque on the body
FrontTorque_result = (PFx_result - x_result).*FFy_result - (PFy_result - y_result).*FFx_result;
HindTorque_result = (PHx_result - x_result).*FHy_result - (PHy_result - y_result).*FHx_result;

NetTorque = FrontTorque_result + HindTorque_result;

%Foot Bounding Box Result
PFcenterX_result_world = x_result + cos(theta_result)*PFcenterX - sin(theta_result)*PFcenterY;
PFcenterY_result_world = y_result + sin(theta_result)*PFcenterX + cos(theta_result)*PFcenterY;

PHcenterX_result_world = x_result + cos(theta_result)*PHcenterX - sin(theta_result)*PHcenterY;
PHcenterY_result_world = y_result + sin(theta_result)*PHcenterX + cos(theta_result)*PHcenterY;
