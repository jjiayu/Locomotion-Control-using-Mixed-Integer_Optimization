% Mixed-integer Nonlinear Optimization in 2D case, use auto differentiation
% to compute gradients and hessians
% Mode Selection at every time step
% Foot-ground contact configurations are modeled as 0/1 for each foot

% Check readme for notes ad future improvements

clear;
clc;

%========================================================
% Import CasADi related packages
import casadi.*

%========================================================
% Command Line Logging
diary off
diary_filename = strcat('log-', datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
diary(diary_filename);

%=========================================================
% Display Script Information
disp('CasADi Implementation');
disp('2D Locomotion Control using Mixed-integer Nonlinear Optimization');
disp('Mode Configuration Change per Time Step');
disp(' ');
disp('Date and Time:');
disp(datetime('now'));
disp(['Correspondent Log File Name: ', diary_filename]);
disp('---------------------------------------------------');
%=====================================================================
% Add path
%addpath('/home/jiayu/bin/casadi-linux-matlabR2014b-v3.4.5')

%=====================================================================
% Choose solver

SolverNum = input('Solver Selection, 1: Knitro, 2: Bonmin \n');
if SolverNum == 1
    SolverSelected = 'knitro';
elseif SolverNum == 2
    SolverSelected = 'bonmin';
else
    ME_SelectSolvers = MException('Initialization:SelectSolvers','Unknown Solver Nominated');
    throw(ME_SelectSolvers)
end
disp(['Selected Solver: ', SolverSelected])
disp('---------------------------------------------------')
%======================================================================
%Inertia Parameters(Information from MIT Cheetah 3)
m = 45; %kg
I = 2.1; %kg m^2 Izz
g = 9.80665; %m/s^2
%======================================================================

%======================================================================
%Environment Information
TerrainHeight = 0; %terrain height
TerrainNorm = [0,1];
miu = 0.6; %friction coefficient
disp('Environment Information: ');
disp('Friction Cone: ');
disp(num2str(miu));
disp('---------------------------------------------------');
%======================================================================

%======================================================================
% Time Step and Discretization Parameter Settings
%   Terminal time
Tend = input('Input Termianl Time (e.g. 1s): \n');
%   Number of time steps
NumTimeSteps = input('Input Number of Time Steps (10, 20, 30, 40): \n');
%   Time Step Length
h = Tend/NumTimeSteps;
disp('Time Step Length: ');
disp([num2str(h),' (s)']);
%   Time Series Generation
TimeSeries = 0:h:Tend;
TimeSeriesLength = length(TimeSeries);
disp(['Number of Knots/Discretizations (From Time ',num2str(TimeSeries(1)),' to ', num2str(TimeSeries(end)), '):'])
disp([num2str(TimeSeriesLength), ' (NumTimeSteps + 1)']);
disp('---------------------------------------------------')
%======================================================================

%======================================================================
% Parameter Setting
%======================================================================
%   Big-M Parameters for Complementarity Constraints
%----------------------------------------------------------------------
Mpos_y = 100; %(meters) big-M for foot positions in y-axis
Mvel = 5; %(m/s) big-M for foot velocity in both x and y axis
Mfx = 1e5; %(N) big-M for foot-ground reaction forces for x-axis
Mfy = 1e5; %(N) big-M for foot-ground reaction forces for y-axis
%----------------------------------------------------------------------
%   Kinematics Constraint Parameters
%----------------------------------------------------------------------
%       Body Size
BodyLength = 0.6;
BodyHeight = 0.2;
%       Default foot position in Local robot frame
DefaultLegLength = 0.45; %default leg length , distance from the default Leg Y to Torso (LOWER BORDER of the TORSO)
%           Front Foot
PFcenterX = 1/2*BodyLength;
PFcenterY = -(1/2*BodyHeight + DefaultLegLength);
%           Hind Foot
PHCenterX = -1/2*BodyLength;
PHCenterY = -(1/2*BodyHeight + DefaultLegLength);
%       Kinematics Bounding Box Constraint
BoundingBox_Width = 0.3;
BoundingBox_Height = 0.3;
%======================================================================

%======================================================================
% Task Specifications
%======================================================================
%   Initial Conditions
%----------------------------------------------------------------------
x_Init = 0;
y_Init = 1/2*BodyHeight + DefaultLegLength;
thetaInit = 0;
xdot_Init = 0;
ydot_Init = 0;
thetadot_Init = 0;
%----------------------------------------------------------------------
%   Terminal Conditions
x_End = input('Input Travel Distance along x-axis (m): \n' );
y_End = 1/2*BodyHeight + DefaultLegLength;
theta_End = 0;
xdot_End = 0;
ydot_End = 0;
thetadot_End = 0;
%----------------------------------------------------------------------
%   Test if initial and termianl conditions violates kinematics constraint
%   Robot Height should set in a way that highest foot posiitons are not
%   undre the terrain height, otherwise conflicting with complementarity
%   constraints
if (y_Init - 1/2*BodyHeight - DefaultLegLength + 1/2*BoundingBox_Height) < 0
    ME_InitHeight = MException('Initialization:ProblematicInitialHeight','Initial Height Error (y_init), Increase Initial Height');
    throw(ME_InitHeight)
end

if (y_End - 1/2*BodyHeight - DefaultLegLength + 1/2*BoundingBox_Height) < 0
    ME_TerminalHeight = MException('Initialization:ProblematicTerminalHeight','Terminal Height Error (y_end), Increase Terminal Height');
    throw(ME_TerminalHeight)
end
disp('---------------------------------------------------')
%=======================================================================
% (Place Holder): Setup of Soft constraints on Terminal Condition or not
%=======================================================================

%=======================================================================
% Define/Create Modeling Variables
%=======================================================================
%   Display some Info
disp('Generating Decision Variables Vectors/Lists')
%-----------------------------------------------------------------------
%   Continuous Variables
%-----------------------------------------------------------------------
%   State Variables: r = [x, y, theta, xdot, ydot, thetadot]
%       x: horizontal position
%       y: vertical position
%       theta: torso orientation
%       xdot: horizontal velocity
%       ydot: vertical velocity
%       thetadot: torso angular velocity
%   Foot Step Locations (IN WORLD FRAME):
%       Front Leg Location and Velocities: PF = [PFx, PFy, PFxdot, PFydot]
%       Hind Leg Location and Velocities : PH = [PHx, PHy, PHxdot, PHydot]
%   Foot-ground Reaction Forces (IN WORLD FRAME): 
%       Front Leg Forces: FF = [FFx, FFy]
%       Hind Leg Forces : FH = [FHx, FHy]
%-----------------------------------------------------------------------
%   Create CasADi SX variable Lists
x = SX.sym('x', TimeSeriesLength); 
y = SX.sym('y', TimeSeriesLength); 
theta = SX.sym('theta', TimeSeriesLength);
xdot = SX.sym('xdot', TimeSeriesLength); 
ydot = SX.sym('ydot', TimeSeriesLength); 
thetadot = SX.sym('thetadot', TimeSeriesLength);
%           FootStep Locations and Velocities
%               Front Leg Locations
PFx = SX.sym('PFx', TimeSeriesLength);
PFy = SX.sym('PFy', TimeSeriesLength);
%               Front Leg Velocities
PFxdot = SX.sym('PFxdot', TimeSeriesLength); 
PFydot = SX.sym('PFydot', TimeSeriesLength);
%               Hind Leg Locations
PHx = SX.sym('PHx', TimeSeriesLength);
PHy = SX.sym('PHy', TimeSeriesLength);
%               Hind Leg Velocities
PHxdot = SX.sym('PHxdot', TimeSeriesLength);
PHydot = SX.sym('PHydot', TimeSeriesLength);
%           Foot-Ground Reaction Forces
%               Front Leg Forces
FFx = SX.sym('FFx', TimeSeriesLength);
FFy = SX.sym('FFy', TimeSeriesLength);
%               Hind Leg Forces
FHx = SX.sym('FHx', TimeSeriesLength);
FHy = SX.sym('FHy', TimeSeriesLength);
%-----------------------------------------------------------------------
%       Create Decision Variable Name Lists 
%           Using CreateVarsNameList(varsCasADi)
%           varsCasADi is the CasADi variables
%-----------------------------------------------------------------------
x_label      = CreateVarsNameList(x);       y_label      = CreateVarsNameList(y);       theta_label    = CreateVarsNameList(theta);
xdot_label   = CreateVarsNameList(xdot);    ydot_label   = CreateVarsNameList(ydot);    thetadot_label = CreateVarsNameList(thetadot);
PFx_label    = CreateVarsNameList(PFx);     PFy_label    = CreateVarsNameList(PFy);    
PFxdot_label = CreateVarsNameList(PFxdot);  PFydot_label = CreateVarsNameList(PFydot);
PHx_label    = CreateVarsNameList(PHx);     PHy_label    = CreateVarsNameList(PHy);    
PHxdot_label = CreateVarsNameList(PHxdot);  PHydot_label = CreateVarsNameList(PHydot);
FFx_label    = CreateVarsNameList(FFx);     FFy_label    = CreateVarsNameList(FFy);
FHx_label    = CreateVarsNameList(FHx);     FHy_label    = CreateVarsNameList(FHy);
%-----------------------------------------------------------------------
%   Discrete Variables --> Contact Configuration
%-----------------------------------------------------------------------
%       Contact Configuration (Mode)
%           Leg Contact Configurations: C = [CF, CH]; CF, CH = 0/1 (Binary Variable)
%               Front Leg Contact On/Off: CF
%               Hind Leg Contact On/Off : CH
%-----------------------------------------------------------------------
%       Create CasADi SX variables
%--------------------------------------------
%       (Place Holder) Need to re-create to the same size with respect to
%       number of phases, when moving to phase-based foprmulation
%       (Place Holder) Futher, when move to phase-based formulation, first
%       create contact configuration from 0 to number of phases, and
%       extract the sub variable list from 1 to number of phases
%--------------------------------------------
CF = SX.sym('CF', TimeSeriesLength);
CH = SX.sym('CH', TimeSeriesLength);
%       Create Variable Name List
CF_label = CreateVarsNameList(CF);
CH_label = CreateVarsNameList(CH);
%-----------------------------------------------------------------------
%   Assemble Lists for all decision variables
%       Identifiers of all variables
varList = ["x",    "y",      "theta",...
           "xdot", "ydot",   "thetadot",...
           "PFx",  "PFy",    "PFxdot",      "PFydot",...
           "PHx",  "PHy",    "PHxdot",      "PHydot",...
           "FFx",  "FFy",...
           "FHx",  "FHy",...
           "CF",   "CH"];
%       Variable Length List
VarLengthList = [length(x_label),     length(y_label),      length(theta_label), ...
                 length(xdot_label),  length(ydot_label),   length(thetadot_label), ...
                 length(PFx_label),   length(PFy_label),    length(PFxdot_label),       length(PFydot_label), ...
                 length(PHx_label),   length(PHy_label),    length(PHxdot_label),       length(PHydot_label), ...
                 length(FFx_label),   length(FFy_label),...
                 length(FHx_label),   length(FHy_label),...
                 length(CF_label),    length(CH_label)];

%       Full decision variable names list
VarNamesList = [x_label,      y_label,      theta_label,...
                xdot_label,   ydot_label,   thetadot_label,...
                PFx_label,    PFy_label,    PFxdot_label,      PFydot_label,...
                PHx_label,    PHy_label,    PHxdot_label,      PHydot_label,...
                FFx_label,    FFy_label,...
                FHx_label,    FHy_label,...
                CF_label,     CH_label];
%----------------------------------------------------------------------
%   Verifications
%       (Place Holder - Not Required) Check if variable names are defined
disp('-----------------------------------------------------------')
%=======================================================================

%=======================================================================
% Symbolic Functions
%=======================================================================
%   Euler Integration: x_k+1 - x_k = h*u_k -->
%                      x_k+1 - x_k - h*u_k = 0
%-----------------------------------------------------------------------
%       Variable Definitions:
xk  = SX.sym('x[k]'); %xk, current state
xkk = SX.sym(['x[k+',num2str(1),']']); %xk+1 next state
uk  = SX.sym('u[k]'); %uk, control input at current time step
hk  = SX.sym('h[k]'); %hk, current time step length
%       Build Function
euler = xkk - xk - hk*uk; %expression
EulerIntegration = Function('Euler',{xkk,xk,hk,uk},{euler}); %function, can take either symbolic or numerical inputs
%-----------------------------------------------------------------------
%   Complementarity Constraint (a Set of Inequality Equations):
%-----------------------------------------------------------------------
%       Variable Definitions:
vk   = SX.sym('v[k]'); %vk, continuous decision variable
zk   = SX.sym('z[k]'); %zk, integer variable
bigM = SX.sym('bigM'); % big-M
%       Build Functions
%           Type 1
%           Difference --> v[k] - bigM*z[k] >=/<= Constant
ineq_diff = vk - bigM*zk;
Ineq_Difference = Function('Ineq_Difference', {vk, bigM, zk}, {ineq_diff});
%           Type 2
%           Summation --> v[k] + bigM*z[k] >=/<= Constant
ineq_sum  = vk + bigM*zk;
Ineq_Summation  = Function('Ineq_Summation',  {vk, bigM, zk}, {ineq_sum});
%-----------------------------------------------------------------------
%   Kinematics Constraint -->
%          -1/2*b (Constant Vector) <= R(theta[k])(P[k]-r[k])- Pcenter <= 1/2*b (Constant Vector)
%                       b = [bw;bh];                                                                                                                                                                                                         
%--------------------------
%       (Place Holder) Need to Augment when moving to 3D
%--------------------------
%       Variable Definitions
xk     = SX.sym('x[k]');
yk     = SX.sym('y[k]');
thetak = SX.sym('theta[k]');
rk     = [xk,yk,thetak];
PcX    = SX.sym('PcenterX'); %Foot/End-Effector default x-axis position
PcY    = SX.sym('PcenterY'); %Foot/End-Effector default y-axis position
Pc     = [PcX,PcY]; %Foot/End-Effector default positions
Pxk    = SX.sym('Px[k]');
Pyk    = SX.sym('Py[k]');
Pk     = [Pxk,Pyk];
%       Build Function
kinematics = [cos(thetak), -sin(thetak); sin(thetak), cos(thetak)]'*(Pk' - [xk,yk]') - Pc';
KinematicsConstraint = Function('KinematicsConstraint',{rk, Pk, Pc},{kinematics});
%-----------------------------------------------------------------------
%   Friction Cones (In 2D, Flat Terrain) -->
%               Fx[k] - Const_miu*(Norm*[Fx[k],Fy[k]]') <= 0
%--------------------------------
%   (Place Holder) Need to Change when move to 3D Case and/or Uneven
%   Terrains
%-------------------------------
%       Variable Definitions
Const_miu = SX.sym('miu');
NormX     = SX.sym('Nx[k]');
NormY     = SX.sym('Ny[k]');
Norm      = [NormX, NormY];
ForceX    = SX.sym('Fx[k]');
ForceY    = SX.sym('Fy[k]');
Force     = [ForceX, ForceY];
%       Build Functions
friction = ForceX - Const_miu*(dot(Norm,Force));
FrictionCone = Function('FrictionCone',{Norm, Force, Const_miu},{friction});
%-----------------------------------------------------------------------

%=======================================================================
% Build Constraints and Objective Function
%=======================================================================   
%   (ToDo) Initialize Constraints and Cost Function Constainers
%-----------------------------------------------------------------------
%   (ToDo) Generate COnstraints and Cost Function
for k = 1:TimeSeriesLength
    
    % Robot Torso dynamics
    %x-axis first-order dynamics (position)
    %x-axis second-order dynamics (velocity)
    %y-axis first-order dynamics (position)
    %y-axis second-order dynamics (velocity)
    %theta first-order dynamics (position)
    %theta second-order dynamics (velocity)
    
    % Footstep Location dynamics
    % PFx (Front foot x-axis) first-order dynamics (velocity)
    % PFy (Front foot y-axis) first-order dynamics (velocity)
    % PHx (Hind foot x-axis) first-order dynamics (velocity)
    % PHy (Hind foot y-axis) first-order dynamics (velocity)
    
    % Complementarity Constraint
    
    % Kinematics Constraint
    
    % Friction Cone
    
    % Cost Function
    
end
%=======================================================================

%=======================================================================
% (ToDo) Solve the Problem
%=======================================================================

%=======================================================================
% (ToDo) Extract the Solution
%=======================================================================

%=======================================================================
% Close Diary
diary off