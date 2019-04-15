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
diary_filename = strcat('log-', datestr(datetime('now'),30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
diary(diary_filename);

%=========================================================
% Display Script Information
disp('CasADi Implementation');
disp('2D Locomotion Control using Mixed-integer Nonlinear Optimization')
disp('Mode Configuration Change per Time Step')
disp(' ')
disp('Date and Time:');
disp(datetime('now'));
disp(['Correspondent Log File Name: ', diary_filename]);
disp('---------------------------------------------------')
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
disp('Environment Information: ')
disp('Friction Cone: ')
disp(num2str(miu))
disp('---------------------------------------------------')
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
Mpos_y = 50; %(meters) big-M for foot positions in y-axis
Mvel = 10; %(m/s) big-M for foot velocity in both x and y axis
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
% Place Holder: Setup of Soft constraints on Terminal Condition or not
%=======================================================================

%=======================================================================
% Define/Create Modeling Variables
%=======================================================================
%   Display some Info
disp('Generating Decision Variables Vectors/Lists')
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
%       Create CasADi SX variable Lists
%           State Variables
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
%           Create Empty Decision Variable Name Lists
x_label = strings(1,TimeSeriesLength);      y_label = strings(1,TimeSeriesLength);    theta_label = strings(1,TimeSeriesLength);
xdot_label = strings(1,TimeSeriesLength);   ydot_label = strings(1,TimeSeriesLength); thetadot_label = strings(1,TimeSeriesLength);
PFx_label = strings(1,TimeSeriesLength);    PFy_label = strings(1,TimeSeriesLength);
PFxdot_label = strings(1,TimeSeriesLength); PFydot_label = strings(1,TimeSeriesLength);
PHx_label = strings(1,TimeSeriesLength);    PHy_label = strings(1,TimeSeriesLength);
PHxdot_label = strings(1,TimeSeriesLength); PHydot_label = strings(1,TimeSeriesLength);
FFx_label = strings(1,TimeSeriesLength);    FFy_label = strings(1,TimeSeriesLength);
FHx_label = strings(1,TimeSeriesLength);    FHy_label = strings(1,TimeSeriesLength);
%           Assign labels
for i = 1:TimeSeriesLength
    x_label(i) = strcat('x_',num2str(i-1));               y_label(i) = strcat('y_',num2str(i-1));              theta_label(i) = strcat('theta_',num2str(i-1));
    xdot_label(i) = strcat('xdot_', num2str(i-1));        ydot_label(i) = strcat('ydot_', num2str(i-1));       thetadot_label(i) = strcat('thetadot_', num2str(i-1));
    PFx_label(i) = strcat('PFx_', num2str(i-1));          PFy_label(i) = strcat('PFy_',num2str(i-1));
    PFxdot_label(i) = strcat('PFxdot_', num2str(i-1));    PFydot_label(i) = strcat('PFydot_', num2str(i-1));
    PHx_label(i) = strcat('PHx_', num2str(i-1));          PHy_label(i) = strcat('PHy_', num2str(i-1));
    PHxdot_label(i) = strcat('PHxdot_', num2str(i-1));    PHydot_label(i) = strcat('PHydot_', num2str(i-1));
    FFx_label(i) = strcat('FFx_', num2str(i-1));          FFy_label(i) = strcat('FFy_', num2str(i-1));
    FHx_label(i) = strcat('FHx_', num2str(i-1));          FHy_label(i) = strcat('FHy_', num2str(i-1));
end
%-----------------------------------------------------------------------
%       Check length of variables and lists
%           CasADi type state variable length list
VarsLengthList = [x.length(),    y.length(),    theta.length(), ...
                  xdot.length(), ydot.length(), thetadot.length(),...
                  PFx.length(),  PFy.length(),  PFxdot.length(),  PFydot.length(),...
                  PHx.length(),  PHy.length(),  PHxdot.length(),  PHydot.length(),...
                  FFx.length(),  FFy.length(),...
                  FHx.length(),  FHy.length()]; 
%           Variable Name List
NamesLengthList = [length(x_label),    length(y_label),    length(theta_label), ...
                   length(xdot_label), length(ydot_label), length(thetadot_label), ...
                   length(PFx_label),  length(PFy_label),  length(PFxdot_label),      length(PFydot_label), ...
                   length(PHx_label),  length(PHy_label),  length(PHxdot_label),      length(PHydot_label), ...
                   length(FFx_label),  length(FFy_label),...
                   length(FHx_label),  length(FHy_label)];
%-----------------------------------------------------------------------               
%           Check
if (sum(VarsLengthList - NamesLengthList) == 0)
    disp('Checked: CasADi Type Decision Variables and Variable Name Lists (i.e. state variables, footstep locations and velocities and foot-ground reactions forces) are defined on Every Knots/Discretization')
else
    ME_StateVariableSize = MException('Initialization:WrongSizeofStateVariables','Size of the Variable Name Lists does not match the Size of CasADi Type Decision Variables');
    throw(ME_StateVariableSize)
end
%-----------------------------------------------------------------------
%   Contact Configuration (Mode)
%       Leg Contact Configurations: C = [CF, CH]; CF, CH = 0/1 (Binary Variable)
%           Front Leg Contact On/Off: CF
%           Hind Leg Contact On/Off : CH
%       Create CasADi SX variables
CF = SX.sym('CF', TimeSeriesLength);
CH = SX.sym('CH', TimeSeriesLength);
%       Create Variable Name List
CF_label = strings(1,TimeSeriesLength);
CH_label = strings(1,TimeSeriesLength);
for i = 1:TimeSeriesLength
    CF_label(i) = strcat('CF_', num2str(i-1));
    CH_label(i) = strcat('CH_', num2str(i-1));
end
CFLength = length(CF_label);
CHLength = length(CH_label);
%-----------------------------------------------------------------------
disp('-----------------------------------------------------------')

%=======================================================================
% Close Diary
diary off