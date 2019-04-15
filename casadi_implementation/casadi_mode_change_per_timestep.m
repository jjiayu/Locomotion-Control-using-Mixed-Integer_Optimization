% Mixed-integer Nonlinear Optimization in 2D case, use auto differentiation
% to compute gradients and hessians
% Mode Selection at every time step
% Foot-ground contact configurations are modeled as 0/1 for each foot

% Check readme for notes ad future improvements

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
disp(['Correspondent Log File Name: ', diary_filename]);
disp(' ');
disp('2D Locomotion Control using Mixed-integer Nonlinear Optimization')
disp('Mode Configuration Change per Time Step')
disp(' ')

%=====================================================================
% Add path
%addpath('/home/jiayu/bin/casadi-linux-matlabR2014b-v3.4.5')

%=====================================================================
% Choose solver

SolverNum = input('Select Solver First, 1: Knitro, 2: Bonmin \n');
if SolverNum == 1
    SolverSelected = 'knitro';
elseif SolverNum == 2
    SolverSelected = 'bonmin';
else
    ME_SelectSolvers = MException('Initialization:SelectSolvers','Unknown Solver Nominated');
    throw(ME_SelectSolvers)
end
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
%======================================================================

%======================================================================
% Time Step and Discretization Parameter Settings
%   Terminal time
Tend = input('Input Termianl Time (e.g. 1s): \n');
disp(' ');
%   Number of time steps
NumTimeSteps = input('Input Number of Time Steps (10, 20, 30, 40): \n');
disp(' ');
%   Time Step Length
h = Tend/NumTimeSteps;
disp(['Time Step Length: ',num2str(h),' (s)']);

%   Time Series Generation
TimeSeries = 0:h:Tend;
TimeSeriesLength = length(TimeSeries);
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
x_End = input('Input Travel Distance along x-axis (m) :' );
disp(' ');
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

disp(' ')
%=======================================================================

%=======================================================================
% Close Diary
diary off