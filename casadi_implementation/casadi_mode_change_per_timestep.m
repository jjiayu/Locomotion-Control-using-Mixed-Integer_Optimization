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
G = 9.80665; %m/s^2
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
disp('Time Step Size: ');
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
%           Front Foot Default Positions (IN ROBOT FRAME)
PFcenterX = 1/2*BodyLength;
PFcenterY = -(1/2*BodyHeight + DefaultLegLength);
%           Hind Foot Default Positions (IN ROBOT FRAME)
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
x_Init         = 0;
y_Init         = 1/2*BodyHeight + DefaultLegLength;
theta_Init     = 0;
xdot_Init      = 0;
ydot_Init      = 0;
thetadot_Init  = 0;
%----------------------------------------------------------------------
%   Terminal Conditions
x_End        = input('Input Travel Distance along x-axis (m): \n' );
y_End        = 1/2*BodyHeight + DefaultLegLength;
theta_End    = 0;
xdot_End     = 0;
ydot_End     = 0;
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
%   Euler Integration: x_k+1 - x_k = h*fdyn_k -->
%                      x_k+1 - x_k - h*fdyn_k = 0
%-----------------------------------------------------------------------
%       Variable Definitions:
xk  = SX.sym('x[k]');                  %xk, current state
xkk = SX.sym(['x[k+',num2str(1),']']); %xk+1 next state
fdyn_k  = SX.sym('fdyn[k]');               %fdyn_k, dynamical equation value evaluated at time step k
hk  = SX.sym('h[k]');                  %hk, current time step length
%       Build Function
euler = xkk - xk - hk*fdyn_k; %expression
EulerIntegration = Function('Euler',{xkk,xk,hk,fdyn_k},{euler}); %function, can take either symbolic or numerical inputs
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
%   Terrains (change equation and norm)
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
%   Initialize Constraints and Cost Function Constainers
%-----------------------------------------------------------------------
%       Collect all decision varibales
DecisionVars = {x,          y,          theta,...
                xdot,       ydot,       thetadot,...
                PFx,        PFy,        PFxdot,         PFydot,...
                PHx,        PHy,        PHxdot,         PHydot,...
                FFx,        FFy,...
                FHx,        FHy,...
                CF,         CH}; 
DecisionVars = vertcat(DecisionVars{:}); %make a vertical vector

%       Verify if the variable names are consistent in DecisionVars and VarNameList 
if length(DecisionVars) == length(VarNamesList)
    for i = 1:length(DecisionVars)
        temp = DecisionVars(i);
        if temp.name() ~= VarNamesList(i)
            ME_NameListInconsistent = MException('Initialization:InconsistentNameList',['The variable names in DecisionVars and VarNamesList are different at index ', num2str(i)]);
            throw(ME_NameListInconsistent)
        end
    end
    disp('Checked - Names in DecisionVars are consistent with the name labels in VarNamesList');
else
    ME_NameListLengthInconsistent= MException('Initialization:NameListInconsistent','DecisionVars has different size with respect to VarNamesList');
    throw(ME_NameListLengthInconsistent)
end

%       Other Important Variables
%           Initial Guess of Decision Variables
DecisionVarsInit = rand(size(DecisionVars));
%           Lower and upper bounds, variable type
lb_DecisionVars = [];  %Variable Lower Bound 
ub_DecisionVars = [];  %Variable Upper Bound
varstype        = [];  %Variable Type: 0 -> Continuous Variable/ 1 -> Binary Variable
for i = 1:length(varList)
    if contains(varList(i),'C') == 1 %binary variable
        lb_DecisionVars = [lb_DecisionVars, zeros(1, VarLengthList(i))];    
        ub_DecisionVars = [ub_DecisionVars, ones( 1, VarLengthList(i))];
        varstype        = [varstype,        ones(1, VarLengthList(i))];            %0 -> Continuous Variable/ 1 -> Binary Variable
    else %continuous variables
        lb_DecisionVars = [lb_DecisionVars, repmat(-inf, 1, VarLengthList(i))];    
        ub_DecisionVars = [ub_DecisionVars, repmat( inf, 1, VarLengthList(i))];
        varstype        = [varstype,        zeros(1, VarLengthList(i))];           %0 -> Continuous Variable/ 1 -> Binary Variable
    end
end
%------------------------
%           (Place Holder) Check the size of lower and upper bounds and
%           vartype with respect the DecisionVars
%------------------------
%           container of constraints
g   = {}; 
%           Upper Bound of Constraint Functions
lbg = []; 
%           Lower Bound of Constraint Functions
ubg = []; 
%           Cost Function
J   = 0;  
%-----------------------------------------------------------------------
% Generate Constraints and Cost Function
%-----------------------------------------------------------------------
for k = 1:TimeSeriesLength
    %--------------------------------------
    % System dynamics
    %   Use EulerIntegration Function to Construct
    %   EulerIntegration = x[k+1] - x[k] -h[k]*fdyn[k] = 0
    %   Input: x[k+1], x[k], h[k], fdyn[k] (Display sequence as defined in EulerIntegration Function)
    %   lbg = 0 (Equality constraint)
    %   ubg = 0 (Equality constraint)
    %   Covering Range: from k = 1 to k = TimeSeriesLength - 1
    %--------------------------------------
    %   Constraint the range
    %--------------------------------------
    if k <= TimeSeriesLength - 1
        %----------------------------------
        % Robot Torso Dynamics
        %--------------------------------------
        % (*) x-axis first-order dynamics (position)
        %       Equation: x[k+1] - x[k] - h*xdot[k] = 0
        %       Input: x[k+1]  = x[k+1] (x[k+1] at rhs means x-position)
        %              x[k]    = x[k]   (x[k] at rhs means x-position)
        %              h[k]    = h
        %              fdyn[k] = xodt[k]
        %       lbg = 0
        %       ubg = 0
        EqTemp = EulerIntegration(x(k+1), x(k), h, xdot(k));
        g   = {g{:}, EqTemp};  %Append to constraint function list
        lbg = [lbg;  0];       %Give constraint lower bound
        ubg = [ubg;  0];       %Give constraint upper bound
        %----------------------------------------
        %(*) x-axis second-order dynamics (velocity)
        %       Equation: xdot[k+1] - xdot[k] - h*(1/m*FFx[k] + 1/m*FHx[k])
        %       Input: x[k+1]  = xdot[k+1]
        %              x[k]    = xdot[k]
        %              h[k]    = h
        %              fdyn[k] = 1/m*(FFx[k]+FHx[k])
        %       lbg = 0;
        %       ubg = 0;
        fdyntemp  = 1/m*(FFx(k)+FHx(k)); 
        EqTemp    = EulerIntegration(xdot(k+1), xdot(k), h, fdyntemp);
        g   = {g{:}, EqTemp};  %Append to constraint function list
        lbg = [lbg;  0];       %Give constraint lower bound
        ubg = [ubg;  0];       %Give constraint upper bound
        %-----------------------------------------
        % (*) y-axis first-order dynamics (position)
        %       Equation: y[k+1] - y[k] - h[k]*ydot[k] = 0
        %       Input: x[k+1]  = y[k+1]
        %              x[k]    = y[k]
        %              h[k]    = h
        %              fdyn[k] = ydot[k]
        EqTemp = EulerIntegration(y(k+1), y(k), h, ydot(k));
        g   = {g{:}, EqTemp};  %Append to constraint function list
        lbg = [lbg;  0];       %Give constraint lower bound
        ubg = [ubg;  0];       %Give constraint upper bound
        %-----------------------------------------
        % (*) y-axis second-order dynamics (velocity)
        %       Equation: ydot[k+1] - ydot[k] - h[k]*[1/m*(FFy[k]+FHy[k]-g)] = 0
        %       Input: x[k+1]  = ydot[k+1]
        %              x[k]    = ydot[k]
        %              h[k]    = h
        %              fdyn[k] = 1/m*(FFy[k] + FHy[k] - g)
        %       lbg = 0
        %       ubg = 0
        fdyntemp = 1/m*(FFy(k) + FHy(k)) - G;
        EqTemp   = EulerIntegration(ydot(k+1), ydot(k), h, fdyntemp);
        g   = {g{:}, EqTemp};  %Append to constraint function list
        lbg = [lbg;  0];       %Give constraint lower bound
        ubg = [ubg;  0];       %Give constraint upper bound
        %-----------------------------------------
        % (*) theta first-order dynamics (position)
        %       Equation: theta[k+1] - theta[k] - h[k]*thetadot[k] = 0
        %       Input: x[k+1]  = theta[k+1]
        %              x[k]    = theta[k]
        %              h[k]    = h
        %              fdyn[k] = thetadot[k]
        %       lbg = 0
        %       ubg = 0
        EqTemp = EulerIntegration(theta(k+1), theta(k), h, thetadot(k));
        g   = {g{:}, EqTemp};   %Append to constraint function list
        lbg = [lbg;  0];        %Give constraint lower bound
        ubg = [ubg;  0];        %Give constraint upper bound
        %-----------------------------------------
        % (*) theta second-order dynamics (velocity)
        %       Equation: Izz*thetadot[k+1] - Izz*thetadot[k] - h[k]*[(PFx[k] - x[k])*FFy[k] - (PFy[k] - y[k])*FFx[k] + (PHx[k] - x[k])*FHy[k] - (PHy[k] - y[k])*FHx[k]]
        %       Input: x[k+1]  = Izz*thetadot[k+1]
        %              x[k]    = Izz*thetadot[k]
        %              h[k]    = h
        %              fdyn[k] = (PFx[k] - x[k])*FFy[k] - (PFy[k] - y[k])*FFx[k] + (PHx[k] - x[k])*FHy[k] - (PHy[k] - y[k])*FHx[k]
        %       lbg = 0
        %       ubg = 0
        fdyntemp = (PFx(k) - x(k))*FFy(k) - (PFy(k) - y(k))*FFx(k) + (PHx(k) - x(k))*FHy(k) - (PHy(k) - y(k))*FHx(k);
        EqTemp   = EulerIntegration(I*thetadot(k+1), I*thetadot(k), h, fdyntemp);
        g   = {g{:}, EqTemp};  %Append to constraint function list
        lbg = [lbg;  0];       %Give constraint lower bound
        ubg = [ubg;  0];       %Give constraint upper bound
        %-----------------------------------------
        % Footstep Location dynamics
        %-----------------------------------------
        % (*) PFx (Front foot x-axis) first-order dynamics (velocity)
        %       Equation: PFx[k+1] - PFx[k] - h[k]*PFxdot[k] = 0
        %       Input: x[k+1]  = PFx[k+1]
        %              x[k]    = PFx[k]
        %              h[k]    = h
        %              fdyn[k] = PFxdot[k]
        %       lbg = 0
        %       ubg = 0
        EqTemp = EulerIntegration(PFx(k+1), PFx(k), h, PFxdot(k));
        g   = {g{:}, EqTemp};  %Append to constraint function list
        lbg = [lbg;  0];       %Give constraint lower bound
        ubg = [ubg;  0];       %Give constraint upper bound
        %-----------------------------------------
        % (*) PFy (Front foot y-axis) first-order dynamics (velocity)
        %       Euqation: PFy[k+1] - PFy[k] - h[k]*PFydot[k] = 0
        %       Input: x[k+1]  = PFy[k+1]
        %              x[k]    = PFy[k]
        %              h[k]    = h
        %              fdyn[k] = PFydot[k]
        %       lbg = 0;
        %       ubg = 0;
        EqTemp = EulerIntegration(PFy(k+1), PFy(k), h, PFydot(k));
        g   = {g{:}, EqTemp};  %Append to constraint function list
        lbg = [lbg;  0];       %Give constraint lower bound
        ubg = [ubg;  0];       %Give constraint upper bound
        %-----------------------------------------
        % (*) PHx (Hind foot x-axis) first-order dynamics (velocity)
        %       Equation: PHx[k+1] - PHx[k] - h[k]*PHxdot[k] = 0
        %       Input: x[k+1]  = PHx[k+1]
        %              x[k]    = PHx[k]
        %              h[k]    = h
        %              fdyn[k] = PHxdot[k]
        %       lbg = 0;
        %       ubg = 0;
        EqTemp = EulerIntegration(PHx(k+1), PHx(k), h, PHxdot(k));
        g   = {g{:}, EqTemp};  %Append to constraint function list
        lbg = [lbg;  0];       %Give constraint lower bound
        ubg = [ubg;  0];       %Give constraint upper bound
        %-----------------------------------------
        % (*) PHy (Hind foot y-axis) first-order dynamics (velocity)
        %       Equation: PHy[k+1] - PHy[k] - h[k]*PHydot[k] = 0
        %       Input: x[k+1]  = PHy[k+1]
        %              x[k]    = PHy[k]
        %              h[k]    = h
        %              fdyn[k] = PHydot[k]
        %       lbg = 0
        %       ubg = 0
        EqTemp = EulerIntegration(PHy(k+1), PHy(k), h, PHydot(k));
        g   = {g{:}, EqTemp};  %Append to constraint function list
        lbg = [lbg;  0];       %Give constraint lower bound
        ubg = [ubg;  0];       %Give constraint upper bound
        %-----------------------------------------
    end
    %   Dynamical Equation Constraint Setup - Done
    %----------------------------------------------------
    % Complementarity Constraint
    %   Use Functions: Ineq_Difference --> v[k] - bigM*z[k]
    %                  Ineq_Summation  --> v[k] + bigM*z[k]
    %   Input: v[k] --> Continuous Variables at time step k
    %          bigM --> bigM constant
    %          z[k] --> Integer/Binary Variable at time step k
    %----------------------------------------------------
    %   (*) Foot/End-effector Position (y-axis only)
    %---------------------------------------------------
    %       (Place Holder) Need to Change Height into loop-up table
    %       function ifr we go for uneven terrain
    %--------------------------------------------------
    %       - Equation (1): Py <= Height + Mpos(1-C) -->
    %                       Py + Mpos*C <= Height + Mpos
    %         Use Ineq_Summation
    %         Input: v[k] = P(F/H)y[k]
    %                bigM = Mpos_y
    %                z[k] = C(F/H)[k]
    %         lbg = -inf 
    %         ubg = Height + Mpos_y
    %-------------------------------------------
    %           Front Leg
        EqTemp = Ineq_Summation(PFy(k),Mpos_y, CF(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  TerrainHeight + Mpos_y];      %Give constraint upper bound
        
    %           Hind Leg
        EqTemp = Ineq_Summation(PHy(k), Mpos_y, CH(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  TerrainHeight + Mpos_y];      %Give constraint upper bound
    %-------------------------------------------
    %       - Equation (2): Py >= Height -->
    %                       Height <= Py <= inf
    %----------------------------------------------------
    %         (Place Holder) For even terrain, achieve this constraint by changing variable lower bounds
    %                        Change to complementarity form when
    %                        introducing uneven terrain
    %----------------------------------------------------
    %           Front Leg
        lb_DecisionVars(find(VarNamesList == ['PFy_',num2str(k-1)])) = TerrainHeight;
        
    %           Hind Leg
        lb_DecisionVars(find(VarNamesList == ['PHy_',num2str(k-1)])) = TerrainHeight;
    %----------------------------------------------------
    %   (*) Foot/End-Effector Velocity (for both x-axis and y-axis)
    %----------------------------------------------------
    %       - Equation (1): Pdot <= 0 + Mvel(1-C) -->
    %                       Pdot + Mvel*C <= Mvel -->
    %         Use Ineq_Summation
    %         Input: v[k] = P(F/H)(x/y)dot[k]
    %                bigM = Mvel
    %                z[k] = C(F/H)[k]
    %         lbg = -inf
    %         ubg = Mvel
    %----------------------------------------------------
    %           Front Leg x-axis
        EqTemp = Ineq_Summation(PFxdot(k), Mvel, CF(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  Mvel];                        %Give constraint upper bound
        
    %           Front Leg y-axis
        EqTemp = Ineq_Summation(PFydot(k), Mvel, CF(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  Mvel];                        %Give constraint upper bound
        
    %           Hind Leg x-axis
        EqTemp = Ineq_Summation(PHxdot(k), Mvel, CH(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  Mvel];                        %Give constraint upper bound
        
    %           Hind Leg y-axis
        EqTemp = Ineq_Summation(PHydot(k), Mvel, CH(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  Mvel];                        %Give constraint upper bound
    %------------------------------------------------------              
    %       - Equation (2): Pdot >= 0 - Mvel(1-C) -->
    %                       Pdot - Mvel*C >= -Mvel -->
    %                       -Mvel <= Pdot - Mvel*C
    %         Use Function Ineq_Difference
    %         Input: v[k] = P(F/H)(x/y)dot[k]
    %                bigM = -Mvel
    %                z[k] = C(F/H)[k]
    %         lbg = -Mvel
    %         ubg = inf
    %------------------------------------------------------
    %           Front Leg x-axis
        EqTemp = Ineq_Difference(PFxdot(k), Mvel, CF(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -Mvel];                       %Give constraint lower bound
        ubg = [ubg;  inf];                         %Give constraint upper bound
        
    %           Front Leg y-axis
        EqTemp = Ineq_Difference(PFydot(k), Mvel, CF(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -Mvel];                       %Give constraint lower bound
        ubg = [ubg;  inf];                         %Give constraint upper bound
        
    %           Hind Leg x-axis
        EqTemp = Ineq_Difference(PHxdot(k), Mvel, CH(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -Mvel];                       %Give constraint lower bound
        ubg = [ubg;  inf];                         %Give constraint upper bound
        
    %           Hind Leg y-axis
        EqTemp = Ineq_Difference(PHydot(k), Mvel, CH(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -Mvel];                       %Give constraint lower bound
        ubg = [ubg;  inf];                         %Give constraint upper bound
    %------------------------------------------------------
    %   (*) Foot/End-Effector Forces
    %------------------------------------------------------
    %     (-) x-axis
    %-----------------------------------------------------
    %       - Euqation (1): Fx <= 0 + Mfx*C -->
    %                     Fx - Mfx*C <= 0
    %           Use Ineq_Difference
    %           Input: v[k] = F(F/H)x[k]
    %                  bigM = Mfx
    %                  z[k] = C(F/H)[k]
    %           lbg = -inf
    %           ubg = 0
    %-----------------------------------------------------
    %           Front Leg
        EqTemp = Ineq_Difference(FFx(k), Mfx, CF(k));
        g   = {g{:}, EqTemp};     %Append to constraint function list
        lbg = [lbg;  -inf];       %Give constraint lower bound
        ubg = [ubg;  0];          %Give constraint upper bound
        
    %           Hind Leg
        EqTemp = Ineq_Difference(FHx(k), Mfx, CH(k));
        g   = {g{:}, EqTemp};     %Append to constraint function list
        lbg = [lbg;  -inf];       %Give constraint lower bound
        ubg = [ubg;  0];          %Give constraint upper bound
    %------------------------------------------------------           
    %       - Equation (2): Fx >= 0 - Mfx*C -->
    %                       Fx + Mfx*C >= 0 -->
    %                       0 <= Fx + Mfx*C
    %           Use Ineq_Summation
    %           Input: v[k] = F(F/H)x[k]
    %                  bigM = Mfx
    %                  z[k] = C(F/H)[k]
    %           lbg = 0
    %           ubg = inf
    %------------------------------------------------------
    %           Front Leg
        EqTemp = Ineq_Summation(FFx(k), Mfx, CF(k));
        g   = {g{:}, EqTemp};     %Append to constraint function list
        lbg = [lbg;  0];          %Give constraint lower bound
        ubg = [ubg;  inf];        %Give constraint upper bound
        
    %           Hind Leg
        EqTemp = Ineq_Summation(FHx(k), Mfx, CH(k));
        g   = {g{:}, EqTemp};     %Append to constraint function list
        lbg = [lbg;  0];          %Give constraint lower bound
        ubg = [ubg;  inf];        %Give constraint upper bound
    %------------------------------------------------------
    %     (-) y-axis
    %------------------------------------------------------
    %       - Equation (1): Fy <= 0 + Mfy*C -->
    %                       Fy - Mfy*C <= 0
    %           Use Ineq_Difference
    %           Input: v[k] = F(F/H)y[k]
    %                  bigM = Mfy
    %                  z[k] = C(F/H)[k]
    %           lbg = -inf
    %           ubg = 0
    %------------------------------------------------------
    %           Front Leg
        EqTemp = Ineq_Difference(FFy(k), Mfy, CF(k));
        g   = {g{:}, EqTemp};     %Append to constraint function list
        lbg = [lbg;  -inf];       %Give constraint lower bound
        ubg = [ubg;  0];          %Give constraint upper bound
        
    %           Hind Leg
        EqTemp = Ineq_Difference(FHy(k), Mfy, CH(k));
        g   = {g{:}, EqTemp};     %Append to constraint function list
        lbg = [lbg;  -inf];          %Give constraint lower bound
        ubg = [ubg;  0];        %Give constraint upper bound
    %------------------------------------------------------
    %       - Equation (2): F(F/H)y >= 0 
    %           Achieve by Changing Lower Variable bounds
    %------------------------------------------------------
    %           Front Leg
        lb_DecisionVars(find(VarNamesList == ['FFy_',num2str(k-1)])) = 0;
    %           Hind Leg
        lb_DecisionVars(find(VarNamesList == ['FHy_',num2str(k-1)])) = 0;
    % Complementarity Constraints Built  
    %----------------------------------------------------
    % Kinematics Constraint
    %----------------------------------------------------
    %   Equation: -[bw;bh]/2 <= R(theta[k])*(P[k] - [x[k];y[k]])-Pcenter <= [bw;bh]/2
    %       Use Function KinematicsConstraint 
    %       Input: r[k] = [x[k],y[k],theta[k]]         --> Robot Torso State
    %              Pk   = [P(F/H)x[k],P(F/H)y[k]]      --> Foot/End-Effector Location
    %              Pc   = [PcenterX, PcenterY]         --> Default Foot/End-Effector Location
    %       lbg = -[bw;bh]/2 
    %       ubg = [bw/bh]/2   --> bw is bounding box width, bh is boungding box height
    %----------------------------------------------------
    %     Front Leg
        EqTemp = KinematicsConstraint([x(k), y(k), theta(k)], [PFx(k), PFy(k)], [PFcenterX, PFcenterY]);
        g   = {g{:}, EqTemp};                                              %Append to constraint function list
        lbg = [lbg;  -[BoundingBox_Width;BoundingBox_Height]/2];           %Give constraint lower bound
        ubg = [ubg;  [BoundingBox_Width;BoundingBox_Height]/2];            %Give constraint upper bound
        
    %     Hind Leg
        EqTemp = KinematicsConstraint([x(k), y(k), theta(k)], [PHx(k), PHy(k)], [PHCenterX, PHCenterY]);
        g   = {g{:}, EqTemp};                                              %Append to constraint function list
        lbg = [lbg;  -[BoundingBox_Width;BoundingBox_Height]/2];           %Give constraint lower bound
        ubg = [ubg;  [BoundingBox_Width;BoundingBox_Height]/2];            %Give constraint upper bound
    %----------------------------------------------------
    % Friction Cone
    %----------------------------------------------------
    %   Equation: Fx[k] - miu*dot(Norm[k],Force[k]) <= 0
    %       Use Function FrictionCone
    %       Input: Norm[k]   = [Nx[k],Ny[k]]
    %              Force[k]  = [F(F/H)x[k], F(F/H)y[k]]
    %              Const_miu = miu
    %       lbg = -inf
    %       ubg = 0
    %----------------------------------------------------
    %   (Place Holder) Need to change terrain norm when move to (uneven)
    %   curvature terrain
    %----------------------------------------------------
    %       Front Leg
        EqTemp = FrictionCone(TerrainNorm, [FFx(k),FFy(k)], miu);
        g   = {g{:}, EqTemp};         %Append to constraint function list
        lbg = [lbg;  -inf];           %Give constraint lower bound
        ubg = [ubg;  0];              %Give constraint upper bound
        
    %       Hind Leg
        EqTemp = FrictionCone(TerrainNorm, [FHx(k), FHy(k)], miu);
        g   = {g{:}, EqTemp};         %Append to constraint function list
        lbg = [lbg;  -inf];           %Give constraint lower bound
        ubg = [ubg;  0];              %Give constraint upper bound
    %----------------------------------------------------
    % Cost Function - Integral/Lagrangian Term
    J = J + h*FFx(k)^2 + h*FFy(k)^2 + h*FHx(k)^2 + h* FHy(k)^2; 
    %----------------------------------------------------
end

%-----------------------------------------------------------------------
%   Initial Condition and Terminal Condition
%----------------------------------------------------------------------
%       x_Init
lb_DecisionVars(find(VarNamesList == ['x_',num2str(0)])) = x_Init;
ub_DecisionVars(find(VarNamesList == ['x_',num2str(0)])) = x_Init;
%       y_Init
lb_DecisionVars(find(VarNamesList == ['y_',num2str(0)])) = y_Init;
ub_DecisionVars(find(VarNamesList == ['y_',num2str(0)])) = y_Init;
%       theta_Init
lb_DecisionVars(find(VarNamesList == ['theta_',num2str(0)])) = theta_Init;
ub_DecisionVars(find(VarNamesList == ['theta_',num2str(0)])) = theta_Init;
%       xdot_Init
lb_DecisionVars(find(VarNamesList == ['xdot_',num2str(0)])) = xdot_Init;
ub_DecisionVars(find(VarNamesList == ['xdot_',num2str(0)])) = xdot_Init;
%       ydot_Init
lb_DecisionVars(find(VarNamesList == ['ydot_',num2str(0)])) = ydot_Init;
ub_DecisionVars(find(VarNamesList == ['ydot_',num2str(0)])) = ydot_Init;
%       thetadot_Init
lb_DecisionVars(find(VarNamesList == ['thetadot_',num2str(0)])) = thetadot_Init;
ub_DecisionVars(find(VarNamesList == ['thetadot_',num2str(0)])) = thetadot_Init;
%       x_End
lb_DecisionVars(find(VarNamesList == ['x_',num2str(NumTimeSteps)])) = x_End;
ub_DecisionVars(find(VarNamesList == ['x_',num2str(NumTimeSteps)])) = x_End;
%       y_End
lb_DecisionVars(find(VarNamesList == ['y_',num2str(NumTimeSteps)])) = y_End;
ub_DecisionVars(find(VarNamesList == ['y_',num2str(NumTimeSteps)])) = y_End;
%       theta_End
lb_DecisionVars(find(VarNamesList == ['theta_',num2str(NumTimeSteps)])) = theta_End;
ub_DecisionVars(find(VarNamesList == ['theta_',num2str(NumTimeSteps)])) = theta_End;
%       xdot_End
lb_DecisionVars(find(VarNamesList == ['xdot_',num2str(NumTimeSteps)])) = xdot_End;
ub_DecisionVars(find(VarNamesList == ['xdot_',num2str(NumTimeSteps)])) = xdot_End;
%       ydot_End
lb_DecisionVars(find(VarNamesList == ['ydot_',num2str(NumTimeSteps)])) = ydot_End;
ub_DecisionVars(find(VarNamesList == ['ydot_',num2str(NumTimeSteps)])) = ydot_End;
%       thetadot_End
lb_DecisionVars(find(VarNamesList == ['thetadot_',num2str(NumTimeSteps)])) = thetadot_End;
ub_DecisionVars(find(VarNamesList == ['thetadot_',num2str(NumTimeSteps)])) = thetadot_End;
%-----------------------------------------------------------------------
disp('Constraints and Objetive Function Constructed')
disp('-------------------------------------------------')
%=======================================================================

%=======================================================================
% Solve the Problem
%=======================================================================
%   Assemble optimization problem definitions
prob = struct('f', J, 'x', DecisionVars, 'g', vertcat(g{:}));
%   Setup solver-dependent options
if strcmp(SolverSelected, 'knitro')
    solverOption = struct('mip_outinterval', 25,...     % (Log Output Frequency) Log Output per Nodes
                          'mip_outlevel',    2,...      % Print accumulated time for every node.
                          'mip_selectrule',  3,...      % The rule for selecting nodes 
                          'mip_branchrule',  1,...      % MIP Branching rule
                          'mip_maxnodes',    1e24);      % Max Number of Nodes wish to be explored
    
elseif strcmp(SolverSelected, 'bonmin')
    solverOption = struct('option_file_name', 'bonmin.opt');  
end

%   Construct Nonlinear Programming Function Object
solver = nlpsol('solver', SolverSelected, prob, struct('discrete', varstype, SolverSelected, solverOption));

%   Solver the Problem
sol = solver('x0',  DecisionVarsInit, ...
             'lbx', lb_DecisionVars,...
             'ubx', ub_DecisionVars,...
             'lbg', lbg,...
             'ubg', ubg);

%=======================================================================
% (ToDo) Extract the Solution
res = full(sol.x);
%=======================================================================

%=======================================================================
% Close Diary
diary off