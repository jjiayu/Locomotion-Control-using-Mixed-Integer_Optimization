% Mixed-integer Nonlinear Optimization in 2D case, use auto differentiation
% to compute gradients and hessians
% Multiphase Formulation:
%   Cut the time horizon into a few phases
%   Each phase has the same number of knots but the phase horizon is
%   different
% Foot-ground contact configurations are modeled as 0/1 for each foot

% Check readme for notes ad future improvements

clear;
clc;

%cd /home/jiayu/Desktop/Locomotion-Control-using-Mixed-Integer_Optimization/casadi_implementation/

%========================================================
% Import CasADi related packages
import casadi.*

%========================================================
% Command Line Logging
diary off
diary_filename = strcat('Periodical-Loco-log-', datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
diary(diary_filename);

%=========================================================
% Display Script Information
disp('====================================================');
disp('Genearl Information:');
disp('====================================================');
disp('CasADi Implementation');
disp('2D Locomotion Control using Mixed-integer Nonlinear Optimization');
disp('With a Particular Emphasis on Periodical Gait Discovery');
disp('Multi-Phase Formulation:');
disp('Optimize over State, Control, Gait Sequence, and Switching Time');
disp('----------------------------------------------------');
disp('Date and Time:');
disp(datetime('now'));
disp('----------------------------------------------------');
disp(['Correspondent Log File Name: ', diary_filename]);
disp('====================================================');
disp(' ');
%=====================================================================
% Add path
%addpath('/home/jiayu/bin/casadi-linux-matlabR2014b-v3.4.5')

%======================================================================
%Inertia Parameters(Information from MIT Cheetah 3)
m = 45; %kg
I = 2.1; %kg m^2 Izz
G = 9.80665; %m/s^2
%======================================================================

%======================================================================
%   Kinematics Constraint Parameters
%======================================================================
%       Body Size
BodyLength = 0.6;
BodyHeight = 0.2;
%       Default foot position in Local robot frame
DefaultLegLength = 0.45; %default leg length , distance from the default Leg Y to Torso (LOWER BORDER of the TORSO)
%           Front Foot Default Positions (IN ROBOT FRAME)
PFCenterX = 1/2*BodyLength;
PFCenterY = -(1/2*BodyHeight + DefaultLegLength);
%           Hind Foot Default Positions (IN ROBOT FRAME)
PHCenterX = -1/2*BodyLength;
PHCenterY = -(1/2*BodyHeight + DefaultLegLength);
%       Kinematics Bounding Box Constraint
disp('====================================================');
disp('Setup Robot Kinematics Properties: ')
disp('----------------------------------------------------');
BoundingBox_Width = 0.6;
BoundingBox_Height = 0.6;
%======================================================================

%======================================================================
%Environment Information
%----------------------------------------------------------------------
%   Display some info
%----------------------------------------------------------------------
disp('====================================================');
disp('Setup Terrain Model: ')
disp('----------------------------------------------------');
%----------------------------------------------------------------------
%   Setup the Terrain Model
%----------------------------------------------------------------------
TerrainType = input('Specify the Terrain Type: 1 -> Flat Terrain; 2 -> Slopes\n');

if TerrainType == 1 %Flat Terrain
    disp('Selected Flat Terrain');
    %Use Lagecy Implementation
    %       Maximum Number of Stairs can handle
    MaxNumStairs = 10;
    NumStairs = MaxNumStairs; %May remove
    HeightChangingPlaces = zeros(1,MaxNumStairs);
    LevelChanges   = zeros(1,MaxNumStairs);
    %For Visualization program
    HeightChangingPlaces_vis = ones(1,MaxNumStairs);
    LevelChanges_vis = LevelChanges;
elseif TerrainType == 2 %Slope
    disp('Selected Slope Terrain');
    ME_SlopeUnavailable = MException('Initialization:SlopeUnavailable',['Slope Terrain Un-implemented']);
else %Unknown Scenario
    ME_TerrainType = MException('Initialization:TerrainType',['Unknown Terrain Type']);
    throw(ME_TerrainType)
end

%         Build Terrain Model Function --> CasADi If-Else implementation
%           Define CasADi symbolic variables
h_terrain = 0;
x_query   = SX.sym('x_query', 1);
for i = 1:MaxNumStairs
    h_terrain = h_terrain + if_else(x_query < sum(HeightChangingPlaces(1:i)), 0, LevelChanges(i));
end
TerrainModel = Function('TerrainModel', {x_query}, {h_terrain});
disp('----------------------------------------------------');
%-----------------------------------------------------------------------
%   Plot Terrain Model
PlotTerrainFlag = input('Plot the Terrain Model? 1 -> Yes; 2 -> No\n');
if PlotTerrainFlag == 1 %Yes, Plot the terrain model    
    terrainx = linspace(-2, sum(HeightChangingPlaces)+3, 1e4);
    terrainy = full(TerrainModel(terrainx));
    plot(terrainx,terrainy,'LineWidth',2)
    ylim([min(terrainy)-1,max(terrainy)+1])
    disp('----------------------------------------------------');
end
%-----------------------------------------------------------------------
%Other Parameters
TerrainNorm = [0,1];
miu = 0.6; %friction coefficient
disp('----------------------------------------------------');
disp(['Friction Cone: ', num2str(miu)]);
disp('====================================================');
disp(' ')
%======================================================================

%======================================================================
% Task Specifications
%======================================================================
%   Display Information
%----------------------------------------------------------------------
disp('====================================================');
disp('Task Specification:')
disp('----------------------------------------------------');
%----------------------------------------------------------------------
%   Specify Desired Speed
%----------------------------------------------------------------------
speed = input('Specify Travel Speed along X-axis (m/s): \n');
disp('----------------------------------------------------');
%----------------------------------------------------------------------
%   Terminal time
%----------------------------------------------------------------------
disp('Terminal Time:')
Tend_flag = input('Optimization of Terminal Time (Tend): \n1 -> Optimize, 2 -> Left as Free Variable \n');
if Tend_flag == 1 %Optimize Terminal Time
    Tend = input('Input Termianl Time (e.g. 1s): \n');
elseif Tend_flag == 2
    disp('Terminal Time (Tend) is Set as Free Variables')
    Tend_Bound = input('Specify Upper Bound of Terminal Time (i.e. 1,2..(s)):\n');
else
    ME_Tend = MException('Initialization:Tend_flag','Unknown Indicator for Determining the on/off of Terminal Time Optimization');
    throw(ME_Tend)
end
disp('----------------------------------------------------');
disp(' ')
%======================================================================

%======================================================================
% Time Step and Discretization Parameter Settings
%----------------------------------------------------------------------
%   Display Info
%----------------------------------------------------------------------
disp('====================================================');
disp('Temporal and Discretization Setup:');
disp('----------------------------------------------------');
%   Number of Phases
NumPhases = input('Input Number of Phases: \n');
disp('----------------------------------------------------');
%   Number of timesteps for each phase
NumLocalKnots = input('Input Number of Knots for Each Phase: \n');
disp('----------------------------------------------------');
%   Total Number of TimeSteps exclude time step 0
NumKnots = NumPhases*NumLocalKnots;
%   Parameter tau
tau_upper_limit = 1; %upper limit of tau --> tau \in [0,1]
tauStepLength = tau_upper_limit/NumKnots; %discritization step size of tau
%   Discretization of tau
tauSeries = 0:tauStepLength:tau_upper_limit;
tauSeriesLength = length(tauSeries);
%   Print some Info
disp('Resultant Discretization:')
disp(['Knot/Discretization Step Size of tau: ', num2str(tauStepLength)]);
disp(['Number of Knots/Discretization of tau (from 0 to ', num2str(tau_upper_limit), ': ', num2str(tauSeriesLength), ') (Number of Knots/Discretization in Each Phase * Number of Phases(NumKnots) + 1)']);
disp('====================================================');
disp(' ')
%======================================================================


%=======================================================================
% (Place Holder): Setup of Soft constraints on Terminal Condition or not
%=======================================================================

%======================================================================
% Big-M Parameters for Complementarity Constraints
%======================================================================
%   Display Information
%----------------------------------------------------------------------
disp('====================================================');
disp('Big-M Setup')
disp('====================================================');
%----------------------------------------------------------------------
%   Big-M for foot positions in y-axis (meters)
Mpos_y = 100; 
%----------------------------------------------------------------------
%   Determine big-M for foot velocity
%-------------------------------
%   (Place Holder) big-M for all x, y, z axis velocities need to respecify
%   when move to 3D
%------------------------------------------------------------------------
%       big-M for X-axis Foot/End-Effector Velocity
%-------------------------------------------------------------------------
disp('Big-M for Foot/End-Effector in X-axis:')
MvelxCase = input('Select the Setup Case for big-M value for Foot/End-Effector Velocity in x-axis: \n1 -> Default Value (5m/s); 2 -> N time of average task speed (in X-axis); 3 -> User Specified: \n');
disp(' ')
if MvelxCase == 1
    Mvelx = 5; %Default Mvel Value
%    disp(['Selected Default Value for Big-M value for Foot/End-Effector Valocity in X-axis: ', num2str(Mvelx)]);
elseif MvelxCase == 2
    if exist('Tend', 'var')
        disp(['Current Averate Task Speed in X-axis: ', num2str(speed)]);
        MvelxScaling = input('Input the Scaling Factor of the Average Task Speed (in X-axis) (i.e. 1, 2, 3, etc...): \n');
        Mvelx = speed*MvelxScaling;
    else
        warning('Terminal Time is Undefined -> Unable to Compute Average Task Speed in X-axis');
        Mvelx = input('Manually Specify a Big-M value for Foot/End-Effector Velocity in X-axis: \n');
    end
elseif MvelxCase == 3
    Mvelx = input('Input Big-M value for foot/end-effector Velocity for X-axis: \n');
else
    ME_MvelxCase = MException('Initialization:MvelxCase','Unexpected Case for big-M for X-axis foot/End-effector Velocity');
    throw(ME_MvelxCase)
end
disp(' ')
disp(['Configured Big-M Value for Foot/End-Effector Velocity for X-axis: ', num2str(Mvelx), ' m/s']);
disp('----------------------------------------------------');
%---------------------------------------------------------------------
%       big-M for Y-axis Foot/End-Effector Velocity
%---------------------------------------------------------------------
disp('Big-M for Foot/End-Effector in Y-axis:')
MvelyCase = input('Select the Setup Case for big-M value for Foot/End-Effector Velocity in Y-axis: \n1 -> Default Value (5m/s); 2 -> User Specified: \n');
disp(' ')
if MvelyCase == 1
    Mvely = 5; %Default Mvely Value
%    disp(['Selected Default Value for Big-M value for Foot/End-Effector Valocity in Y-axis: ', num2str(Mvely), ' m/s']);
elseif MvelyCase == 2
    Mvely = input('Input Big-M value for foot/end-effector Velocity for Y-axis (e.g. 1-5+ m/s): \nNOTE: Can Write the Value as the Kinematics Bounding Box Height (BoundingBox_Height)/Desired Time to Travel the Entire Bounding Box Height\n');
%    disp(['Big-M value for Foot/End-Effector Velocity in Y-axis set as: ', num2str(Mvely), ' m/s']);
else
    ME_MvelyCase = MException('Initialization:MvelyCase','Unexpected Case for big-M for Y-axis foot/End-effector Velocity');
    throw(ME_MvelyCase)
end
disp(' ')
disp(['Configured Big-M Value for Foot/End-Effector Velocity for Y-axis: ', num2str(Mvely), ' m/s']);
disp('----------------------------------------------------');
%----------------------------------------------------------------------
%   Big-M for Foot-Ground Reaction Forces
disp('Big-M for Contact Forces')
Mfx = input('Input Big-M for Foot-Ground Reaction Forces along X-axis (e.g. 200,300,1000,1e5):\n');
disp('----------------------------------------------------');
Mfy = input('Input Big-M for Foot-Ground Reaction Forces along Y-axis (e.g. 1e3, 1e5, better larger than Mfx):\n');
%Mfx = 1e2; %(N) big-M for foot-ground reaction forces for x-axis
%Mfy = 1e5; %(N) big-M for foot-ground reaction forces for y-axis
disp('====================================================');
%=======================================================================

%=====================================================================
% Solver SetUp
%=====================================================================
%   Choose Solver
%---------------------------------------------------------------------
disp('====================================================');
disp('Solver Setups:')
disp('====================================================');
disp('Solver Selection: ')
SolverNum = input('1 -> Knitro; 2 -> Bonmin \n');
if SolverNum == 1
    SolverSelected = 'knitro';
elseif SolverNum == 2
    SolverSelected = 'bonmin';
else
    ME_SelectSolvers = MException('Initialization:SelectSolvers','Unknown Solver Nominated');
    throw(ME_SelectSolvers)
end
disp(['Selected Solver: ', SolverSelected])
disp('----------------------------------------------------');
%   Solver Dependent Options
disp('Solver Dependent Options:')
disp('----------------------------------------------------');
%       Define maximum nodes to be explored
NumMaxNodesCases = input('Define Number of Max Nodes to be Explored: \n 1--> Worst Case Scenario; 2 --> User Specified; 3 --> Default Value\n');
if NumMaxNodesCases == 1  %Worst-case Scenario
    %-------------------------------------------
    %   (Place Holder) Need to change the exponential base when have more
    %   legs in 3D
    %-------------------------------------------
    NumMaxNodes = (2*2)^(NumPhases);
    disp(['Selected Worst-case Scenarios to Explore ', num2str(NumMaxNodes), ' Nodes']);
elseif NumMaxNodesCases == 2 %User-specified
    NumMaxNodes = input('Input number of maximum node to be explored: \n');
elseif NumMaxNodesCases == 3 %Default Value
    NumMaxNodes = 1e5;
    disp(['Selected Default Case to Explore ', num2str(NumMaxNodes), ' Nodes'])
else
    ME_NumMaxNodes = MException('Initialization:NumMaxNodes','Unexpected Settings of Max Number of Nodes');
    throw(ME_NumMaxNodes)
end
disp('====================================================');
disp(' ')
%======================================================================

%=======================================================================
% Define/Create Modeling Variables
%=======================================================================
%   Display some Info
disp('====================================================');
disp('Generate Decision Variables Vectors/Lists:')
disp('----------------------------------------------------');
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
x = SX.sym('x', tauSeriesLength); 
y = SX.sym('y', tauSeriesLength); 
theta = SX.sym('theta', tauSeriesLength);
xdot = SX.sym('xdot', tauSeriesLength); 
ydot = SX.sym('ydot', tauSeriesLength); 
thetadot = SX.sym('thetadot', tauSeriesLength);
%           FootStep Locations and Velocities
%               Front Leg Locations
PFx = SX.sym('PFx', tauSeriesLength);
PFy = SX.sym('PFy', tauSeriesLength);
%               Front Leg Velocities
PFxdot = SX.sym('PFxdot', tauSeriesLength); 
PFydot = SX.sym('PFydot', tauSeriesLength);
%               Hind Leg Locations
PHx = SX.sym('PHx', tauSeriesLength);
PHy = SX.sym('PHy', tauSeriesLength);
%               Hind Leg Velocities
PHxdot = SX.sym('PHxdot', tauSeriesLength);
PHydot = SX.sym('PHydot', tauSeriesLength);
%           Foot-Ground Reaction Forces
%               Front Leg Forces
FFx = SX.sym('FFx', tauSeriesLength);
FFy = SX.sym('FFy', tauSeriesLength);
%               Hind Leg Forces
FHx = SX.sym('FHx', tauSeriesLength);
FHy = SX.sym('FHy', tauSeriesLength);
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
%   Phase-related Variables
%-----------------------------------------------------------------------
%       Note: For fixed switching time formulation and multi-phase formulation, 
%             first create contact configuration and (siwtching time) from 0 to number of phases, and
%             extract the sub variable list from 1 to number of phases
%             The contact configuration and switching time variables should have the same length
%             with respect to the number of phases
%-----------------------------------------------------------------------
%       Define CasADi Variables
%-----------------------------------------------------------------------
%       (*) Switching Time of Each Phase (Continuous Variable, Termination time of each phase)
%-----------------------------------------------------------------------
Ts = SX.sym('Ts', NumPhases + 1); %Ts is dented as Swtiching Time
Ts = Ts(2:end);
Ts_label = CreateVarsNameList(Ts);
%-----------------------------------------------------------------------
%       (*) Contact Configuration (Discrete Vairbale)
%-----------------------------------------------------------------------
%           Contact Configuration (Mode)
%               Leg Contact Configurations: C = [CF, CH]; CF, CH = 0/1 (Binary Variable)
%                   Front Leg Contact On/Off: CF
%                   Hind Leg Contact On/Off : CH
%--------------------------------------------
CF = SX.sym('CF', NumPhases + 1);
CH = SX.sym('CH', NumPhases + 1);
CF = CF(2:end);
CH = CH(2:end);

%       Create Variable Name List
CF_label = CreateVarsNameList(CF);
CH_label = CreateVarsNameList(CH);
%----------------------------------------------------------------------
%       Check Correctness of the Generated Variables
%----------------------------------------------------------------------
if length(CF) == NumPhases && length(CH) == NumPhases && length(Ts) == NumPhases
    disp('Checked - Contact Configuration and Switching Time Varaibels for Each Phase are Generated Coorectly')
else
    ME_PhaseRelatedConfig = MException('Initialization:PhaseRelatedConfig','The contact configuration and switching time is not defined for each phase');
    throw(ME_PhaseRelatedConfig)
end

%-----------------------------------------------------------------------
%   Assemble Lists for all decision variables
%       Identifiers of all variables
varList = ["x",    "y",      "theta",...
           "xdot", "ydot",   "thetadot",...
           "PFx",  "PFy",    "PFxdot",      "PFydot",...
           "PHx",  "PHy",    "PHxdot",      "PHydot",...
           "FFx",  "FFy",...
           "FHx",  "FHy",...
           "Ts",...
           "CF",   "CH"];
       
%       Variable Length List
VarLengthList = [length(x_label),     length(y_label),      length(theta_label), ...
                 length(xdot_label),  length(ydot_label),   length(thetadot_label), ...
                 length(PFx_label),   length(PFy_label),    length(PFxdot_label),       length(PFydot_label), ...
                 length(PHx_label),   length(PHy_label),    length(PHxdot_label),       length(PHydot_label), ...
                 length(FFx_label),   length(FFy_label),...
                 length(FHx_label),   length(FHy_label),...
                 length(Ts_label),...
                 length(CF_label),    length(CH_label)];

%       Full decision variable names list
VarNamesList = [x_label,      y_label,      theta_label,...
                xdot_label,   ydot_label,   thetadot_label,...
                PFx_label,    PFy_label,    PFxdot_label,      PFydot_label,...
                PHx_label,    PHy_label,    PHxdot_label,      PHydot_label,...
                FFx_label,    FFy_label,...
                FHx_label,    FHy_label,...
                Ts_label,...
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
%--------------------------------
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
                Ts,...
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

%       Other Important Setups
%           Initial Guess of Decision Variables
DecisionVarsInit = rand(size(DecisionVars)); %Random Initial Guess
%DecisionVarsInit = zeros(size(DecisionVars)); %Zero Initial guess
%           Lower and upper bounds, variable type
lb_DecisionVars = [];  %Variable Lower Bound 
ub_DecisionVars = [];  %Variable Upper Bound
varstype        = [];  %Variable Type: 0 -> Continuous Variable/ 1 -> Binary Variable
for i = 1:length(varList)
    if contains(varList(i),'C') == 1 %binary variable
        lb_DecisionVars = [lb_DecisionVars, zeros(1, VarLengthList(i))];    
        ub_DecisionVars = [ub_DecisionVars, ones( 1, VarLengthList(i))];
        varstype        = [varstype,        ones(1, VarLengthList(i))];            %0 (use zeros) -> Continuous Variable/ 1 (use ones) -> Binary Variable
    else %continuous variables
        if strcmp(varList(i),'theta') == 1
            lb_DecisionVars = [lb_DecisionVars, repmat(-pi/2, 1, VarLengthList(i))];    
            ub_DecisionVars = [ub_DecisionVars, repmat( pi/2, 1, VarLengthList(i))];
            
        else %other unbounded variables
            lb_DecisionVars = [lb_DecisionVars, repmat(-inf, 1, VarLengthList(i))];    
            ub_DecisionVars = [ub_DecisionVars, repmat( inf, 1, VarLengthList(i))];
        end
        
        varstype        = [varstype,        zeros(1, VarLengthList(i))];           %0 (use zeros) -> Continuous Variable/ 1 (use ones) -> Binary Variable
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
%   Create Time Step Variable for each Phase
hVector = [Ts(1)-0];
hVector = tauStepLength*NumPhases*[hVector;diff(Ts)];
%-----------------------------------------------------------------------
%   Loop Over the Time Horizon
%-----------------------------------------------------------------------
for k = 1:tauSeriesLength
    
    %--------------------------------------
    % Extract Phase Index
    if k<= tauSeriesLength - 1
        PhaseIdx = floor((k-1)/NumLocalKnots) + 1; % k-1 is the time step enumeration
    elseif k == tauSeriesLength
        PhaseIdx = floor((k-1)/NumLocalKnots) + 1 - 1; %the equation will give last time belongs to NumPhases +1, we simply classify it into the last phase
    end
    %--------------------------------------
    
    %--------------------------------------
    % System dynamics
    %   Use EulerIntegration Function to Construct
    %   EulerIntegration = x[k+1] - x[k] -h[k]*fdyn[k] = 0
    %   Input: x[k+1], x[k], h[k], fdyn[k] (Display sequence as defined in EulerIntegration Function)
    %   lbg = 0 (Equality constraint)
    %   ubg = 0 (Equality constraint)
    %   Covering Range: from k = 1 to k = tauSeriesLength - 1
    %--------------------------------------
    %   Constraint the range
    %--------------------------------------
    if k <= tauSeriesLength - 1
        %----------------------------------
        % Get time step
        h = hVector(PhaseIdx);
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
        % (*) x-axis second-order dynamics (velocity)
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
        %   Dynamical Equation Constraint Setup - Done
    end
    %----------------------------------------------------
    % Complementarity Constraint
    %   Use Functions: Ineq_Difference --> v[k] - bigM*z[k]
    %                  Ineq_Summation  --> v[k] + bigM*z[k]
    %   Input: v[k] --> Continuous Variables at time step k
    %          bigM --> bigM constant
    %          z[k] --> Integer/Binary Variable at time step k
    %----------------------------------------------------
%         %   (*) Extract Index for Identifying Contact Configuration for
%         %--------------------------------------------------------------
%         %   Governing Current Time Step
% 
%             ContactConfigIdx = floor((k-1)/NumLocalTimeSteps) + 1; % k-1 is the time step enumeration

    %----------------------------------------------------
    %   (*) Foot/End-effector Position (y-axis only)
    %---------------------------------------------------
    %       (Place Holder) Need to Change Height into loop-up table
    %       function if we go for uneven terrain
    %--------------------------------------------------
    %       - Equation (1): Py <= Height + Mpos(1-C) -->
    %                       Py + Mpos*C <= Height + Mpos -->
    %                       Py + Mpos*C - Height(x) <= Mpos
    %         Use Ineq_Summation
    %         Input: v[k] = P(F/H)y[k]
    %                bigM = Mpos_y
    %                z[k] = C(F/H)[k]
    %                Additionally: - TerrainHeight
    %         lbg = -inf 
    %         ubg = Mpos_y
    %-------------------------------------------    
    %           Front Leg
        EqTemp = Ineq_Summation(PFy(k),Mpos_y, CF(PhaseIdx)) - TerrainModel(PFx(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  Mpos_y];      %Give constraint upper bound

    %           Hind Leg
        EqTemp = Ineq_Summation(PHy(k), Mpos_y, CH(PhaseIdx)) - TerrainModel(PHx(k));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  Mpos_y];      %Give constraint upper bound
    %-------------------------------------------
    %       - Equation (2): Py >= Height -->
    %                       Height <= Py <= inf -->
    %       For Uneven Terrain: 0 <= Py -Height(x) <= inf
    %
    %----------------------------------------------------
    %         (Place Holder) For even terrain, achieve this constraint by changing variable lower bounds
    %                        Change to complementarity form when
    %                        introducing uneven terrain
    %----------------------------------------------------
    %   Front Leg
    EqTemp = PFy(k) - TerrainModel(PFx(k));
    g   = {g{:}, EqTemp};
    lbg = [lbg;  0];
    ubg = [ubg;  inf];
    %   Hind Leg
    EqTemp = PHy(k) - TerrainModel(PHx(k));
    g   = {g{:}, EqTemp};
    lbg = [lbg;  0];
    ubg = [ubg;  inf];

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
        EqTemp = Ineq_Summation(PFxdot(k), Mvelx, CF(PhaseIdx));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  Mvelx];                       %Give constraint upper bound

    %           Front Leg y-axis
        EqTemp = Ineq_Summation(PFydot(k), Mvely, CF(PhaseIdx));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  Mvely];                       %Give constraint upper bound

    %           Hind Leg x-axis
        EqTemp = Ineq_Summation(PHxdot(k), Mvelx, CH(PhaseIdx));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  Mvelx];                       %Give constraint upper bound

    %           Hind Leg y-axis
        EqTemp = Ineq_Summation(PHydot(k), Mvely, CH(PhaseIdx));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -inf];                        %Give constraint lower bound
        ubg = [ubg;  Mvely];                       %Give constraint upper bound
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
        EqTemp = Ineq_Difference(PFxdot(k), Mvelx, CF(PhaseIdx));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -Mvelx];                      %Give constraint lower bound
        ubg = [ubg;  inf];                         %Give constraint upper bound

    %           Front Leg y-axis
        EqTemp = Ineq_Difference(PFydot(k), Mvely, CF(PhaseIdx));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -Mvely];                      %Give constraint lower bound
        ubg = [ubg;  inf];                         %Give constraint upper bound

    %           Hind Leg x-axis
        EqTemp = Ineq_Difference(PHxdot(k), Mvelx, CH(PhaseIdx));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -Mvelx];                      %Give constraint lower bound
        ubg = [ubg;  inf];                         %Give constraint upper bound

    %           Hind Leg y-axis
        EqTemp = Ineq_Difference(PHydot(k), Mvely, CH(PhaseIdx));
        g   = {g{:}, EqTemp};                      %Append to constraint function list
        lbg = [lbg;  -Mvely];                      %Give constraint lower bound
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
        EqTemp = Ineq_Difference(FFx(k), Mfx, CF(PhaseIdx));
        g   = {g{:}, EqTemp};     %Append to constraint function list
        lbg = [lbg;  -inf];       %Give constraint lower bound
        ubg = [ubg;  0];          %Give constraint upper bound

    %           Hind Leg
        EqTemp = Ineq_Difference(FHx(k), Mfx, CH(PhaseIdx));
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
        EqTemp = Ineq_Summation(FFx(k), Mfx, CF(PhaseIdx));
        g   = {g{:}, EqTemp};     %Append to constraint function list
        lbg = [lbg;  0];          %Give constraint lower bound
        ubg = [ubg;  inf];        %Give constraint upper bound

    %           Hind Leg
        EqTemp = Ineq_Summation(FHx(k), Mfx, CH(PhaseIdx));
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
        EqTemp = Ineq_Difference(FFy(k), Mfy, CF(PhaseIdx));
        g   = {g{:}, EqTemp};     %Append to constraint function list
        lbg = [lbg;  -inf];       %Give constraint lower bound
        ubg = [ubg;  0];          %Give constraint upper bound

    %           Hind Leg
        EqTemp = Ineq_Difference(FHy(k), Mfy, CH(PhaseIdx));
        g   = {g{:}, EqTemp};     %Append to constraint function list
        lbg = [lbg;  -inf];          %Give constraint lower bound
        ubg = [ubg;  0];        %Give constraint upper bound
    %------------------------------------------------------
    %       - Equation (2): F(F/H)y >= 0 
    %           Achieve by Changing Lower Variable bounds
    %-------------------------------------------------------
    %       (Place Holder) May need to consider the terrain direction when
    %       having slope terrain (Do not pull from the ground surface)
    %------------------------------------------------------
    %           Front Leg
        lb_DecisionVars(find(VarNamesList == ['FFy_',num2str(k-1)])) = 0;
    %           Hind Leg
        lb_DecisionVars(find(VarNamesList == ['FHy_',num2str(k-1)])) = 0;
    % Complementarity Constraints Built  
    %------------------------------------------------------

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
        EqTemp = KinematicsConstraint([x(k), y(k), theta(k)], [PFx(k), PFy(k)], [PFCenterX, PFCenterY]);
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
%     %----------------------------------------------------
%     % Torso y-axis level constraint (The Highest Bounding Box border should be always above the Ground)
%     %----------------------------------------------------
%     %   Equation: 1/2*BodyHeight <= y[k] - TerrainModel(x[k]) <= inf
%     %   lbg = 0
%     %   ubg = inf
%         
%         EqTemp = y(k) - TerrainModel(x(k));
%         g   = {g{:}, EqTemp};
%         lbg = [lbg; 1/2*BodyHeight];
%         ubg = [ubg; inf];
%     
    %----------------------------------------------------
    % Cost Function - Integral/Lagrangian Term
    %J = J + h*FFx(k)^2 + h*FFy(k)^2 + h*FHx(k)^2 + h*FHy(k)^2 + h*PFxdot(k)^2 + h*PFydot(k)^2 + h*PHxdot(k)^2 + h*PHydot(k)^2;
    J = J + h*FFx(k)^2 + h*FFy(k)^2 + h*FHx(k)^2 + h*FHy(k)^2; 
    %J = J + h*xdot(k)^2;%h*ydot(k)^2 + h*thetadot(k)^2;% + h*thetadot(k)^2; h*xdot(k)^2 + 
    %----------------------------------------------------
end

%-----------------------------------------------------------------------
%   Switching Time Constraints
%       Equations: 0 <= Ts[1] <= Ts[2] <= ... <= Ts[end]
%       In CasAdi Form:
%           (*) 0 <= Ts[1] - 0 (Init Time)
%               Lower Bound: 0
%               Upper Bound: inf
%           (*) 0 <= Ts[2] - Ts[1] ... 0 <= Ts[i] - Ts[i-1]; for i from 2
%           to NumPhase - 1
%               Lower Bound: 0
%               Upper Bound: inf
%           (*) Ts[end] = Tend (Terminal Time Constraint)
%               - Achieve by Setting up Variable Bounds (Lower Bound = Upper Bound = Tend)
%-----------------------------------------------------------------------
for i = 1:NumPhases
    if i == 1
        EqTemp = Ts(1) - 0;
        g = {g{:}, EqTemp};
        lbg = [lbg; 0];
        ubg = [ubg; inf];
    else
        EqTemp = Ts(i) - Ts(i-1);
        g = {g{:}, EqTemp};
        lbg = [lbg; 0];
        ubg = [ubg; inf]; 
    end
end
%-----------------------------------------
%    Terminal Time Constraint
%-----------------------------------------
if Tend_flag == 1 %only constrain terminal time when user specifies that
    lb_DecisionVars(find(VarNamesList == ['Ts_',num2str(NumPhases)])) = Tend;
    ub_DecisionVars(find(VarNamesList == ['Ts_',num2str(NumPhases)])) = Tend;
elseif Tend_flag == 2 %Bounded Terminal Time
    lb_DecisionVars(find(VarNamesList == ['Ts_',num2str(NumPhases)])) = 0;
    ub_DecisionVars(find(VarNamesList == ['Ts_',num2str(NumPhases)])) = Tend_Bound;
end
%-----------------------------------------------------------------------

%-----------------------------------------------------------------------
%   Periodical Motion Constraints
%       Equation (Except X-axis positions): State_Init - State_End = 0
%       lbg = 0
%       ubg = 0
%-----------------------------------------------------------------------
%       X-axis positions
%-----------------------------------------------------------------------
%           Initial X-axis Position -> always = 0
lb_DecisionVars(find(VarNamesList == ['x_',num2str(0)])) = 0;
ub_DecisionVars(find(VarNamesList == ['x_',num2str(0)])) = 0;
%           Terminal X-axis Position
if Tend_flag == 1 %specified Tend
   lb_DecisionVars(find(VarNamesList == ['x_',num2str(NumKnots)])) = speed*Tend;
   ub_DecisionVars(find(VarNamesList == ['x_',num2str(NumKnots)])) = speed*Tend; 
elseif Tend_flag == 2 %Bounded Terminal Time -> x_end = speed*Tend -> x_end - speed*Tend = 0
    EqTemp = x(end) - speed*Ts(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
end
%-----------------------------------------------------------------------
%       Y-axis Positions
%-----------------------------------------------------------------------
EqTemp = y(1) - y(end);
g = {g{:}, EqTemp};
lbg = [lbg; 0];
ubg = [ubg; 0];
%-----------------------------------------------------------------------
%       Theta Positions
%-----------------------------------------------------------------------
EqTemp = theta(1) - theta(end);
g = {g{:}, EqTemp};
lbg = [lbg; 0];
ubg = [ubg; 0];
%-----------------------------------------------------------------------
%       X-axis Velocities
%-----------------------------------------------------------------------
EqTemp = xdot(1) - xdot(end);
g = {g{:}, EqTemp};
lbg = [lbg; 0];
ubg = [ubg; 0];
%-----------------------------------------------------------------------
%       Y-axis Velocities
%-----------------------------------------------------------------------
EqTemp = ydot(1) - ydot(end);
g = {g{:}, EqTemp};
lbg = [lbg; 0];
ubg = [ubg; 0];
%-----------------------------------------------------------------------
%       Angular Velocity
%-----------------------------------------------------------------------
EqTemp = thetadot(1) - thetadot(end);
g = {g{:}, EqTemp};
lbg = [lbg; 0];
ubg = [ubg; 0];
%-----------------------------------------------------------------------
%   Front Leg Positions (Convert to Robot Local Frame)
%-----------------------------------------------------------------------
EqTemp = [cos(theta(1)), -sin(theta(1)); sin(theta(1)), cos(theta(1))]'*([PFx(1),PFy(1)]' - [x(1),y(1)]') - [cos(theta(end)), -sin(theta(end)); sin(theta(end)), cos(theta(end))]'*([PFx(end),PFy(end)]' - [x(end),y(end)]');
g = {g{:}, EqTemp};
lbg = [lbg; 0; 0];
ubg = [ubg; 0; 0];
%-----------------------------------------------------------------------
%   Hind Leg Positions (Convert to Robot Local Frame)
%-----------------------------------------------------------------------
EqTemp = [cos(theta(1)), -sin(theta(1)); sin(theta(1)), cos(theta(1))]'*([PHx(1),PHy(1)]' - [x(1),y(1)]') - [cos(theta(end)), -sin(theta(end)); sin(theta(end)), cos(theta(end))]'*([PHx(end),PHy(end)]' - [x(end),y(end)]');
g = {g{:}, EqTemp};
lbg = [lbg; 0; 0];
ubg = [ubg; 0; 0];
%-----------------------------------------------------------------------
disp('Constraints and Objetive Function Constructed')
disp('===================================================')
disp(' ')
%=======================================================================

%=======================================================================
% Solve the Problem
%=======================================================================
%   Display some Info
disp('===================================================')
disp('Optimization Started')
%   Assemble optimization problem definitions
prob = struct('f', J, 'x', DecisionVars, 'g', vertcat(g{:}));

%       Build Solver Option Structure
if strcmp(SolverSelected, 'knitro')
    solverOption = struct('mip_outinterval', 50,...     % (Log Output Frequency) Log Output per Nodes
                          'mip_heuristic',   0,...
                          'mip_outlevel',    2,...      % Print accumulated time for every node.
                          'mip_selectrule',  3,...      % The rule for selecting nodes 2 has the best performance
                          'mip_branchrule',  2,...      % MIP Branching rule
                          'mip_maxnodes',    NumMaxNodes);      % Max Number of Nodes wish to be explored
    
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
disp('===================================================')
disp(' ')
%=======================================================================
% Extract the Solution and Visualization
%=======================================================================
%   Display some info
disp('===================================================');
disp('Result Extraction:');
disp('---------------------------------------------------');
%-----------------------------------------------------------------------
%   Recover the full solution
%-----------------------------------------------------------------------
res = full(sol.x);

%   Extract Switching Time
PhaseSwitchingTime = [0;res(find(VarNamesList == Ts_label(1)):find(VarNamesList == Ts_label(end)))]; %include initial time 0
TimeSlope = NumPhases.*diff(PhaseSwitchingTime);
TimeIntercept = [PhaseSwitchingTime(1:end-1)];

TimeSeries = zeros(NumKnots + 1,1); %with starting time 0
for i = 1:NumPhases
    for j = 1:NumLocalKnots
        TimeSeries((i-1)*NumLocalKnots + j + 1) = TimeIntercept(i) + TimeSlope(i)*j*tauStepLength; %start from index 2, index 1 is time 0
    end
end
%------------------------------------------------------------------------

%------------------------------------------------------------------------
%   Extract Original Solutions/Variables
%------------------------------------------------------------------------
% Robot state (position)
x_result     = res(find(VarNamesList == 'x_0'):find(VarNamesList == x_label(end)));
y_result     = res(find(VarNamesList == 'y_0'):find(VarNamesList == y_label(end)));
theta_result = res(find(VarNamesList == 'theta_0'):find(VarNamesList == theta_label(end)));

% Robot state (Velocity)
xdot_result = res(find(VarNamesList == 'xdot_0'):find(VarNamesList == xdot_label(end)));
ydot_result = res(find(VarNamesList == 'ydot_0'):find(VarNamesList == ydot_label(end)));
thetadot_result = res(find(VarNamesList == 'thetadot_0'):find(VarNamesList == thetadot_label(end)));

% End-effector locations
PFx_result = res(find(VarNamesList == 'PFx_0'):find(VarNamesList == PFx_label(end)));
PFy_result = res(find(VarNamesList == 'PFy_0'):find(VarNamesList == PFy_label(end)));
PHx_result = res(find(VarNamesList == 'PHx_0'):find(VarNamesList == PHx_label(end)));
PHy_result = res(find(VarNamesList == 'PHy_0'):find(VarNamesList == PHy_label(end)));

% End-effector velocities
PFxdot_result = res(find(VarNamesList == 'PFxdot_0'):find(VarNamesList == PFxdot_label(end)));
PFydot_result = res(find(VarNamesList == 'PFydot_0'):find(VarNamesList == PFydot_label(end)));
PHxdot_result = res(find(VarNamesList == 'PHxdot_0'):find(VarNamesList == PHxdot_label(end)));
PHydot_result = res(find(VarNamesList == 'PHydot_0'):find(VarNamesList == PHydot_label(end)));

% Contact Configuration
CF_result = res(find(VarNamesList == CF_label(1)):find(VarNamesList == CF_label(end)));
CH_result = res(find(VarNamesList == CH_label(1)):find(VarNamesList == CH_label(end)));

% Contact force result
FFx_result = res(find(VarNamesList == 'FFx_0'):find(VarNamesList == FFx_label(end)));
FFy_result = res(find(VarNamesList == 'FFy_0'):find(VarNamesList == FFy_label(end)));
FHx_result = res(find(VarNamesList == 'FHx_0'):find(VarNamesList == FHx_label(end)));
FHy_result = res(find(VarNamesList == 'FHy_0'):find(VarNamesList == FHy_label(end)));

NetForceX = FFx_result + FHx_result;
NetForceY = FFy_result + FHy_result;

% Torque on the body
FrontTorque_result = (PFx_result - x_result).*FFy_result - (PFy_result - y_result).*FFx_result;
HindTorque_result = (PHx_result - x_result).*FHy_result - (PHy_result - y_result).*FHx_result;

NetTorque = FrontTorque_result + HindTorque_result;

% Foot Bounding Box Result
PFcenterX_result_world = x_result + cos(theta_result)*PFCenterX - sin(theta_result)*PFCenterY;
PFcenterY_result_world = y_result + sin(theta_result)*PFCenterX + cos(theta_result)*PFCenterY;

PHcenterX_result_world = x_result + cos(theta_result)*PHCenterX - sin(theta_result)*PHCenterY;
PHcenterY_result_world = y_result + sin(theta_result)*PHCenterX + cos(theta_result)*PHCenterY;

disp('Result Variables Extracted')
%---------------------------------------------------------------------
% Backup Original Results
%---------------------------------------------------------------------
TimeSeries_origin  = TimeSeries;
x_result_origin    = x_result;       y_result_origin    = y_result;      theta_result_origin    = theta_result;
xdot_result_origin = xdot_result;    ydot_result_origin = ydot_result;   thetadot_result_origin = thetadot_result;
PFx_result_origin  = PFx_result;     PFy_result_origin  = PFy_result;    PFxdot_result_origin   = PFxdot_result;      PFydot_result_origin = PFydot_result;
PHx_result_origin  = PHx_result;     PHy_result_origin  = PHy_result;    PHxdot_result_origin   = PHxdot_result;      PHydot_result_origin = PHydot_result;
CF_result_origin   = CF_result;      CH_result_origin   = CH_result;
FFx_result_origin  = FFx_result;     FFy_result_origin  = FFy_result;    
FHx_result_origin  = FHx_result;     FHy_result_origin  = FHy_result;
NetForceX_origin   = NetForceX;      NetForceY_origin   = NetForceY;
FrontTorque_result_origin = FrontTorque_result;      HindTorque_result_origin = HindTorque_result;
NetTorque_origin   = NetTorque;
PFcenterX_result_world_origin = PFcenterX_result_world;      PFcenterY_result_world_origin = PFcenterY_result_world;
PHcenterX_result_world_origin = PHcenterX_result_world;      PHcenterY_result_world_origin = PHcenterY_result_world;
disp('Original Result Variables Backuped - Result Include Vanishing Phases - End with "origin"');
%-----------------------------------------------------------------------

%-----------------------------------------------------------------------
% Clean Up Time/Control/State Lists - Remove Phases with Zero Length  
%-----------------------------------------------------------------------
TimeStepDiff = diff(TimeSeries);

%States, TimeStepDiff + 1
TimeSeries(find(TimeStepDiff <= 1e-3) + 1) = [];
x_result(find(TimeStepDiff <= 1e-3) + 1) = [];
y_result(find(TimeStepDiff <= 1e-3) + 1) = [];
theta_result(find(TimeStepDiff <= 1e-3) + 1) = [];

xdot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
ydot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
thetadot_result(find(TimeStepDiff <= 1e-3) + 1) = [];

PFcenterX_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PFcenterY_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];

PHcenterX_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PHcenterY_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];

%Inputs, TimeStepDiff due to euler integration
PFx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
PFy_result(find(TimeStepDiff <= 1e-3) + 1) = [];
PHx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
PHy_result(find(TimeStepDiff <= 1e-3) + 1) = [];

NetForceX(find(TimeStepDiff <= 1e-3)) = [];
NetForceY(find(TimeStepDiff <= 1e-3)) = [];

NetTorque(find(TimeStepDiff <= 1e-3)) = [];

% NetForceX(end) = NetForceX(end - 1);
% NetForceY(end) = NetForceY(end - 1);
% NetTorque(end) = NetTorque(end - 1);

%Clear the Input at last time step, for simulation
NetForceX(end) = 0;
NetForceY(end) = 0;
NetTorque(end) = 0;

disp('Removed Variables within Vanished Phases');
disp('===================================================');
disp(' ');
%=======================================================================

%=======================================================================
% Close Diary
diary off