% Parameter Template for 2D Quadruped --> Use ANYmal Parameters
%   Sections wrapped by "<--------->" lines are the parameter places

import casadi.*

%==========================================================================
% Optimization Type Set Up
%<---------------------------------------------------------->
%   gait_discovery_switch = 1 -> Discover Gait using MINLP
%   gait_discovery_switch = 2 -> Fixed Gait Optimization
gait_discovery_switch = 1;
%<---------------------------------------------------------->
%==========================================================================

%==========================================================================
% Retrive Gait Sequence if we choose to have fixed gait optimization
if gait_discovery_switch == 2
    %<---------------------------------------------------------->
    % 0      -> User Specified Gait
    % other  -> other gait from gait library
    UserDefinedGaitNumber = 0;
    %<---------------------------------------------------------->
    
    %Build Gait Sequence
    if UserDefinedGaitNumber == 0
        Clf = input('Input Contact Sequence for Left Front  (LF) Foot (i.e. a column vector):\n');
        Clh = input('Input Contact Sequence for Left Hind   (LH) Foot (i.e. a column vector):\n');
        Crf = input('Input Contact Sequence for Right Front (RF) Foot (i.e. a column vector):\n');
        Crh = input('Input Contact Sequence for Right Hind  (RH) Foot (i.e. a column vector):\n');
    else
        error('Optimiation using gait from pre-defined gait library is not implemented for 2D quadruped')
        %[Clf,Clh,Crf,Crh,GaitName] = Gait_Selection(UserDefinedGaitNumber);
    end
end
%==========================================================================

%==========================================================================
%<---------------------------------------------------------->
% Inertia Parameters
m = 34.7; %kg
Iyy = 2.3; %kg m^2
G = 9.80665; %m/s^2
%<---------------------------------------------------------->
%==========================================================================

%==========================================================================
% Kinematics Parameters
%<---------------------------------------------------------->
%   Body Size
BodyLength = 0.63;
BodyHeight = 0.24;
%       Default foot position in Local robot frame
DefaultLegLength = 0.34; %default leg length , distance from the default Leg Y to Torso (LOWER BORDER of the TORSO)
%<---------------------------------------------------------->

%<---------------------------------------------------------->
%   Morphology Definition: 
%   Morpho_change_flag = 1-> Yes, Alllow Mophorlogy change 
%   Morpho_change_flag = 2-> No Morphology change (Default: Half-CHeetah)
Morpho_change_flag = 2; 
%<---------------------------------------------------------->

%   Define Percentage of Morphology change (how much percentage we move a limb from CoM to the border of the torso)
if Morpho_change_flag == 1 %Allow morphology change
    %<---------------------------------------------------------->
    Morpho_Percentage = 0; % A Humanoid Model
    %<---------------------------------------------------------->
    Morpho_Percentage = Morpho_Percentage/100;
else %No morphology change (Default Half_Cheetah)
    Morpho_Percentage = 100;
    Morpho_Percentage = Morpho_Percentage/100;
end
%<---------------------------------------------------------->
%   Kinematics Bounding Box Constraint
BoundingBox_Width  = 0.5;
BoundingBox_Height = 0.14;
%<---------------------------------------------------------->

%   Default Foot Positions
%   Left Front (lf)
PlfCenter = [Morpho_Percentage*(1/2*BodyLength);  -(1/2*BodyHeight + DefaultLegLength)];
%   Left Hind (lh)
PlhCenter = [-Morpho_Percentage*(1/2*BodyLength); -(1/2*BodyHeight + DefaultLegLength)];
%   Right Front (rf)
PrfCenter = [Morpho_Percentage*(1/2*BodyLength);  -(1/2*BodyHeight + DefaultLegLength)];
%   Right Hind (rh)
PrhCenter = [-Morpho_Percentage*(1/2*BodyLength); -(1/2*BodyHeight + DefaultLegLength)];
%==========================================================================

%==========================================================================
% Environment Information
%<---------------------------------------------------------->
%	TerrainType
%       TerrainType = 1 -> Flat Terrain (Use this if Terrain Slope = 0);
%       TerrainType = 2 -> Slopes
TerrainType = 1;
%<---------------------------------------------------------->

if TerrainType == 1 %Flat Terrain
    %Build Terrain Model for flat terrain 
    terrain_slope_degrees = 0;
    terrain_slope_rad = terrain_slope_degrees/180*pi;
    terrain_slope = tan(terrain_slope_rad);
elseif TerrainType == 2 %Slope
    %<---------------------------------------------------------->
    %Spoecify the terrain slope in Degrees (i.e.: -30, -10, 0, 10, 30...)
    terrain_slope_degrees = 10;
    %<---------------------------------------------------------->
    terrain_slope_rad = terrain_slope_degrees/180*pi;
    terrain_slope = tan(terrain_slope_rad);
else %Unknown Scenario
    error('Unknown Terrain Types')
end

%Other Parameters
if TerrainType == 1 %Flat Terrain
    TerrainNorm = [0;1];
    TerrainTangent = [1;0];
elseif TerrainType == 2 % if Stairs, over-write the terrain norm with 
    TerrainNorm = [0;1];
    TerrainNorm = [cos(terrain_slope_rad), -sin(terrain_slope_rad); sin(terrain_slope_rad), cos(terrain_slope_rad)]*TerrainNorm;
    TerrainTangent = [1;0];
    TerrainTangent = [cos(terrain_slope_rad),-sin(terrain_slope_rad);sin(terrain_slope_rad),cos(terrain_slope_rad)]*TerrainTangent;
end

%Build Terrain Model
x_query   = SX.sym('x_query', 1);
h_terrain = terrain_slope*x_query;
TerrainModel = Function('TerrainModel', {x_query}, {h_terrain});

%<---------------------------------------------------------->
%   Friction Cone
miu = 0.6; %friction coefficient
%<---------------------------------------------------------->
%==========================================================================

%==========================================================================
% Task Specifications
%<---------------------------------------------------------->
%   Desired Speed Direction
%    SpeedDirection = 1 -> Horizontal (x-axis of the world frame)
%    SpeedDirection = 2 -> Tangential to the slope
SpeedDirection = 1;
%    Specify the MINIMUM Desired Speed along the Desired Direction (m/s)
MinSpeed = 0.3;
%    Specify the MAXIMUM Desired Speed along the Desired Direction (m/s)
MaxSpeed = 3.5;
%    Specify the Resolution for Scanning the Previously Defined Speed Range (e.g. 0.1, 0.05, 0.02, etc.)
SpeedResolution = 0.2;
%<---------------------------------------------------------->
SpeedList = MinSpeed:SpeedResolution:MaxSpeed;

% Time Step and Discretization Parameter Settings
%   Number of Phases
if gait_discovery_switch == 1 %Free Gait Optimization
    %<---------------------------------------------------------->
    NumPhases = 8; 
    %<---------------------------------------------------------->
elseif gait_discovery_switch == 2 %Fixed Gait Optimization
    %[CF_Sequence, ~, ~] = Gait_Selection(user_defined_gait);
    NumPhases = length(Clf);
end

%<---------------------------------------------------------->
%   Number of timesteps for each phase
NumLocalKnots = 10;
%<---------------------------------------------------------->

%   Setup other dependent paramters
%   Total Number of TimeSteps exclude time step 0
NumKnots = NumPhases*NumLocalKnots;
%   Parameter tau
tau_upper_limit = 1; %upper limit of tau --> tau \in [0,1]
tauStepLength = tau_upper_limit/NumKnots; %discritization step size of tau
%   Discretization of tau
tauSeries = 0:tauStepLength:tau_upper_limit;
tauSeriesLength = length(tauSeries);

%<---------------------------------------------------------->
%   Phase Lower Bound setup
phase_lower_bound_portion = 10; %in Percentage
phase_lower_bound_portion = phase_lower_bound_portion/100;
%<---------------------------------------------------------->

%   Check Phase Lower Bound Setup is reasonable or not
if phase_lower_bound_portion*NumPhases >= 1
    error('Minimum Phase Duration is invalid -> Total Minimum Phase duration is larger than the Allowed Termianl Time')
end
%==========================================================================

%==========================================================================
% Big-M Parameters for Complementarity Constraints
%<---------------------------------------------------------->
%   Big-M for foot positions in y-axis (meters)
M_pos = 100; 
%   Big-M for feet velocities
Mvel = 100;
%<---------------------------------------------------------->

%   Velocity Boundary (Abusolute Value) for Foot/End-Effector Velocity in X-axis (In Robot Frame)
Vmax = [0;0];
%<---------------------------------------------------------->
%       Decide Velocity Boundary (Abusolute Value) for Foot/End-Effector in Robot frame in X-axis (In Robot Frame)
Vmax(1) = 2.5;
%       Decide Velocity Boundary (Abusolute Value) for Foot/End-Effector in Robot frame in Y-axis (In Robot Frame)
Vmax(2) = 2.5;
%<---------------------------------------------------------->

%   Big-M/Boundaries for Foot-Ground Reaction Forces
Mf = [0;0];
%<---------------------------------------------------------->
%Big-M/Boundaries for Foot-Ground Reaction Forces along X-axis (in World Frame)
Mf(1) = 500;
%Big-M/Boundaries for Foot-Ground Reaction Forces along Z-axis (in WOrld Frame)
Mf(2) = 500;
%<---------------------------------------------------------->
%==========================================================================

%==========================================================================
% Cost Function
%<---------------------------------------------------------->
%   cost_flag = 1 -> Minimize Force Squared
%   [Not Implemented] cost_flag = 2 -> Minimize Tangential Force (Maximize Robustness)
%   cost_flag = 3 -> Minimize Vibration (theta towards terrain slope, thetadot towards zero, normal velocity towards zero)
%   [Not Implemented] cost_flag == 4 -> Maximize Velocity Smoothness (x_tangent towards desired speed, ydot towards zero, thetadot towards zero)
%   cost_flag = 5 -> Smooth Motion: 1)tangential speed is constant and close to the desired velocity in tangential direction 2)theta close to theta_slope 3)thetadot close to 0 normal velocity close to 0
%   cost_flag = 6 -> Feet Velocity
%   cost_flag = 7 -> Humanoid Smooth Motion
cost_flag = 1;
%<---------------------------------------------------------->
%==========================================================================

%==========================================================================
% Solver SetUp
%<---------------------------------------------------------->
%   Choose Solver
%    SolverNum = 1 -> Knitro
%    SolverNum = 2 -> Bonmin
SolverNum = 1;
%<---------------------------------------------------------->

if SolverNum == 1
    SolverSelected = 'knitro';
elseif SolverNum == 2
    SolverSelected = 'bonmin';
end

%<---------------------------------------------------------->
%   Solver Dependent Options
%       Define maximum nodes to be explored
%           NumMaxNodesCases = 1--> Worst Case Scenario
%           NumMaxNodesCases = 2 --> User Specified
%           NumMaxNodesCases = 3 --> Default Value
NumMaxNodesCases = 1;
%<---------------------------------------------------------->

if NumMaxNodesCases == 1  %Worst-case Scenario
    NumMaxNodes = (2^4)^(NumPhases+1);
elseif NumMaxNodesCases == 2 %User-specified
    NumMaxNodes = 1000;
elseif NumMaxNodesCases == 3 %Default Value
    NumMaxNodes = 1e5;
else
    error('Unexpected Settings of Max Number of Nodes');
end

%<---------------------------------------------------------->
%	Define number of multistart solves for each sub-nonlinear
%	optimizaiton problem, (i.e. 1, 10, 25, 50, 100)
NumofRuns = 15;
%<---------------------------------------------------------->
%==========================================================================