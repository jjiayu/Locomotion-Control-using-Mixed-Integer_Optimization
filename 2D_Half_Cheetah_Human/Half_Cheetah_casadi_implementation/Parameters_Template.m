    %Parameter File

    % Import CasADi related packages
    import casadi.*
    
    %======================================================================
    % Optimization Type Set up
    % Fixed_FreeGait_flag = 1 -> Free Gait Discovery
    % Fixed_FreeGait_flag = 2 -> Fixed Gait Optimization
       
    Fixed_FreeGait_flag = 1;
    
    if Fixed_FreeGait_flag == 2 %Fixed Gait Optimization
        %1 -> Walking-D
        %2 -> Trotting
        %3 -> Galloping
        %4 -> Bounding-D (Symmetric Bounding)
        %5 -> Pronking
        %6-> Walking-S (Asymmetric Walking/Galloping without flying phase) (3-Phases)
        %7-> Bounding-S (Asymmetric Bounding/Galloping without double support phase) (3Phases)
        user_defined_gait = 1;
    end
    
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
    
    %-------------------------------
    %   Morphology Definition: Morpho_change_flag
    %   1-> Yes, Alllow Mophorlogy change 
    %   2-> No Morphology change (Default: Half-CHeetah)
    %-------------------------------
    Morpho_change_flag = 1; 
    
    if Morpho_change_flag == 1 %Allow morphology change
        Morpho_Percentage = 0; % A Humanoid Model
        Morpho_Percentage = Morpho_Percentage/100;
    else %No morphology change (Default Half_Cheetah)
        Morpho_Percentage = 100;
        Morpho_Percentage = Morpho_Percentage/100;
    end
    
    %           Front Foot Default Positions (IN ROBOT FRAME)
    PFCenterX = Morpho_Percentage*(1/2*BodyLength);
    PFCenterY = -(1/2*BodyHeight + DefaultLegLength);
    %           Hind Foot Default Positions (IN ROBOT FRAME)
    PHCenterX = Morpho_Percentage*(-1/2*BodyLength);
    PHCenterY = -(1/2*BodyHeight + DefaultLegLength);
    
    %       Kinematics Bounding Box Constraint
    %Setup Robot Kinematics Properties
    BoundingBox_Width  = 0.4;
    BoundingBox_Height = 0.4;

    %======================================================================
    %Setup Cost
    %   Cost Flag
    %       1-> Minimize Force Squared (Energy Loss)
    %       2-> Minimize Tangential Force (Maximize Robustness)
    %       3-> Minimize Vibration (theta towards terrain slope, thetadot towards zero, ydot towards zero)
    %       4-> Maximize Velocity Smoothness (x_tangent towards desired speed, ydot towards zero, thetadot towards zero)
    %       5-> Minimize Velocity Smoothnes with Fixed Orientatation (add orientation the same as the terrain slope)
    %       6-> Feet Velocity
    cost_flag = 1;
    
    %   Cost Type Flag
    %       1-> Time Integral Only
    %       2-> Time Integral with Scaled Cost
    %       3 -> No Time Integral
    %       4 -> No Time Integral with Scaled Cost
    %       5 -> Infinity Norm
    
    if cost_flag~= 1 && cost_flag ~= 2 && cost_flag ~= 6
        cost_type_flag = 2;
    end
    


    %======================================================================
    %Environment Information
    %----------------------------------------------------------------------
    %   Setup the Terrain Model
    %----------------------------------------------------------------------
    %       Terrain Type
    %           1 -> Flat Terrain (Use this if Terrain Slope = 0);
    %           2 -> Slopes
    TerrainType = 1;
    
    if TerrainType == 1 %Flat Terrain
        %Build Terrain Model for flat terrain 
        h_terrain = 0;
        x_query   = SX.sym('x_query', 1);
        TerrainModel = Function('TerrainModel', {x_query}, {h_terrain});
        terrain_slope_degrees = 0;
        terrain_slope_rad = terrain_slope_degrees/180*pi;
        terrain_slope = tan(terrain_slope_rad);
        disp('----------------------------------------------------');
    elseif TerrainType == 2 %Slope
        %------------------------------------------------------------------
        %Spoecify the terrain slope in Degrees (i.e.: -30, -10, 0, 10, 30...)
        terrain_slope_degrees = 10;
        %------------------------------------------------------------------
        terrain_slope_rad = terrain_slope_degrees/180*pi;
        terrain_slope = tan(terrain_slope_rad);
        x_query   = SX.sym('x_query', 1);
        h_terrain = terrain_slope*x_query;
        TerrainModel = Function('TerrainModel', {x_query}, {h_terrain});
        disp('----------------------------------------------------');
    else %Unknown Scenario
        ME_TerrainType = MException('Initialization:TerrainType',['Unknown Terrain Type']);
        throw(ME_TerrainType)
    end
    %-----------------------------------------------------------------------
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

    %-----------------------------------------------------------------------
    % Friction Cone
    miu = 0.6; %friction coefficient
    %======================================================================

    %======================================================================
    % Task Specifications
    %======================================================================
    %----------------------------------------------------------------------
    %   Specify Desired Speed
    %----------------------------------------------------------------------
    %       Specify the Speed Direction
    %       1-> Horizontal
    %       2-> Tangential to the slope
    SpeedDirection = 1;
    %       Specify the MINIMUM Desired Speed along X-axis (m/s)
    MinSpeed = 0.3;
    %       Specify the MAXIMUM Desired Speed along X-axis (m/s)
    MaxSpeed = 2.5;
    %----------------------------------------------------------------------
    %   Setup other dependent parameters
    %       Specify the Resolution for Scanning the Previously Defined Speed Range (e.g. 0.1, 0.05, 0.02, etc.)
    SpeedResolution = 0.2;
    SpeedList = MinSpeed:SpeedResolution:MaxSpeed;
    %======================================================================

    %======================================================================
    %   Phase Lower Bound setup
    phase_lower_bound_portion = 10; %in Percentage
    phase_lower_bound_portion = phase_lower_bound_portion/100;
    %======================================================================
    
    %======================================================================
    % Time Step and Discretization Parameter Settings
    %----------------------------------------------------------------------
    %   Number of Phases
    if Fixed_FreeGait_flag == 1 %Free Gait Optimization
        NumPhases = 4; 
    elseif Fixed_FreeGait_flag == 2 %Fixed Gait Optimization
        [CF_Sequence, ~, ~] = Gait_Selection(user_defined_gait);
        NumPhases = length(CF_Sequence);
    end
    %   Number of timesteps for each phase
    NumLocalKnots = 10;
    %----------------------------------------------------------------------
    %   Setup other dependent paramters
    %   Total Number of TimeSteps exclude time step 0
    NumKnots = NumPhases*NumLocalKnots;
    %   Parameter tau
    tau_upper_limit = 1; %upper limit of tau --> tau \in [0,1]
    tauStepLength = tau_upper_limit/NumKnots; %discritization step size of tau
    %   Discretization of tau
    tauSeries = 0:tauStepLength:tau_upper_limit;
    tauSeriesLength = length(tauSeries);
    %======================================================================

    %======================================================================
    % Big-M Parameters for Complementarity Constraints
    %======================================================================
    %----------------------------------------------------------------------
    %   Big-M for foot positions in y-axis (meters)
    Mpos_y = 100; 
    %----------------------------------------------------------------------
    %   Big-M for feet velocities
    Mvel = 100;
    %---------------------------------------------------------------------
    %   Velocity Boundary (Abusolute Value) for Foot/End-Effector Velocity in X-axis (In Robot Frame)
    %       Decide Velocity Boundary (Abusolute Value) for Foot/End-Effector in Robot frame in X-axis (In Robot Frame)
    Mvelx = 2.5;
    %       Decide Velocity Boundary (Abusolute Value) for Foot/End-Effector in Robot frame in Y-axis (In Robot Frame)
    Mvely = 2.5;
    %----------------------------------------------------------------------
    %   Big-M/Boundaries for Foot-Ground Reaction Forces
    %----------------------------------------------------------------------
    Mfx = 1000;
    Mfy = 1000;
    %=======================================================================

    %=====================================================================
    % Solver SetUp
    %=====================================================================
    %   Choose Solver
    %---------------------------------------------------------------------
    %       1 -> Knitro
    %       2 -> Bonmin
    SolverNum = 1;
    if SolverNum == 1
        SolverSelected = 'knitro';
    elseif SolverNum == 2
        SolverSelected = 'bonmin';
    end
    %---------------------------------------------------------------------
    %   Solver Dependent Options
    %---------------------------------------------------------------------
    %       Define maximum nodes to be explored
    %           1--> Worst Case Scenario
    %           2 --> User Specified
    %           3 --> Default Value
    NumMaxNodesCases = 1;
    if NumMaxNodesCases == 1  %Worst-case Scenario
        %-------------------------------------------
        %   (Place Holder) Need to change the exponential base when have more
        %   legs in 3D
        %-------------------------------------------
        NumMaxNodes = (2*2)^(NumPhases+1);
    elseif NumMaxNodesCases == 2 %User-specified
        NumMaxNodes = 1000;
    elseif NumMaxNodesCases == 3 %Default Value
        NumMaxNodes = 1e5;
    else
        ME_NumMaxNodes = MException('Initialization:NumMaxNodes','Unexpected Settings of Max Number of Nodes');
        throw(ME_NumMaxNodes)
    end
    %----------------------------------------------------------------------
    %       Define number of multistart solves for each sub-nonlinear
    %       optimizaiton problem, (i.e. 1, 10, 25, 50, 100)
    NumofRuns = 15;
    