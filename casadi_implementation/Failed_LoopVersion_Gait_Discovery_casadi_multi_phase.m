% Perdiodic Gait Discovery using Mixed-integer Nonlinear Programming
%   - 2D centroidal planar half-cheetah model
%   - Multiphase Formulation
%       - Cut the time horizon into a few phases
%       - Each phase has the same number of uniformly distributed knots
%       - Scaling the phase interval will scale the time step interval of
%       each knot
%   - For loop implementation to discover the gait for a wide range of
%   desired locomotion speeds

% Check readme for notes and future improvements

% Note: Since Casadi-Type Variables Cannot be Saved, we need to reconstruct
% the terrain model for every time we load parameters

clear;
clc;

%========================================================
% Identfy Data Storage Folder
ExpDirectory = uigetdir;
disp(['Experiment Working Directory: ',ExpDirectory])

%========================================================
% Import CasADi related packages
import casadi.*

%========================================================
% Command Line Logging
%========================================================
diary off
ExpSummaryLogFile = strcat('Gait_Discovery_Experiments_Task_and_Parameters_Settings',datestr(datetime('now'), 30));
diary([ExpDirectory,'/',ExpSummaryLogFile]);

%=========================================================
% Display Script Information
disp('====================================================');
disp('Task and Parameter Setup for Gait Discovery - Discovering Optimal Gait for Different Desired Locomotion Speed using Loops:');
disp('====================================================');
disp('CasADi Implementation');
disp('Periodic Gait Discovery for 2D Centroidal Planar Half Cheetah Model - using Mixed-integer Nonlinear Optimization');
disp('Multi-Phase Formulation:');
disp('Optimize over State, Control, Gait Sequence, and Switching Time');
disp('----------------------------------------------------');
disp('Date and Time:');
disp(datetime('now'));
disp('----------------------------------------------------');
disp(['Correspondent Log File Name: ', ExpSummaryLogFile]);
disp('====================================================');
disp(' ');

%========================================================
% Parameter SetUp
%========================================================
disp('====================================================');
ParameterSetUpFlag = input('Select 1-> Setup the Task Description and Parameters from Scratch, or, 2 -> Use Existing Task Description and Paramters: \n');
disp('----------------------------------------------------');
if ParameterSetUpFlag == 1
    %========================================================
    % Display Info
    disp('Set-up Task Description and Parameters from Scratch');
    disp('----------------------------------------------------');

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
    BoundingBox_Width  = input('Define Kinematics Bounding Box Width (i.e. 0.4, 0.6):\n');
    disp('----------------------------------------------------');
    BoundingBox_Height = input('Define Kinematics Bounding Box Hiehgt (i.e. 0.2, 0.25, 0.3):\n');
    disp('----------------------------------------------------');
    %BoundingBox_Width = 0.6;
    %BoundingBox_Height = 0.6;
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
    MinSpeed = input('Specify the MINIMUM Desired Speed along X-axis (m/s): \n');
    disp('----------------------------------------------------');
    MaxSpeed = input('Specify the MAXIMUM Desired Speed along X-axis (m/s): \n');
    disp('----------------------------------------------------');
    SpeedResolution = input('Specify the Resolution for Scanning the Previously Defined Speed Range (e.g. 0.1, 0.05, 0.02, etc.):\n');
    disp('----------------------------------------------------');
    SpeedList = MinSpeed:SpeedResolution:MaxSpeed;
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
    % Save Task and Parameter Set Ups
    save([ExpDirectory,'/TaskandParameterSetup.mat'])

elseif ParameterSetUpFlag == 2
    %========================================================
    % Display Info
    disp('Load Task and Parameter Set up from Saved File in the Experiment Directory');
    disp('----------------------------------------------------');
    
    %========================================================
    % Load Parameters
    load([ExpDirectory,'/TaskandParameterSetup.mat']);
    
    %========================================================
    % Display Important Setups
    
    %======================================================================

    %======================================================================
    %   Kinematics Constraint Parameters
    %======================================================================
    disp('====================================================');
    disp('Robot Kinematics Properties: ');
    disp('----------------------------------------------------');
    disp(['Bounding Box Width: ',num2str(BoundingBox_Width)]);
    disp('----------------------------------------------------');
    disp(['Bounding Box Height: ',num2str(BoundingBox_Height)]);
    disp('----------------------------------------------------');
    %======================================================================

    %======================================================================
    %Environment Information
    %----------------------------------------------------------------------
    %   Display some info
    %----------------------------------------------------------------------
    disp('====================================================');
    disp('Terrain Information: ')
    disp('----------------------------------------------------');
    if TerrainType == 1 %Flat Terrain
        disp('Selected Flat Terrain');
    elseif TerrainType == 2 %Slope
        disp('Selected Slope Terrain');
        ME_SlopeUnavailable = MException('Initialization:SlopeUnavailable',['Slope Terrain Un-implemented']);
    else %Unknown Scenario
        ME_TerrainType = MException('Initialization:TerrainType',['Unknown Terrain Type']);
        throw(ME_TerrainType)
    end
    
    %Re-build Terrain Model
    %         Build Terrain Model Function --> CasADi If-Else implementation
    %           Define CasADi symbolic variables
    h_terrain = 0;
    x_query   = SX.sym('x_query', 1);
    for i = 1:MaxNumStairs
        h_terrain = h_terrain + if_else(x_query < sum(HeightChangingPlaces(1:i)), 0, LevelChanges(i));
    end
    TerrainModel = Function('TerrainModel', {x_query}, {h_terrain});
    disp('Due to Terrain Model Involve Casadi-Type Variable that Cannot be Saved/Loaded, We Re-construct the Terrain Model Here.')
    
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
    disp(['Desired Locomotion Speed Range: ',num2str(MinSpeed), ' - ', num2str(MaxSpeed)]);
    disp('----------------------------------------------------');
    disp(['Resolution/Incremental Speed Change in the Defined Speed Range: ', num2str(SpeedResolution)]);
    disp('----------------------------------------------------');
    %----------------------------------------------------------------------
    %   Terminal time
    %----------------------------------------------------------------------
    disp('Terminal Time:')
    if Tend_flag == 1 %Optimize Terminal Time
        disp(['Fixed Terminal Time/Stride Frequency: ', num2str(Tend), 's']);
    elseif Tend_flag == 2
        disp('Terminal Time (Tend) is Set as Free Variables')
        disp(['Upper Bound of the Terminal Time: ', num2str(Tend_Bound), 's']);
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
    disp(['Number of Phases: ', num2str(NumPhases)]);
    disp('----------------------------------------------------');
    %   Number of timesteps for each phase
    disp(['Number of Knots in Each Phase: ', NumLocalKnots]);
    disp('----------------------------------------------------');
    %   Total Number of TimeSteps exclude time step 0
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
    disp(['Configured Big-M Value for Foot/End-Effector Velocity for X-axis: ', num2str(Mvelx), ' m/s']);
    disp('----------------------------------------------------');
    %---------------------------------------------------------------------
    %       big-M for Y-axis Foot/End-Effector Velocity
    %---------------------------------------------------------------------
    disp(['Configured Big-M Value for Foot/End-Effector Velocity for Y-axis: ', num2str(Mvely), ' m/s']);
    disp('----------------------------------------------------');
    %----------------------------------------------------------------------
    %   Big-M for Foot-Ground Reaction Forces
    disp(['Big-M for Contact Forces in X-axis: ',num2str(Mfx)])
    disp('----------------------------------------------------');
    disp(['Big-M for Contact Forces in Y-axis: ',num2str(Mfy)])
    disp('----------------------------------------------------');
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
    disp(['Selected Solver: ', SolverSelected])
    disp('----------------------------------------------------');
    %   Solver Dependent Options
    disp('Solver Dependent Options:')
    disp('----------------------------------------------------');
    %       Define maximum nodes to be explored
    if NumMaxNodesCases == 1  %Worst-case Scenario
        %-------------------------------------------
        %   (Place Holder) Need to change the exponential base when have more
        %   legs in 3D
        %-------------------------------------------
        disp(['Selected Worst-case Scenarios to Explore ', num2str(NumMaxNodes), ' Nodes']);
    elseif NumMaxNodesCases == 2 %User-specified
        disp(['User Defined Maximum Number of Nodes ', num2str(NumMaxNodes), ' Nodes']);
    elseif NumMaxNodesCases == 3 %Default Value
        disp(['Selected Default Case to Explore ', num2str(NumMaxNodes), ' Nodes'])
    else
        ME_NumMaxNodes = MException('Initialization:NumMaxNodes','Unexpected Settings of Max Number of Nodes');
        throw(ME_NumMaxNodes)
    end
    disp('====================================================');
    disp(' ')
    %======================================================================
    
else
    error('Initialization:WrongParameterSetupFlag','Undefined Flag Signal for Parameter and Task Descriptor Set-ups');
end

diary off

%======================================================================
% Do the Actual Computation
%======================================================================
