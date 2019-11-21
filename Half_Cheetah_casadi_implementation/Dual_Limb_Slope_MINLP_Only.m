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

diary off

% Import CasADi related packages
import casadi.*

%cd /home/jiayu/Desktop/Locomotion-Control-using-Mixed-Integer_Optimization/casadi_implementation/

Paras_Define_Method = input('Define the way of defining parameters: 1 -> Take from File 2-> Define Manually: \n');

if Paras_Define_Method == 1 %Take from file, load parameters
    % Identfy Data Storage Folder
    
    Terminator_or_Laptop = input('Define what device we are using now: 1-> Laptop/Desktop 2-> Terminator: \n');
    if Terminator_or_Laptop == 1 %Laptop/Desktop
        ParamFileDir = uigetdir;
    elseif Terminator_or_Laptop == 2 %Terminator
        ParamFileDir = input('Manually Define Folder Path Storing Parameter File (quote with ''): \n');
        [cmd_status,cmdoutput] = system('screen -ls')
        %screen_name = input('Type Screen Name (quote using ''):\n');
        %email_message = input('Type Email Message (i.e. like robot type (half cheetah, humanoid, terrain_type, etc., any features, using '' to quote)):\n');
    end
    
    %ParamFileDir = uigetdir;
    disp(['Directory Storing Parameter File: ', ParamFileDir])
    
    %Load Parameter File
    run([ParamFileDir,'/Parameters.m']);
end

%!Select Working Directory then, if we choose to define paramters
%manually, for the case of predefined parameters, the Expdirecotry will be
%decided via stride period
if Paras_Define_Method == 2 
    ExpDirectory = uigetdir;
    disp(['Experiment Working Directory: ', ExpDirectory])
end

% Display Pre-defined Setup or Input Parameters
%----------------------------------------------------------------------
%   Terminal time Setup (Only for Cases when load parameter from File)
%----------------------------------------------------------------------
if Paras_Define_Method == 1 %load parameter from file
    disp('=====================================================');
    disp('Define Terminal Time:')
    Tend_flag = input('Optimization of Terminal Time (Tend): \n1 -> Fixed, 2 -> Left as Free Variable \n');
    if Tend_flag == 1 %Optimize Terminal Time,
        disp('Fixed Terminal Time');
        Tend = input('Input Termianl Time (e.g. 1s): \n');
        
        %Define the Experiment Direcotry if we load parameter from file
        if exist([ParamFileDir,'/4Phases_StridePeriod_',num2str(Tend)],'dir') == 1
            ExpDirectory = [ParamFileDir,'/4Phases_StridePeriod_',num2str(Tend)];
        else %or mkdir
            mkdir([ParamFileDir,'/4Phases_StridePeriod_',num2str(Tend)]);
            ExpDirectory = [ParamFileDir,'/4Phases_StridePeriod_',num2str(Tend)];
        end
 
    elseif Tend_flag == 2
        disp('Terminal Time (Tend) is Set as Free Variables')
        Tend_Bound = input('Specify Upper Bound of Terminal Time (i.e. 1,2..(s)):\n');
        ExpDirectory = ParamFileDir;
    else
        ME_Tend = MException('Initialization:Tend_flag','Unknown Indicator for Determining the on/off of Terminal Time Optimization');
        throw(ME_Tend)
    end
    disp('=====================================================');
end
 
%========================================================
% Command Line Logging
%diary off
TaskParameterLog_filename = strcat('MultiMINLPRuns-Periodical-Loco-log-', datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
diary([ExpDirectory, '/', TaskParameterLog_filename]);

% %=========================================================
% % Display screen number if the computation is on terminator
% if Terminator_or_Laptop == 2 %on terminator
%     disp('====================================================');
%     disp('Screen Number:')
%     disp(cmdoutput)
%     disp('====================================================');
% end
% %=========================================================

%=========================================================
% Display Script Information
disp('====================================================');
disp('Genearl Information:');
disp('====================================================');
disp('Parameter Loaded From File')
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
disp(['Correspondent Log File Name: ', TaskParameterLog_filename, 'in the Directory: ', ExpDirectory]);
disp('====================================================');
disp('Using Rotated Force Limits')
disp('====================================================');
disp(' ');
%=====================================================================


%======================================================================
%Inertia Parameters(Information from MIT Cheetah 3)
disp('====================================================');
disp(['Inertial Parameters'])
disp('====================================================');
if Paras_Define_Method == 1 %Take fro file
    disp(['mass: ',num2str(m),'(kg)']);
    disp(['Z-axis Inertial - Izz: ',num2str(I), '(kg m^2)']);
    disp(['Gravaty: ',num2str(G), 'm/s']);
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
    %m = 45; %kg
    %I = 2.1; %kg m^2 Izz
    m = input('Input Body Mass (i.e. 60) (kg): \n');
    I = input('Input Body Inertia (i.e. 2.5): \n');  
    G = 9.80665; %m/s^2
end
%======================================================================

%======================================================================
%   Kinematics Constraint Parameters
%======================================================================
disp('====================================================');
disp(['Kinematics Constraint Parameters'])
disp('====================================================');
%       Body Size
if Paras_Define_Method == 1 %Take fro file
    disp('Body Size')
    disp('----------------------------------------------------');
    disp(['Body Length: ',num2str(BodyLength),' (m)']);
    disp(['Body Height: ',num2str(BodyHeight),' (m)']);
    disp('----------------------------------------------------');
    disp(['Default Limb Length:', num2str(DefaultLegLength)]);
    disp('----------------------------------------------------');
    disp('Morphology Set up:')
    if Morpho_change_flag == 1
        if Morpho_Percentage == 0
            disp(['Use A Humanoid Model; Morphology Percentage = ', num2str(Morpho_Percentage*100),'%']);
        else
            disp(['Allow Morphology Change ', num2str(Morpho_Percentage*100),'%', ' from the center of the body to the edge of the body'])
        end
    end
    disp('----------------------------------------------------');
    disp('Robot Kinematics Bounding Box')
    disp('----------------------------------------------------');
    disp(['Bounding Box Width: ',num2str(BoundingBox_Width),' (m)']);
    disp(['Bounding Box Height: ',num2str(BoundingBox_Height),' (m)']);
    disp('----------------------------------------------------');
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
    %       Body Size
    %BodyLength = 0.6;
    %BodyHeight = 0.2;
    BodyLength = input('Input Body Length (i.e. 0.15): \n');
    BodyHeight = input('Input Body Height (i.e. 0.6): \n');
    %       Default foot position in Local robot frame
    %DefaultLegLength = 0.45; %default leg length , distance from the default Leg Y to Torso (LOWER BORDER of the TORSO)
    DefaultLegLength = input('Input Default Leg Length (i.e. 1.1): \n');
    
    %       Decide if there are morphology changes
    Morpho_change_flag = input('Decide if we allow morphology change (range from humanoid to half-cheetah): 1-> Yes, Alllow Mophorlogy change 2-> No Morphology change (Default: Half-CHeetah)\n');
    if Morpho_change_flag == 1 %Yes allow mophorlogy change
        Morpho_Percentage = input('Input How much mophorlogy change (displacement from center to edge of the body) we allow (i.e. 0 - 100%, from humanoid to half_cheetah, Do Not Type Percentage Symbol):\n');
        if Morpho_Percentage == 0
            disp('------------------------------------');
            disp('A Humanoid Model!');
            disp('------------------------------------');
        end
        Morpho_Percentage = Morpho_Percentage/100;
        %           Front Foot Default Positions (IN ROBOT FRAME)
        PFCenterX = Morpho_Percentage*(1/2*BodyLength);
        PFCenterY = -(1/2*BodyHeight + DefaultLegLength);
        %           Hind Foot Default Positions (IN ROBOT FRAME)
        PHCenterX = Morpho_Percentage*(-1/2*BodyLength);
        PHCenterY = -(1/2*BodyHeight + DefaultLegLength);
    elseif Morpho_change_flag == 2 %No mophorlogy change allowed
        %           Front Foot Default Positions (IN ROBOT FRAME)
        PFCenterX = 1/2*BodyLength;
        PFCenterY = -(1/2*BodyHeight + DefaultLegLength);
        %           Hind Foot Default Positions (IN ROBOT FRAME)
        PHCenterX = -1/2*BodyLength;
        PHCenterY = -(1/2*BodyHeight + DefaultLegLength);
    end
    %       Kinematics Bounding Box Constraint
    disp('====================================================');
    disp('Setup Robot Kinematics Properties: ')
    disp('----------------------------------------------------');
    BoundingBox_Width  = input('Define Kinematics Bounding Box Width (i.e. 0.4, 0.6):\n');
    disp('----------------------------------------------------');
    BoundingBox_Height = input('Define Kinematics Bounding Box Hiehgt (i.e. 0.2, 0.25, 0.3):\n');
    disp('----------------------------------------------------');
end

%======================================================================

%======================================================================
%Cost Setup
if Paras_Define_Method == 1 %Take fro file
    disp('====================================================');
    disp('Cost Setup')
    disp('----------------------------------------------------');
    disp('Selected Cost:');
    if cost_flag == 1
    disp('1-> Minimize Force Squared (Energy Loss)');
    elseif cost_flag == 2
    disp('2-> Minimize Tangential Force (Maximize Robustness)');
    elseif cost_flag == 3
    disp('3-> Minimize Vibration (theta towards terrain slope, thetadot towards zero, ydot towards zero)');
    elseif cost_flag == 4
    disp('4-> Maximize Velocity Smoothness (x_tangent towards desired speed, ydot towards zero, thetadot towards zero)');
    elseif cost_flag == 5
    disp('5-> Minimize Velocity Smoothnes with Fixed Orientatation (add orientation the same as the terrain slope)');
    elseif cost_flag == 6
    disp('6-> Feet Velocity');
    elseif cost_flag == 7
    disp('7-> Humanoid Smooth Motion')
    end

    if cost_flag~= 1 && cost_flag ~= 2 && cost_flag ~= 6
    disp('----------------------------------------------------');
    disp('Cost Formulation Type: ')
    if cost_type_flag ==1
        disp('1-> Time Integral Only');
    elseif cost_type_flag == 2
        disp('2-> Time Integral with Scaled Cost');
    elseif cost_type_flag == 3
        disp('3 -> No Time Integral')
    elseif cost_type_flag == 4
        disp('4 -> No Time Integral with Scaled Cost')
    elseif cost_type_flag == 5
        disp('5 -> Infinity Norm')
    end
    end
    disp('====================================================');
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
    disp('====================================================');
    disp('Set up Cost Terms:')
    disp('----------------------------------------------------');
    cost_flag = input('Decide Cost: \n 1-> Minimize Force Squared (Energy Loss) \n 2-> Minimize Tangential Force (Maximize Robustness) \n 3-> Minimize Vibration (theta towards terrain slope, thetadot towards zero, ydot towards zero) \n 4-> Maximize Velocity Smoothness (x_tangent towards desired speed, ydot towards zero, thetadot towards zero) \n 5-> Minimize Velocity Smoothnes with Fixed Orientatation (add orientation the same as the terrain slope) \n 6-> Feet Velocity \n 7-> Humanoid Smooth Motion (theta, thetadot towards zero) \n');
    %cost_flag = input('Decide Cost: \n 1-> Minimize Force Squared \n 2-> Minimize Body Vibration (ydot, theta towards terrain slope, thetadot) \n 3-> Minimize tangential speed (along the terrain) to the deisred speed at every knot \n 4-> Minimize tangential speed (to the desired speed), normal axis speed (towards zero) \n 5 -> Minimize tangential speed (to the desired speed), normal speed (tp zero), angular speed thetadot (to the terrain slope) \n 6-> Minimize Body Vibration with Constant Tangential Speed (ydot, theta towards terrain slope, thetadot, xdot toward desired tangetial speed): \n');
    if cost_flag~= 1 && cost_flag ~= 2 && cost_flag ~= 6
        disp('----------------------------------------------------');
        cost_type_flag = input('Decide the Type (Formulation-wise) of the cost (2 and 4 are prefered): 1-> Time Integral Only \n 2-> Time Integral with Scaled Cost \n 3 -> No Time Integral \n 4 -> No Time Integral with Scaled Cost \n 5 -> Infinity Norm \n');
    end
    disp('====================================================');
end

%======================================================================
%Environment Information
%----------------------------------------------------------------------
%   Display some info
%----------------------------------------------------------------------
if Paras_Define_Method == 1 %Take fro file
    disp('====================================================');
    disp('Terrain Model: ')
    disp('----------------------------------------------------');
    if TerrainType == 1
        disp('Flat Terrain');
    elseif TerrainType == 2
        disp('Slopes');
    end
    disp('Terrain Norm is: ');
    TerrainNorm
    TerrainTangent = [cos(terrain_slope_rad),-sin(terrain_slope_rad);sin(terrain_slope_rad),cos(terrain_slope_rad)]*[1;0];
    disp('Terrain Tangent is: ');
    TerrainTangent
    disp('----------------------------------------------------');
    disp('----------------------------------------------------');
    disp(['Friction Cone: ', num2str(miu)]);
    disp('====================================================');
    disp(' ')
    
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
    disp('====================================================');
    disp('Setup Terrain Model: ')
    disp('----------------------------------------------------');
    %----------------------------------------------------------------------
    %   Setup the Terrain Model
    %----------------------------------------------------------------------
    TerrainType = input('Specify the Terrain Type: 1 -> Flat Terrain (Use this if Terrain Slope = 0); 2 -> Slopes\n');

    if TerrainType == 1 %Flat Terrain
        disp('Selected Flat Terrain');
        %Build Terrain Model for flat terrain 
        h_terrain = 0;
        x_query   = SX.sym('x_query', 1);
        TerrainModel = Function('TerrainModel', {x_query}, {h_terrain});
        terrain_slope_degrees = 0;
        terrain_slope_rad = terrain_slope_degrees/180*pi;
        terrain_slope = tan(terrain_slope_rad);
        disp('----------------------------------------------------');
    elseif TerrainType == 2 %Slope
        disp('Selected Slope Terrain');
        terrain_slope_degrees = input('Spoecify the terrain slope in Degrees (i.e.: -30, -10, 0, 10, 30...): \n');
        terrain_slope_rad = terrain_slope_degrees/180*pi;
        terrain_slope = tan(terrain_slope_rad);
        disp(['Defined terrain slope: ',num2str(terrain_slope)]);
        %check if terrain calulation is correct: atan(terrain_slope)/pi*180
        %Build Terrain Model for slope terrain
        x_query   = SX.sym('x_query', 1);
        h_terrain = terrain_slope*x_query;
        TerrainModel = Function('TerrainModel', {x_query}, {h_terrain});
        disp('----------------------------------------------------');
    else %Unknown Scenario
        ME_TerrainType = MException('Initialization:TerrainType',['Unknown Terrain Type']);
        throw(ME_TerrainType)
    end

    %-----------------------------------------------------------------------
    %   Plot Terrain Model
    PlotTerrainFlag = input('Plot the Terrain Model? 1 -> Yes; 2 -> No\n');
    if PlotTerrainFlag == 1 %Yes, Plot the terrain model    
        terrainx = linspace(-2, 10, 1e4);
        terrainy = full(TerrainModel(terrainx));
        plot(terrainx,terrainy,'LineWidth',2)
        ylim([min(terrainy)-1,max(terrainy)+1])
        disp('----------------------------------------------------');
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
    disp('Terrain Norm is: ');
    TerrainNorm
    disp('Terrain Tangent is: ');
    TerrainTangent
    disp('----------------------------------------------------');
    miu = 0.6; %friction coefficient
    disp('----------------------------------------------------');
    disp(['Friction Cone: ', num2str(miu)]);
    disp('====================================================');
    disp(' ')
end
%======================================================================

%======================================================================
% Task Specifications
%======================================================================
%   Display Information
%----------------------------------------------------------------------
disp('====================================================');
disp('Task Specification:')
disp('----------------------------------------------------');
if Paras_Define_Method == 1 %Take fro file
    %----------------------------------------------------------------------
    %   Specify Desired Speed
    %----------------------------------------------------------------------
    disp(['Minimal Speed: ',num2str(MinSpeed)]);
    disp(['Maximum Speed: ',num2str(MaxSpeed)]);
    disp(['Speed Resolution: ',num2str(SpeedResolution)])
    %----------------------------------------------------------------------
    %   Terminal time
    %----------------------------------------------------------------------
    if Tend_flag == 1 %Fixed Terminal Time
        disp(['Fixed Terminal Time: ',num2str(Tend)]);
    elseif Tend_flag == 2
        disp(['Terminal Time as an Optimization Variable'])
    end
    disp('----------------------------------------------------');
    disp(['Phase Duration Lower Bound: ',num2str(phase_lower_bound_portion*100),'%'])
    disp('----------------------------------------------------');
    disp(' ')
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
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
    Tend_flag = input('Optimization of Terminal Time (Tend): \n1 -> Fixed, 2 -> Left as Free Variable \n');
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
    phase_lower_bound_portion = input('Input Phase Lower Bound (in percentage of total motion duration: 2.5% - no need to type percentage symbol): \n');
    phase_lower_bound_portion = phase_lower_bound_portion/100
    disp(['Phase Duration Lower Bound: ',num2str(phase_lower_bound_portion*100),'%'])
    disp('----------------------------------------------------');
    disp(' ')
end
%======================================================================

%======================================================================
% Time Step and Discretization Parameter Settings
%----------------------------------------------------------------------
%   Display Info
%----------------------------------------------------------------------
if Paras_Define_Method == 1 %Take fro file
    disp('====================================================');
    disp('Temporal and Discretization Setup:');
    disp('----------------------------------------------------');
    %   Number of Phases
    disp(['Number of Phase: ',num2str(NumPhases)]);
    disp('----------------------------------------------------');
    %   Number of timesteps for each phase
    disp(['Number of Knots for each phase:' ,num2str(NumLocalKnots)]);
    disp('----------------------------------------------------');
    %   Print some Info
    disp('Resultant Discretization:')
    disp(['Knot/Discretization Step Size of tau: ', num2str(tauStepLength)]);
    disp(['Number of Knots/Discretization of tau (from 0 to ', num2str(tau_upper_limit), ': ', num2str(tauSeriesLength), ') (Number of Knots/Discretization in Each Phase * Number of Phases(NumKnots) + 1)']);
    disp('====================================================');
    disp(' ')
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
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
end
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
disp('Big-M Setup/Limits of Feet Velocity and Forces')
disp('====================================================');
if Paras_Define_Method == 1 %Take fro file
    disp(['Big-M for Foot/End-Effector Velocities: ', num2str(Mvel)]);
    disp('----------------------------------------------------');
    disp(['Velocity Boundary (Abusolute Value) for Foot/End-Effector Velocity in X-axis (In Robot Frame): ',num2str(Mvelx)]);
    disp(['Velocity Boundary (Abusolute Value) for Foot/End-Effector Velocity in Y-axis (In Robot Frame): ',num2str(Mvely)]);
    disp('----------------------------------------------------');
    disp('Big-M for Contact Forces');
    disp('----------------------------------------------------');
    disp(['Big-M for Foot/End-Effector Forces in X-axis: ',num2str(Mfx)]);
    disp(['Big-M for Foot/End-Effector Forces in X-axis: ',num2str(Mfy)]);
    disp('====================================================');
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
    disp('====================================================');
    disp('Big-M Setup/Limits of Feet Velocity and Forces')
    disp('====================================================');
    %----------------------------------------------------------------------
    %   Big-M for foot positions in y-axis (meters)
    Mpos_y = 100; 
    %   ------------------------------------------------------
    %   Big-M for feet velocities
    Mvel = 100;
    %   ------------------------------------------------------
    %   Velocity Boundary (Abusolute Value) for Foot/End-Effector Velocity in X-axis (In Robot Frame)
    disp('Velocity Boundary (Abusolute Value) for Foot/End-Effector in Robot frame')
    disp('----------------------------------------------------');
    Mvelx = input('Decide Velocity Boundary (Abusolute Value) for Foot/End-Effector in Robot frame in X-axis (In Robot Frame): \n');
    Mvely = input('Decide Velocity Boundary (Abusolute Value) for Foot/End-Effector in Robot frame in Y-axis (In Robot Frame): \n');
    disp('----------------------------------------------------');
    disp('Big-M for Contact Forces')
    Mfx = input('Input Big-M/Boundaries for Foot-Ground Reaction Forces along X-axis (in World Frame) (e.g. 200,300,1000,1e5):\n');
    disp('----------------------------------------------------');
    Mfy = input('Input Big-M/Boundaries for Foot-Ground Reaction Forces along Y-axis (in WOrld Frame) (e.g. 200,300,1000,1e5):\n');
    disp('====================================================');
end

%=====================================================================
% Solver SetUp
%=====================================================================
disp('====================================================');
disp('Solver Setups:')
disp('====================================================');
if Paras_Define_Method == 1 %Take fro file
    %---------------------------------
    %   Choose Solver
    disp('Solver Selection: ')
    if SolverNum == 1
        disp('Solver Selected: Knitro');
    elseif SolverNum == 2
        disp('Solver Selected: Bonmin');
    end
    disp('----------------------------------------------------');
    %   Solver Dependent Options
    disp('Solver Dependent Options:')
    disp('----------------------------------------------------');
    disp(['Number of Maximum Nodes to be solved: ',num2str(NumMaxNodes)]);
    disp(['NUmber of Maximum Optimization Runes: ',num2str(NumofRuns)]);
    %   Stop Diary
    diary off
    %-----------------------------------------------------------
    Continue_flag = input('Satisfied with the paramter setting? 1-> yes, 2-> no \n');
    if Continue_flag == 2
        ME_UserStop = MException('Initialization:UserStop',['User is unsatisfied with the parameter setup']);
        throw(ME_UserStop)
    end
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
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
    %---------------------------------------------------------------------
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
        NumMaxNodes = (2*2)^(NumPhases+1);
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
    disp('----------------------------------------------------');
    %       Define number of multistart solves for each sub-nonlinear
    %       optimizaiton problem
    NumofRuns = input('Specify Number of Runs for the MINLP Programming (i.e. 1, 10, 25, 50, 100): \n');
    disp('====================================================');
    disp(' ')
    %   Stop Diary
    diary off
    %======================================================================
end

%======================================================================
%   Run the big computation loop
%======================================================================
run('RotatedForceLimit_Computation_Loop_MINLP_Only.m');
%======================================================================

disp('===================================================');
disp('All Experiments Finished')
disp('===================================================');

if exist('Terminator_or_Laptop','var') == 1 && Terminator_or_Laptop == 2 %terminator
    %Send email to notify the success
    split_cmdoutput = splitlines(cmdoutput);
    sendmail('Jiayi.Wang@ed.ac.uk',[split_cmdoutput{2},' is done~']);
end

