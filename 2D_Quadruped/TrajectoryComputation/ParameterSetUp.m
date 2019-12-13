% Parameter Setup for the 2D quadruepd robot

%Import CaSADi related packages
import casadi.*

%Add current path
addpath(pwd);

%--------------------------------------------------------------------------
% Determine Parameter Specification Method
%--------------------------------------------------------------------------
disp('=====================================================');
Paras_Define_Method = input('Define the way of defining parameters: 1 -> Take from File 2-> Define Manually: \n');
disp(' ')

if Paras_Define_Method == 1 %Load Parameters from files
    %Identify Parameter file storage folder
    Remote_Computing_Flag = input('Specify which device we are using now: 1-> Laptop/Desktop 2-> Remote Computer (i.e. Terminator): \n');
        
    if Remote_Computing_Flag == 1 %Laptop/Desktop
        ParamFileDir = uigetdir;
    elseif Remote_Computing_Flag == 2 %Remote Computer
        ParamFileDir = input('Manually Define Folder Path Storing Parameter File (quote with \''): \n');
        %get screen number for future use of sending finishing emails
        [cmd_status,cmdoutput] = system('screen -ls')
    else
        error('Wrong selection for computing machines')
    end
    
    disp(['Parameter File Stores in: ', ParamFileDir])
    
    %Load Parameter File
    run([ParamFileDir,'/Parameters.m']);
    disp('Parameter loaded successfully')
    disp('=====================================================');
end

%--------------------------------------------------------------------------
% Specify Experiment Wokring Directory (ExpDirectory), if we choose to define parameters
% manually
% The ExpDirectory for the cases where parameters are loaded from file will
% be decided later
%--------------------------------------------------------------------------
if Paras_Define_Method == 2 %Specify Paramters Manually
    ExpDirectory = uigetdir;
    disp(['Experiment Working Directory: ', ExpDirectory])
end
disp('=====================================================');
disp(' ')

%--------------------------------------------------------------------------
% Decide Experiment Type (Gait Discovery or Fixed Gait Motion Optimization)
%--------------------------------------------------------------------------
disp('=====================================================');
if Paras_Define_Method == 2 %Specify Paramters Manually
    gait_discovery_switch = input('Decide 1-> Gait Discovery via MINLP; 2-> Computing Locomotion Plans for a Fixed Gait Sequence: \n');
    
    if gait_discovery_switch == 2 %fixed gait optimization
        UserDefinedGaitNumber = input('Select the gait: \n 0 -> User Input \n 1-> Walking ....\n');
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
end

if (gait_discovery_switch ~= 1) && (gait_discovery_switch ~= 2)
    error('Unknow flag for Computation Type (Neither Gait Discovery using MINLP nor Motion Optimization using Fixed Gait)')
end

disp('=====================================================');
disp(' ')

%--------------------------------------------------------------------------
%   Terminal time Setup (Only for Cases when load parameter from File, needed for determining ExpDirectory)
%----------------------------------------------------------------------
if Paras_Define_Method == 1 %Load Parameters from File
    disp('=====================================================');
    disp('Define Terminal Time:')
    Tend_flag = input('Optimization of Terminal Time (Tend): \n1 -> Pre-defined (Fixed), 2 -> Left as Free Variable \n');
   
    if Tend_flag == 1 %Optimize Terminal Time
        % Specify Terminal Time now for the case when we load parameter
        % from file, since we need Terminal Time to make ExpDirectories to
        % Sotreo results. The Terminal Time for the cases when define
        % parameters manually will be set later
        disp('Fixed Terminal Time');
        Tend = input('Input Termianl Time (e.g. 1s): \n');
        
        %Decide ExpDirectory for the case when we load parameters from file
        %For the cases where the parameters is defined mannually, the
        %ExpDirectory is defined beforehand
        if gait_discovery_switch == 1 %Gait Discovery Using MINLP
            if exist([ParamFileDir,'/StridePeriod_',num2str(Tend)],'dir') == 1
                ExpDirectory = [ParamFileDir,'/StridePeriod_',num2str(Tend)];
            else %or mkdir
                mkdir([ParamFileDir,'/StridePeriod_',num2str(Tend)]);
                ExpDirectory = [ParamFileDir,'/StridePeriod_',num2str(Tend)];
            end
        elseif gait_discovery_switch == 2 %Motion Optimization with Fixed Gait
            if exist([ParamFileDir,'/','StridePeriod_',num2str(Tend)],'dir') == 1
                ExpDirectory = [ParamFileDir,'/','StridePeriod_',num2str(Tend)];
            else %or mkdir
                mkdir([ParamFileDir,'/','StridePeriod_',num2str(Tend)]);
                ExpDirectory = [ParamFileDir,'/','StridePeriod_',num2str(Tend)];
            end
        end
    elseif Tend_flag == 2
        error('Optimization with free Terminal Time is not implemented')
        %disp('Terminal Time (Tend) is Set as Free Variables')
        %Tend_Bound = input('Specify Upper Bound of Terminal Time (i.e. 1,2..(s)):\n');
        %ExpDirectory = ParamFileDir;
    else
        error('Unknown Flag for the Optimization of Terminal Time')
    end
    disp('=====================================================');
end

%--------------------------------------------------------------------------
% Command Line Logging
%--------------------------------------------------------------------------
if gait_discovery_switch == 1 %Gait Discovery using MINLP
    ComputationSetUpLog_filename = strcat('GaitDiscoveryMINLP-Periodical-Loco-log-', datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
elseif gait_discovery_switch == 2 %Fixed Gait Optimization
    ComputationSetUpLog_filename = strcat('FixedGait-Periodical-Loco-log-', datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
end
diary([ExpDirectory, '/', ComputationSetUpLog_filename]);

%--------------------------------------------------------------------------
% Display Introductory Information
%--------------------------------------------------------------------------
disp('====================================================');
disp('Genearl Information:');
disp('====================================================');
if Paras_Define_Method == 1 
    disp('Parameter Loaded From File')
elseif Paras_Define_Method == 2
    disp('Parameter Defined Manually')
end
disp('====================================================');
disp('CasADi Implementation');
if gait_discovery_switch == 1
    disp('2D Quadruped Gait Discovery using Mixed-integer Nonlinear Optimization');
elseif gait_discovery_switch ==2
    disp('2D Quadruped Motion Optimization using Fixed Gait');
end
disp('With a Particular Emphasis on Periodical Motions');
disp('Multi-Phase Formulation:');
disp('Optimize over State, Control, and Switching Time, and (Gait Sequence if we select to optimize)');
disp('----------------------------------------------------');
disp('Date and Time:');
disp(datetime('now'));
disp('----------------------------------------------------');
disp(['Correspondent Log File Name: ', ComputationSetUpLog_filename, 'in the Directory: ', ExpDirectory]);
disp('====================================================');
disp('Using Rotated Force Limits with respect to the slope of the environment')
disp('====================================================');
disp(' ');

%--------------------------------------------------------------------------
% Display or Setup Parameters
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%   Inertial Parameters
%--------------------------------------------------------------------------
disp('====================================================');
disp(['Inertial Parameters'])
disp('====================================================');
if Paras_Define_Method == 1 %Parameters Load from File
    disp(['mass: ',num2str(m),'(kg)']);
    disp(['y-axis Inertial - Iyy: ',num2str(Iyy), '(kg m^2)']);
    disp(['Gravaty: ',num2str(G), 'm/s']);
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
    %m = 45; %kg
    %I = 2.1; %kg m^2 Izz
    m = input('Input Body Mass (i.e. 21) (kg): \n');
    Iyy = input('Input Body Inertia (i.e. 2.3): \n');  
    G = 9.80665; %m/s^2
end

%--------------------------------------------------------------------------
%   Kinematics Parameters
%--------------------------------------------------------------------------
disp('====================================================');
disp(['Kinematics Parameters'])
disp('====================================================');
%       Body Size
if Paras_Define_Method == 1 %Load Parameters from File
    disp('Body Size')
    disp('----------------------------------------------------');
    disp(['Body Length: ',num2str(BodyLength),' (m)']);
    disp(['Body Height: ',num2str(BodyHeight),' (m)']);
    disp('----------------------------------------------------');
    disp(['Default Limb Length:', num2str(DefaultLegLength)]);
    disp('----------------------------------------------------');
    disp('Morphology Set up:')
    if Morpho_change_flag == 1 %Allow Morphology Change
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
    BodyLength = input('Input Body Length (i.e. 0.6): \n');
    BodyHeight = input('Input Body Height (i.e. 0.2): \n');
    %       Default foot position in Local robot frame
    DefaultLegLength = input('Input Default Leg Length (i.e. 0.45): \n');
    
    %       Decide if there are morphology changes
    Morpho_change_flag = input('Decide if we allow morphology change (range from humanoid to half-cheetah): 1-> Yes, Alllow Mophorlogy change 2-> No Morphology change (Default: Half-CHeetah)\n');
    if Morpho_change_flag == 1 %Yes allow mophorlogy change
        Morpho_Percentage = input('Input How much mophorlogy change (displacement from center to edge of the body) is allowed (i.e. 0 - 100%, from humanoid to half_cheetah, Do Not Type Percentage Symbol):\n');
        if Morpho_Percentage == 0
            disp('------------------------------------');
            disp('A Humanoid Model!');
            disp('------------------------------------');
        end
        Morpho_Percentage = Morpho_Percentage/100;
    elseif Morpho_change_flag == 2 %No Morphology Change
        Morpho_Percentage = 100/100;
        disp('No Morphology Change')
    else
        error('Unknown Morphology Change Flag')
    end
    Morpho_Percentage
    
    %   Left Front (lf)
    PlfCenter = [Morpho_Percentage*(1/2*BodyLength);  -(1/2*BodyHeight + DefaultLegLength)];
    %   Left Hind (lh)
    PlhCenter = [-Morpho_Percentage*(1/2*BodyLength); -(1/2*BodyHeight + DefaultLegLength)];
    %   Right Front (rf)
    PrfCenter = [Morpho_Percentage*(1/2*BodyLength);  -(1/2*BodyHeight + DefaultLegLength)];
    %   Right Hind (rh)
    PrhCenter = [-Morpho_Percentage*(1/2*BodyLength); -(1/2*BodyHeight + DefaultLegLength)];
    
    %       Kinematics Bounding Box Constraint
    disp('====================================================');
    disp('Setup Bounding Box for Feet Reachability: ')
    disp('----------------------------------------------------');
    BoundingBox_Width  = input('Define Kinematics Bounding Box Width (i.e. 0.4, 0.6):\n');
    disp('----------------------------------------------------');
    BoundingBox_Height = input('Define Kinematics Bounding Box Hiehgt (i.e. 0.2, 0.25, 0.3):\n');
    disp('----------------------------------------------------');
end

%--------------------------------------------------------------------------
%   Environment Information
%--------------------------------------------------------------------------
if Paras_Define_Method == 1 %Load Parameters from File
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
    %TerrainTangent = [cos(terrain_slope_rad),-sin(terrain_slope_rad);sin(terrain_slope_rad),cos(terrain_slope_rad)]*[1;0];
    disp('Terrain Tangent is: ');
    TerrainTangent
    disp('----------------------------------------------------');
    disp('----------------------------------------------------');
    disp(['Friction Cone: ', num2str(miu)]);
    disp('====================================================');
    disp(' ')
elseif Paras_Define_Method == 2 %Manually Define Parameters
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
        %h_terrain = 0;
        %x_query   = SX.sym('x_query', 1);
        %TerrainModel = Function('TerrainModel', {x_query}, {h_terrain});
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
        %Build Terrain Model for slope terrain
        disp('----------------------------------------------------');
    else %Unknown Scenario
        error('Unknow Terrain Type Flag')
    end

    %Build Terrain Model
    x_query   = SX.sym('x_query', 1);
    h_terrain = terrain_slope*x_query;
    TerrainModel = Function('TerrainModel', {x_query}, {h_terrain});
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
    % For Flat Terrain
    TerrainNorm = [0;1];
    TerrainTangent = [1;0];
    % Rotate Terrain Tangent and Terrain Norm
    TerrainNorm = [0;1];
    TerrainNorm = [cos(terrain_slope_rad), -sin(terrain_slope_rad); sin(terrain_slope_rad), cos(terrain_slope_rad)]*TerrainNorm;
    TerrainTangent = [1;0];
    TerrainTangent = [cos(terrain_slope_rad),-sin(terrain_slope_rad);sin(terrain_slope_rad),cos(terrain_slope_rad)]*TerrainTangent;
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

%--------------------------------------------------------------------------
%   Task Specifications
%--------------------------------------------------------------------------
disp('====================================================');
disp('Task Specification:')
disp('----------------------------------------------------');
if Paras_Define_Method == 1 % Parameter Loaded from File
    %----------------------------------------------------------------------
    %   Specify Desired Speed
    %----------------------------------------------------------------------
    if SpeedDirection == 1
        disp('Desired Velocity Direction along Horizontal Axis');        
    elseif SpeedDirection == 2
        disp('Desired Velocity Direction along Tangential Axis of the Terrain'); 
    end
    disp(['Minimal Speed in Parameter File: ',num2str(MinSpeed)]);
    disp(['Maximum Speed in Parameter File: ',num2str(MaxSpeed)]);
    disp(['Speed Resolution in Parameter File: ',num2str(SpeedResolution)])
    
    ChangSpeedFlag = input('Decide if we want to change the desired speed: 1-> Yes 2->No \n');
    if ChangSpeedFlag ==1
        SpeedDirection = input('Decide what Direction the Desired Velocity should be? 1-> Horizontal 2-> Tangential to the Slope\n');
        MinSpeed = input('Specify the MINIMUM Desired Speed along the Desired Direction (m/s): \n');
        disp('----------------------------------------------------');
        MaxSpeed = input('Specify the MAXIMUM Desired Speed along the Desired Direction (m/s): \n');
        disp('----------------------------------------------------');
        SpeedResolution = input('Specify the Resolution for Scanning the Previously Defined Speed Range (e.g. 0.1, 0.05, 0.02, etc.):\n');
        disp('----------------------------------------------------');
        SpeedList = MinSpeed:SpeedResolution:MaxSpeed;
    end
    %----------------------------------------------------------------------
    %   Terminal time
    %----------------------------------------------------------------------
    if Tend_flag == 1 %Fixed Terminal Time
        disp(['Fixed Terminal Time: ',num2str(Tend)]);
    elseif Tend_flag == 2
        disp('Terminal Time as an Optimization Variable')
        error('Optimization with Variable Terminal Time is Not Implemented')
    end
    disp('----------------------------------------------------');
    disp(' ')
    
elseif Paras_Define_Method == 2 %Manually Define Parameters
    %----------------------------------------------------------------------
    %   Specify Desired Speed
    %----------------------------------------------------------------------
    SpeedDirection = input('Decide what Direction the Desired Velocity should be? 1-> Horizontal 2-> Tangential to the Slope \n');
    MinSpeed = input('Specify the MINIMUM Desired Speed along the Desired Direction (m/s): \n');
    disp('----------------------------------------------------');
    MaxSpeed = input('Specify the MAXIMUM Desired Speed along Desired Direction (m/s): \n');
    disp('----------------------------------------------------');
    SpeedResolution = input('Specify the Resolution for Scanning the Previously Defined Speed Range (e.g. 0.1, 0.05, 0.02, etc.):\n');
    disp('----------------------------------------------------');
    SpeedList = MinSpeed:SpeedResolution:MaxSpeed;
    %----------------------------------------------------------------------
    %   Terminal time
    %----------------------------------------------------------------------
    disp('Terminal Time:')
    Tend_flag = input('Optimization of Terminal Time (Tend): \n1 -> Pre-defined (Fixed), 2 -> Left as Free Variable \n');
    if Tend_flag == 1 %Optimize Terminal Time
        Tend = input('Input Termianl Time (e.g. 1s): \n');
    elseif Tend_flag == 2
        disp('Terminal Time (Tend) is Set as Free Variables')
        Tend_Bound = input('Specify Upper Bound of Terminal Time (i.e. 1,2..(s)):\n');
        error('Optimization with free terminal time is not implemented')
    else
        error('Unknown Terminal Time Setup Flag')
    end
end

%--------------------------------------------------------------------------
% Time Step and Discretization Parameter Settings
%--------------------------------------------------------------------------
if Paras_Define_Method == 1 %Load Parameter From File
    disp('====================================================');
    disp('Temporal and Discretization Setup:');
    disp('----------------------------------------------------');
    %   Number of Phases
    if gait_discovery_switch == 1 %Free Gait Discovery using MINLP
        disp(['Number of Phase: ',num2str(NumPhases)]);
    elseif gait_discovery_switch == 2 %Fixed Gait Optimization
        NumPhases = length(Clf);
        disp(['Number of Phase: ',num2str(NumPhases)])
    end
    disp('----------------------------------------------------');
    %   Number of timesteps for each phase
    disp(['Number of Knots for each phase:' ,num2str(NumLocalKnots)]);
    disp('----------------------------------------------------');
    %   Print some Info
    disp('Resultant Discretization:')
    disp(['Knot/Discretization Step Size of tau: ', num2str(tauStepLength)]);
    disp(['Number of Knots/Discretization of tau (from 0 to ', num2str(tau_upper_limit), ': ', num2str(tauSeriesLength), ') (Number of Knots/Discretization in Each Phase * Number of Phases(NumKnots) + 1)']);
    %   Phase Duration Lower Bound
    disp('----------------------------------------------------');    
    disp(['Phase Duration Lower Bound: ',num2str(phase_lower_bound_portion*100),'%'])
    if phase_lower_bound_portion*NumPhases >= 1
        error('Minimum Phase Duration is invalid -> Total Minimum Phase duration is larger than the Allowed Termianl Time')
    end
    disp('====================================================');
    disp(' ')
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
    disp('====================================================');
    disp('Temporal and Discretization Setup:');
    disp('----------------------------------------------------');
    %   Number of Phases
    if gait_discovery_switch == 1 %Free Gait Discovery using MINLP
        NumPhases = input('Input Number of Phases: \n');
    elseif gait_discovery_switch == 2 %Fixed Gait Optimization
        NumPhases = length(Clf);
        disp(['Number of Phase: ',num2str(NumPhases)])
    end
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
    % Phase Duration Lower Bound
    disp('----------------------------------------------------');
    phase_lower_bound_portion = input('Input Phase Lower Bound (in percentage of total motion duration: 2.5% - no need to type percentage symbol): \n');
    phase_lower_bound_portion = phase_lower_bound_portion/100
    if phase_lower_bound_portion*NumPhases >= 1
        error('Minimum Phase Duration is invalid -> Total Minimum Phase duration is larger than the Allowed Termianl Time')
    end
    disp(['Phase Duration Lower Bound: ',num2str(phase_lower_bound_portion*100),'%'])
    disp('----------------------------------------------------');
    disp(' ')
    disp('====================================================');
    disp(' ')
end

%--------------------------------------------------------------------------
% Big-M Parameters for Complementarity Constraints
%--------------------------------------------------------------------------
%   Display Information
%----------------------------------------------------------------------
disp('====================================================');
disp('Big-M Setup/Limits of Feet Velocity and Forces')
disp('====================================================');
if Paras_Define_Method == 1 %Take fro file
    disp(['Big-M for Foot/End-Effector Velocities: ', num2str(Mvel)]);
    disp('----------------------------------------------------');
    disp(['Velocity Boundary (Abusolute Value) for Foot/End-Effector Velocity in X-axis (In Robot Frame): ',num2str(Vmax(1))]);
    disp(['Velocity Boundary (Abusolute Value) for Foot/End-Effector Velocity in Z-axis (In Robot Frame): ',num2str(Vmax(2))]);
    disp('----------------------------------------------------');
    disp('Big-M for Contact Forces');
    disp('----------------------------------------------------');
    disp(['Big-M for Foot/End-Effector Forces in X-axis: ',num2str(Mf(1))]);
    disp(['Big-M for Foot/End-Effector Forces in Z-axis: ',num2str(Mf(2))]);
    disp('====================================================');
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
    disp('====================================================');
    disp('Big-M Setup/Limits of Feet Velocity and Forces')
    disp('====================================================');
    %----------------------------------------------------------------------
    %   Big-M for foot positions in y-axis (meters)
    M_pos = 100; 
    %   ------------------------------------------------------
    %   Big-M for feet velocities
    Mvel = 100;
    %   ------------------------------------------------------
    %   Velocity Boundary (Abusolute Value) for Foot/End-Effector Velocity in X-axis (In Robot Frame)
    disp('Velocity Boundary (Abusolute Value) for Foot/End-Effector in Robot frame')
    disp('----------------------------------------------------');
    Vmax = [0;0];
    Vmax(1) = input('Decide Velocity Boundary (Abusolute Value) for Foot/End-Effector in Robot frame in X-axis (In Robot Frame): \n');
    Vmax(2) = input('Decide Velocity Boundary (Abusolute Value) for Foot/End-Effector in Robot frame in Z-axis (In Robot Frame): \n');
    disp('----------------------------------------------------');
    disp('Big-M for Contact Forces')
    Mf = [0;0];
    Mf(1) = input('Input Big-M/Boundaries for Foot-Ground Reaction Forces along X-axis (in World Frame) (e.g. 200,300,1000,1e5):\n');
    disp('----------------------------------------------------');
    Mf(2) = input('Input Big-M/Boundaries for Foot-Ground Reaction Forces along Z-axis (in WOrld Frame) (e.g. 200,300,1000,1e5):\n');
    disp('====================================================');
end

%--------------------------------------------------------------------------
%   Cost Setup
%--------------------------------------------------------------------------
if Paras_Define_Method == 1 %Load Parameters from File
    disp('====================================================');
    disp('Cost Setup')
    disp('----------------------------------------------------');
    disp('Selected Cost:');
    if cost_flag == 1
        disp('1-> Minimize Force Squared (Energy Loss)');
    elseif cost_flag == 2
        error('Not Implemented')
        %disp('2-> Minimize Tangential Force (Maximize Robustness)');
    elseif cost_flag == 3
        disp('3-> Minimize Vibration (theta towards terrain slope, thetadot towards zero, normal velocity towards zero)');
    elseif cost_flag == 4
        error('Not Implemented')
        %disp('4-> Maximize Velocity Smoothness (x_tangent towards desired speed, ydot towards zero, thetadot towards zero)');
    elseif cost_flag == 5
        disp('5-> Smooth Motion: \ntangential speed is constant and close to the desired velocity in tangential direction \ntheta close to theta_slope \nthetadot close to 0 normal velocity close to 0)')
    elseif cost_flag == 6
        error('Not Implemented')
        %disp('6-> Feet Velocity');
    elseif cost_flag == 7
        error('Not Implemented')
        %disp('7-> Humanoid Smooth Motion')
    end
    disp('====================================================');
elseif Paras_Define_Method == 2 %Manually Define Variables
    disp('====================================================');
    disp('Set up Cost Terms:')
    disp('----------------------------------------------------');
    cost_flag = input('Decide Cost: \n 1-> Minimize Force Squared (Energy Loss) \n [Not Implemented] 2-> Minimize Tangential Force (Maximize Robustness) \n 3-> Minimize Vibration (theta towards terrain slope, thetadot towards zero, normal velocity towards zero) \n [Not Implemented] 4-> Maximize Velocity Smoothness (x_tangent towards desired speed, ydot towards zero, thetadot towards zero) \n 5-> Smooth Motion (tangential speed is constant and close to the desired velocity in tangential direction \ntheta close to theta_slope \nthetadot close to 0 normal velocity close to 0)\n [Not Implemented] 6-> Feet Velocity \n [Not Implemented] 7-> Humanoid Smooth Motion (theta, thetadot towards zero) \n');
    disp('====================================================');
end

%--------------------------------------------------------------------------
% Solver SetUp
%--------------------------------------------------------------------------
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
    %-----------------------------------------------------------
    Continue_flag = input('Satisfied with the paramter setting? 1-> yes, 2-> no \n');
    if Continue_flag == 2
        error('User is unsatified with the parameter setup')
    end
elseif Paras_Define_Method == 2 %Manually Define, currently set as fixed value
    disp('Solver Selection: ')
    SolverNum = input('1 -> Knitro; 2 -> Bonmin \n');
    if SolverNum == 1
        SolverSelected = 'knitro';
    elseif SolverNum == 2
        SolverSelected = 'bonmin';
    else
        error('Unknown Solver Flag')
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
        NumMaxNodes = (2^4)^(NumPhases+1);
        disp(['Selected Worst-case Scenarios to Explore ', num2str(NumMaxNodes), ' Nodes']);
    elseif NumMaxNodesCases == 2 %User-specified
        NumMaxNodes = input('Input number of maximum node to be explored: \n');
    elseif NumMaxNodesCases == 3 %Default Value
        NumMaxNodes = 1e5;
        disp(['Selected Default Case to Explore ', num2str(NumMaxNodes), ' Nodes'])
    else
        error('Unknow Max Number of Nodes Flag')
    end
    disp('----------------------------------------------------');
    %       Define number of multistart solves for each sub-nonlinear
    %       optimizaiton problem
    NumofRuns = input('Specify Number of Runs for the MINLP Programming (i.e. 1, 10, 25, 50, 100): \n');
    disp('====================================================');
    disp(' ')
    %======================================================================
end

%--------------------------------------------------------------------------
%   Stop the Diary Logging
diary off
%--------------------------------------------------------------------------