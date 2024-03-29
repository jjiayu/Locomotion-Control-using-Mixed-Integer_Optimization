%Script for computing gait discovery result for test set

clear;
clc;

diary off

import casadi.*

% Specify the test set task sample file path
disp('Decide the working directory (the root diretory storing training data, test data, i.e. 0_Degrees,...)')
terminator_laptop_flag = input('Decide the computer we are using: 1-> Laptop, 2->Terminator \n');

if terminator_laptop_flag == 1 %laptop
    working_directory = uigetdir;
elseif terminator_laptop_flag == 2 %terminator
    working_directory = input('Manually Define Folder Path Storing Parameter File (quote with ''): \n');
end

TestSetDirectory = [working_directory,'/TestSet'];

disp(['Working Directory: ', working_directory])
disp('===================================================================')

TestSetComputationLog = strcat('TestSetComputation-log-', datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
diary([TestSetDirectory, '/', TestSetComputationLog]);

%load test set samples
TestSetFileName = input('Input Test Set Samples Files name (the csv file name, using '' to quote, without csv): \n');
testSamples = csvread([TestSetDirectory,'/',TestSetFileName,'.csv'],1,0); %numbers are offsets of row and column numbers

disp(['Total Number of the Samples: ', num2str(size(testSamples,1))])
disp(' ')

%Specify range of test set samples
StartSampleIdx = input('Specify the Starting Index of the Task Samples in the Test Set (i.e. 1): \n');
TerminalSampleIdx = input('Specify the Terminal Index of the Task Samples in the Test Set (i.e. 10): \n');
%Collect samples of interest
SelectedtestSamples = testSamples(StartSampleIdx:TerminalSampleIdx,:);

%Decide the Flag for if we optimize with fixed time interval or not
Tend_flag = 1;

%Load Parameter
run([working_directory,'/Parameters.m']);

% Print the parameter setup
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
disp(['Correspondent Log File Name: ', TestSetComputationLog, 'in the Directory: ', TestSetDirectory]);
disp('====================================================');
disp(' ');
%=====================================================================


%======================================================================
%Inertia Parameters(Information from MIT Cheetah 3)
disp('====================================================');
disp(['Inertial Parameters'])
disp('====================================================');
disp(['mass: ',num2str(m),'(kg)']);
disp(['Z-axis Inertial - Izz: ',num2str(I), '(kg m^2)']);
disp(['Gravaty: ',num2str(G), 'm/s']);
%======================================================================

%======================================================================
%   Kinematics Constraint Parameters
%======================================================================
disp('====================================================');
disp(['Kinematics Constraint Parameters'])
disp('====================================================');
%       Body Size
disp('Body Size')
disp('----------------------------------------------------');
disp(['Body Length: ',num2str(BodyLength),' (m)']);
disp(['Body Height: ',num2str(BodyHeight),' (m)']);
disp('----------------------------------------------------');
disp('Robot Kinematics Bounding Box')
disp('----------------------------------------------------');
disp(['Bounding Box Width: ',num2str(BoundingBox_Width),' (m)']);
disp(['Bounding Box Height: ',num2str(BoundingBox_Height),' (m)']);
disp('----------------------------------------------------');
%======================================================================

%======================================================================
%Cost Setup
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


%======================================================================
%Environment Information
%----------------------------------------------------------------------
%   Display some info
%----------------------------------------------------------------------
disp('====================================================');
disp('Terrain Model: ')
disp('----------------------------------------------------');
if TerrainType == 1
    disp('Flat Terrain');
elseif TerrainType == 2
    disp('Slopes');
end
%-----------------------------------------------------------------------
%     %Other Parameters
%     if TerrainType == 1 %Flat Terrain
%         TerrainNorm = [0;1];
%     elseif TerrainType == 2 % if Stairs, over-write the terrain norm with 
%         TerrainNorm = [0;1];
%         TerrainNorm = [cos(terrain_slope_rad), -sin(terrain_slope_rad); sin(terrain_slope_rad), cos(terrain_slope_rad)]*TerrainNorm;
%     end
disp('Terrain Norm is: ');
TerrainNorm
TerrainTangent = [cos(terrain_slope_rad),-sin(terrain_slope_rad);sin(terrain_slope_rad),cos(terrain_slope_rad)]*[1;0];
disp('Terrain Tangent is: ');
TerrainTangent
disp('----------------------------------------------------');
%     miu = 0.6; %friction coefficient
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
disp(['Minimal Speed: ',num2str(MinSpeed)]);
disp(['Maximum Speed: ',num2str(MaxSpeed)]);
disp(['Speed Resolution: ',num2str(SpeedResolution)])
%----------------------------------------------------------------------
%   Terminal time
%----------------------------------------------------------------------
% if Tend_flag == 1 %Optimize Terminal Time
%     disp(['Fixed Terminal Time: ',num2str(Tend)]);
% elseif Tend_flag == 2
%     disp(['Terminal Time as an Optimization Variable'])
% end
disp('----------------------------------------------------');
disp(['Phase Duration Lower Bound: ',num2str(phase_lower_bound_portion*100),'%'])
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
%   big-M for foot velocity
%-------------------------------
%   (Place Holder) big-M for all x, y, z axis velocities need to respecify
%   when move to 3D
%------------------------------------------------------------------------
%       big-M for X-axis Foot/End-Effector Velocity
%-------------------------------------------------------------------------
disp('Big-M for Foot/End-Effector Velocities')
disp('----------------------------------------------------');
disp(['Big-M for Foot/End-Effector Velocity in X-axis: ',num2str(Mvelx)]);
disp(['Big-M for Foot/End-Effector Velocity in Y-axis: ',num2str(Mvely)]);
disp('----------------------------------------------------');
disp('Big-M for Contact Forces');
disp('----------------------------------------------------');
disp(['Big-M for Foot/End-Effector Forces in X-axis: ',num2str(Mfx)]);
disp(['Big-M for Foot/End-Effector Forces in X-axis: ',num2str(Mfy)]);
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
%======================================================================

Continue_flag = input('Satisfied with the paramter setting? 1-> yes, 2-> no \n');
if Continue_flag == 2
    ME_UserStop = MException('Initialization:UserStop',['User is unsatisfied with the parameter setup']);
    throw(ME_UserStop)
end

% Perform computation

for SampleIdx = 1:size(SelectedtestSamples,1)
    Tend = SelectedtestSamples(SampleIdx,1); %Define Stride Period
    SpeedList = SelectedtestSamples(SampleIdx,2); %Define Speed List
    
    disp('====================================================');
    disp(['Stride Period of the Test Sample: ',num2str(Tend)])
    disp(['Speed of the Test Sample: ', num2str(SpeedList)]);
    disp('====================================================');
    
    %Setup ExpDirectory
    ExpDirectory = [working_directory,'/TestSet/StridePeriod_',num2str(Tend),'_Speed_',num2str(SpeedList)];
    if exist(ExpDirectory,'dir') ~= 1
        mkdir(ExpDirectory);
    end
    
    %run the computation
    run('~/Desktop/Locomotion-Control-using-Mixed-Integer_Optimization/Half_Cheetah_casadi_implementation/MINLP_Computation_Loop.m');
    
end





