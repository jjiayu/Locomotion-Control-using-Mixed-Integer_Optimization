% Initial Guess Computation
%=========================================================================
% Currently Only Designed for Galloping
%   To add more gaits, need to choose and change the path name of InitialGuessesDatabase
%=========================================================================

clear;
clc;

diary off
addpath(pwd)

%==========================================================================
Paras_Define_Method = 1; %Take from file, load parameters
%==========================================================================

% Set up Working Directories, where the gait discovery results store
disp('===================================================================')
disp('Decide Working Directories, where the Initial Guess Story Happens, (Main Initial Guess Folder, not individual ones)')
InitialGuess_ExpDirectory = uigetdir('');
disp('Initial Guess Story Happens in:')
disp(InitialGuess_ExpDirectory);
disp(' ')
disp('-------------------------------------------------------------------')
cd(InitialGuess_ExpDirectory)
cd ..
GaitDiscoveryResult_Path = pwd;
disp('Gait Discovery Result Path:')
disp(GaitDiscoveryResult_Path)
disp('===================================================================')

% Load condition list in a gait region
GaitGroupLabel = input('Specify the group label (with '', like Galloping_Group_A): \n');
FilePostFix = input('Specify the Post Fix of the Group Labeling File (with '', like xlsx): \n');
TrajectoriesinGaitGroups = readtable([GaitDiscoveryResult_Path,'/',GaitGroupLabel,'.',FilePostFix]);
% Extract condition table
ConditionArray = unique(table2array(TrajectoriesinGaitGroups(:,1:2)),'rows');

%Set up the initial guess
InitialGuessType = input('Specify Initial Guess Type: \n 1->Zeros \n 2-> Random \n 3-> Mean \n 4-> Discovered Trajectories \n 5-> Mean with Invariant Quantities \n 6-> Zero with Invariant Quantities \n');

if InitialGuessType == 4
    ConditionNumberasTemplate = input('Specify which Discovered Trajectory in the Gait Group list is the initial guess (i.e. 5): \n');
else
    ConditionNumberasTemplate = 5;
end

if InitialGuessType == 1
    IntialGuessSubfolderName = 'Zeros';  
elseif InitialGuessType == 2
    IntialGuessSubfolderName = 'Random';
elseif InitialGuessType == 3
    IntialGuessSubfolderName = 'Mean';    
elseif InitialGuessType == 4 %Discovered Trajecotires
    IntialGuessSubfolderNameTemp = strcat('Discovered_StridePeriod_',num2str(TrajectoriesinGaitGroups.StridePeriod(ConditionNumberasTemplate)),'_Speed_',num2str(TrajectoriesinGaitGroups.Speed(ConditionNumberasTemplate)),'/',TrajectoriesinGaitGroups.ExpFileName(ConditionNumberasTemplate));
    IntialGuessSubfolderName = IntialGuessSubfolderNameTemp{:}; %remove .mat
    IntialGuessSubfolderName = IntialGuessSubfolderName(1:end-4);
elseif InitialGuessType == 5
    IntialGuessSubfolderName = 'MeanwithInvariant';
elseif InitialGuessType == 6
    IntialGuessSubfolderName = 'ZerowithInvariant';
end

if exist(strcat(InitialGuess_ExpDirectory,'/',GaitGroupLabel,'_',IntialGuessSubfolderName),'dir') == 1
    ExpDirectory = strcat(InitialGuess_ExpDirectory,'/',GaitGroupLabel,'_',IntialGuessSubfolderName);
else
    mkdir(strcat(InitialGuess_ExpDirectory,'/',GaitGroupLabel,'_',IntialGuessSubfolderName));
    ExpDirectory = strcat(InitialGuess_ExpDirectory,'/',GaitGroupLabel,'_',IntialGuessSubfolderName);
end

InitialGuessLog_filename = strcat('InitialGuess-Log-',datestr(datetime('now'), 30));
diary([ExpDirectory,'/',InitialGuessLog_filename]);

% Start Building Initial Guess -> Make a container
MINLP_Discovered_Trajectory = load([TrajectoriesinGaitGroups.ExpFilePath{ConditionNumberasTemplate}],...
                                  'x_result',   'y_result',   'theta_result',...
                                  'xdot_result','ydot_result','thetadot_result',...
                                  'PFx_result', 'PFy_result', 'PFxdot_result',   'PFydot_result',...
                                  'PHx_result', 'PHy_result', 'PHxdot_result',   'PHydot_result',...
                                  'FFx_result', 'FFy_result',...
                                  'FHx_result', 'FHy_result',...
                                  'PhaseSwitchingTime',... %Note, Phase Switching Time Start from a constant zero
                                  'CF_result',   'CH_result');
%Phase Switching Time Result need specical care:
MINLP_Discovered_Trajectory.PhaseSwitchingTime = MINLP_Discovered_Trajectory.PhaseSwitchingTime(2:end); %remove the constant zero ad the beginning of the array
TempInitialGuessTrajectory = [MINLP_Discovered_Trajectory.x_result;      MINLP_Discovered_Trajectory.y_result;        MINLP_Discovered_Trajectory.theta_result;...
                              MINLP_Discovered_Trajectory.xdot_result;   MINLP_Discovered_Trajectory.ydot_result;     MINLP_Discovered_Trajectory.thetadot_result;...
                              MINLP_Discovered_Trajectory.PFx_result;    MINLP_Discovered_Trajectory.PFy_result;      MINLP_Discovered_Trajectory.PFxdot_result;       MINLP_Discovered_Trajectory.PFydot_result;...
                              MINLP_Discovered_Trajectory.PHx_result;    MINLP_Discovered_Trajectory.PHy_result;      MINLP_Discovered_Trajectory.PHxdot_result;       MINLP_Discovered_Trajectory.PHydot_result;...
                              MINLP_Discovered_Trajectory.FFx_result;    MINLP_Discovered_Trajectory.FFy_result;...
                              MINLP_Discovered_Trajectory.FHx_result;    MINLP_Discovered_Trajectory.FHy_result;...
                              MINLP_Discovered_Trajectory.PhaseSwitchingTime]';

%--------------------------------------------------------------------------
%Construct True Initial Guess
%--------------------------------------------------------------------------
if InitialGuessType == 1
    InitialGuessVector = zeros(size(TempInitialGuessTrajectory));
elseif InitialGuessType == 2
    disp('Not Implemented')
elseif InitialGuessType == 3
    FullDiscoveredTrajectoryCollection = load([GaitGroupLabel,'_MeanInitCollection_MINLP_Trajectories','.mat'],'FullTrajectoriesCollection_NoTimeSeries');
    MeanInit = mean(FullDiscoveredTrajectoryCollection.FullTrajectoriesCollection_NoTimeSeries);
    InitialGuessVector = MeanInit;
elseif InitialGuessType == 4
    InitialGuessVector = TempInitialGuessTrajectory;
    %IntialGuessSubfolderName = ''
elseif InitialGuessType == 5 %Mean with Invariant Quantities, Abandon
    TempInit = load([GaitDiscoveryResult_Path,'/Galloping_Group_A_InitialGuess_Mean_with_Invariant.mat'],'MeanInitwithInvariant');
    InitialGuessVector = TempInit.MeanInitwithInvariant
elseif InitialGuessType == 6 %Zero with Invariant Quantities, Abandon
    TempInit = load([GaitDiscoveryResult_Path,'/Galloping_Group_A_ZeroInit_with_Invariant.mat'],'zeroInitwithInvariant');
    InitialGuessVector = TempInit.zeroInitwithInvariant;
end


% Load Parameters
run([InitialGuess_ExpDirectory,'/','Parameters_InitialGuess.m']);

% Save Parameter Setup
save([InitialGuess_ExpDirectory,'/','ParameterSetUp.mat']);

CF = MINLP_Discovered_Trajectory.CF_result;
CH = MINLP_Discovered_Trajectory.CH_result;
%======================================================================
%   Run the big computation loop
%======================================================================
run('RotatedForceLimit_Computation_Loop_InitialGuess.m');
%======================================================================
save([ExpDirectory,'/Success_Vector.mat'],'success_vector','ConditionArray')

disp('===================================================');
disp('All Experiments Finished')
disp('===================================================');