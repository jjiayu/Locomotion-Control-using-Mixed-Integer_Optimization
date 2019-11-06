% Initial Guess Computation
%=========================================================================
% Currently Only Designed for Galloping
%   To add more gaits, need to choose and change the path name of InitialGuessesDatabase
%=========================================================================

clear;
clc;

% Set up Working Directories, where the gait discovery results store
disp('===================================================================')
disp('Decide Working Directories, where the Initial Guess Story Happens')
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
% Load Initial Guess mapping
%   Define the Initial Guess point
InitialGuess_StridePeriod = input('Define the Stride Period for Extracting the Initial Guess (i.e. 0.8): \n');
disp('-------------------------------------------------------------------')
InitialGuess_Speed = input('Define the Speed for Extracting the Initial Guess (i.e. 1.5): \n');
disp('-------------------------------------------------------------------')
StartingPhase = input('Define the Starting Phase of the Initial Guess (2 and 3 are preferred, Currently Only Designed for Galloping Gait): \n 1-> Hind Contact \n 2-> Front Contact \n 3-> Double Support \n 4-> Flying \n');
disp('-------------------------------------------------------------------')
StartingPhase = StartingPhase + 4; %Shift the column
%   Get the name of the Initial Guess File
SelectedInitialGuess = cell2table({InitialGuess_StridePeriod,InitialGuess_Speed},'VariableNames',["StridePeriod","Speed"]);
InitialGuessesDatabase = readtable([InitialGuess_ExpDirectory,'/Initial_Guess_Galloping.csv']);
[~,Idx_InitialGuessDatabase,~]=intersect(InitialGuessesDatabase(:,1:2),SelectedInitialGuess);
InitialGuessFileName = table2array(InitialGuessesDatabase(Idx_InitialGuessDatabase,StartingPhase));

%   Load and the Initial Guess
%       need to have InitialGuessFileName{:}, because InitialGuessFileName
%       is cell
InitialGuessTrajectory = load([GaitDiscoveryResult_Path,'/4Phases_StridePeriod_',num2str(InitialGuess_StridePeriod),'/',InitialGuessFileName{:}],...
                         'x_result',   'y_result',   'theta_result',...
                         'xdot_result','ydot_result','thetadot_result',...
                         'PFx_result', 'PFy_result', 'PFxdot_result',   'PFydot_result',...
                         'PHx_result', 'PHy_result', 'PHxdot_result',   'PHydot_result',...
                         'FFx_result', 'FFy_result',...
                         'FHx_result', 'FHy_result',...
                         'PhaseSwitchingTime',... %Note, Phase Switching Time Start from a constant zero
                         'CF_result',   'CH_result');
%Phase Switching Time Result need specical care:
InitialGuessTrajectory.PhaseSwitchingTime = InitialGuessTrajectory.PhaseSwitchingTime(2:end); %remove the constant zero ad the beginning of the array
InitialGuessVector = [InitialGuessTrajectory.x_result;      InitialGuessTrajectory.y_result;        InitialGuessTrajectory.theta_result;...
                      InitialGuessTrajectory.xdot_result;   InitialGuessTrajectory.ydot_result;     InitialGuessTrajectory.thetadot_result;...
                      InitialGuessTrajectory.PFx_result;    InitialGuessTrajectory.PFy_result;      InitialGuessTrajectory.PFxdot_result;       InitialGuessTrajectory.PFydot_result;...
                      InitialGuessTrajectory.PHx_result;    InitialGuessTrajectory.PHy_result;      InitialGuessTrajectory.PHxdot_result;       InitialGuessTrajectory.PHydot_result;...
                      InitialGuessTrajectory.FFx_result;    InitialGuessTrajectory.FFy_result;...
                      InitialGuessTrajectory.FHx_result;    InitialGuessTrajectory.FHy_result;...
                      InitialGuessTrajectory.PhaseSwitchingTime];
                  
% Build task Vectors
Stride_Period_Vector = table2array(InitialGuessesDatabase(:,1));
Speed_Vector = table2array(InitialGuessesDatabase(:,2));

% Load Parameters
run([InitialGuess_ExpDirectory,'/','Parameters_InitialGuess.m']);

