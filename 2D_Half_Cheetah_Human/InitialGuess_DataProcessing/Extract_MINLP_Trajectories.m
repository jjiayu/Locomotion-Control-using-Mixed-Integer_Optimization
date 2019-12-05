% Currently only for galloping gait
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
StartingPhase = input('Define the Starting Phase of the Initial Guess (2 and 3 are preferred, Currently Only Designed for Galloping Gait): \n 1-> Hind Contact \n 2-> Front Contact \n 3-> Double Support \n 4-> Flying \n');
disp('-------------------------------------------------------------------')

%Load Initial Guess Table
%NOTE:Current Script is for Galloping Gait Only
InitialGuessesDatabase = readtable([InitialGuess_ExpDirectory,'/Initial_Guess_Galloping.csv']);
StridePeriodVector = table2array(InitialGuessesDatabase(:,1));
SpeedVector = table2array(InitialGuessesDatabase(:,2));

Trajectories = [];

MINLP_Result_DataBase = {};
for loopIdx = 1:length(StridePeriodVector)
    TaskSpec = cell2table({StridePeriodVector(loopIdx),SpeedVector(loopIdx)},'VariableNames',["StridePeriod","Speed"]);
    [~,Idx_InitialGuessDatabase,~]=intersect(InitialGuessesDatabase(:,1:2),TaskSpec);
    InitialGuessFileName = table2array(InitialGuessesDatabase(Idx_InitialGuessDatabase,StartingPhase+4));
    %Extract MINLP trajectory
    MINLP_OptResult = load([GaitDiscoveryResult_Path,'/4Phases_StridePeriod_',num2str(StridePeriodVector(loopIdx)),'/',InitialGuessFileName{:}],...
                                    'x_result',   'y_result',   'theta_result',...
                                    'xdot_result','ydot_result','thetadot_result',...
                                    'PFx_result', 'PFy_result', 'PFxdot_result',   'PFydot_result',...
                                    'PHx_result', 'PHy_result', 'PHxdot_result',   'PHydot_result',...
                                    'FFx_result', 'FFy_result',...
                                    'FHx_result', 'FHy_result',...
                                    'PhaseSwitchingTime',... %Note, Phase Switching Time Start from a constant zero
                                    'CF_result',   'CH_result',...
                                    'PFcenterX_result_world','PFcenterY_result_world',...
                                    'PHcenterX_result_world','PHcenterY_result_world',...
                                    'result_cost');
    MINLP_OptResult.PhaseSwitchingTime = MINLP_OptResult.PhaseSwitchingTime(2:end);
    %   Save Result labels
    MINLP_OptResult.StridePeriod = StridePeriodVector(loopIdx);
    MINLP_OptResult.Speed = SpeedVector(loopIdx);
    MINLP_OptResult.MINLP_Result_FileName = InitialGuessFileName;
    %Transform the world frame quantities into robot frame
    %   Foot Step Locations
    for knotIdx = 1:length(MINLP_OptResult.theta_result)
        %Front Leg Position
        FrontLeg_Position_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),sin(MINLP_OptResult.theta_result(knotIdx));-sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PFx_result(knotIdx);MINLP_OptResult.PFy_result(knotIdx)] - [MINLP_OptResult.x_result(knotIdx);MINLP_OptResult.y_result(knotIdx)]);
        MINLP_OptResult.PFx_result_Robot(knotIdx) = FrontLeg_Position_Temp(1);
        MINLP_OptResult.PFy_result_Robot(knotIdx) = FrontLeg_Position_Temp(2);
        
        %Hind Leg Position
        HindLeg_Position_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),sin(MINLP_OptResult.theta_result(knotIdx));-sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PHx_result(knotIdx);MINLP_OptResult.PHy_result(knotIdx)] - [MINLP_OptResult.x_result(knotIdx);MINLP_OptResult.y_result(knotIdx)]);
        MINLP_OptResult.PHx_result_Robot(knotIdx) = HindLeg_Position_Temp(1);
        MINLP_OptResult.PHy_result_Robot(knotIdx) = HindLeg_Position_Temp(2);
        
    end
    %   Feet Velocities
    for knotIdx = 1:length(MINLP_OptResult.theta_result)-1      
        %Front Leg Velocity
        FrontLeg_Velo_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),-sin(MINLP_OptResult.theta_result(knotIdx));sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PFxdot_result(knotIdx);MINLP_OptResult.PFydot_result(knotIdx)] - [MINLP_OptResult.xdot_result(knotIdx);MINLP_OptResult.ydot_result(knotIdx)]);
        MINLP_OptResult.PFxdot_result_Robot(knotIdx) = FrontLeg_Velo_Temp(1);
        MINLP_OptResult.PFydot_result_Robot(knotIdx) = FrontLeg_Velo_Temp(2);
        
        %Hind Leg Velocity
        HindLeg_Velo_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),-sin(MINLP_OptResult.theta_result(knotIdx));sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PHxdot_result(knotIdx);MINLP_OptResult.PHydot_result(knotIdx)] - [MINLP_OptResult.xdot_result(knotIdx);MINLP_OptResult.ydot_result(knotIdx)]);
        MINLP_OptResult.PHxdot_result_Robot(knotIdx) = HindLeg_Velo_Temp(1);
        MINLP_OptResult.PHydot_result_Robot(knotIdx) = HindLeg_Velo_Temp(2);       
    end
    
    %Build the trajectories
    Trajectory_MINLP  = [MINLP_OptResult.x_result;      MINLP_OptResult.y_result;        MINLP_OptResult.theta_result;...
                         MINLP_OptResult.xdot_result;   MINLP_OptResult.ydot_result;     MINLP_OptResult.thetadot_result;...
                         MINLP_OptResult.PFx_result;    MINLP_OptResult.PFy_result;      MINLP_OptResult.PFxdot_result;       MINLP_OptResult.PFydot_result;...
                         MINLP_OptResult.PHx_result;    MINLP_OptResult.PHy_result;      MINLP_OptResult.PHxdot_result;       MINLP_OptResult.PHydot_result;...
                         MINLP_OptResult.FFx_result;    MINLP_OptResult.FFy_result;...
                         MINLP_OptResult.FHx_result;    MINLP_OptResult.FHy_result;...
                         MINLP_OptResult.PhaseSwitchingTime]';
   MINLP_Result_DataBase{loopIdx} = MINLP_OptResult;
   Trajectories(loopIdx,:) = Trajectory_MINLP';
end

save([GaitDiscoveryResult_Path,'/MINLP_Trajectories_Galloping_GaitStartfrom_Phase',num2str(StartingPhase),'.mat'], 'MINLP_Result_DataBase','StridePeriodVector','SpeedVector');
