%Compare with MINLP trajecotries

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

for InitialGuessFolderIdx = 1:length(StridePeriodVector)
    %get into IntialGuess Folder
    disp(['Initial Guess Using: Stride Period: ',num2str(StridePeriodVector(InitialGuessFolderIdx)),' Speed: ',num2str(SpeedVector(InitialGuessFolderIdx))]);
    %display success rate
    disp('Success Rate:')
    success_rate = load([InitialGuess_ExpDirectory,'/InitialGuess_StridePeriod_',num2str(StridePeriodVector(InitialGuessFolderIdx)),'_Speed_',num2str(SpeedVector(InitialGuessFolderIdx)),'/Success_Vector.mat'],'success_vector');
    sum(success_rate.success_vector)/length(StridePeriodVector)
   
    %Compare to MINLP Trajectories
    DIstanceVector = [];
    for InitialGuessExpIdx = 1:length(StridePeriodVector)
        TaskSpec = cell2table({StridePeriodVector(InitialGuessExpIdx),SpeedVector(InitialGuessExpIdx)},'VariableNames',["StridePeriod","Speed"]);
        [~,Idx_InitialGuessDatabase,~]=intersect(InitialGuessesDatabase(:,1:2),TaskSpec);
        InitialGuessFileName = table2array(InitialGuessesDatabase(Idx_InitialGuessDatabase,StartingPhase+4));
        %Extract MINLP trajectory
        MINLP_OptResult = load([GaitDiscoveryResult_Path,'/4Phases_StridePeriod_',num2str(StridePeriodVector(InitialGuessExpIdx)),'/',InitialGuessFileName{:}],...
                                    'x_result',   'y_result',   'theta_result',...
                                    'xdot_result','ydot_result','thetadot_result',...
                                    'PFx_result', 'PFy_result', 'PFxdot_result',   'PFydot_result',...
                                    'PHx_result', 'PHy_result', 'PHxdot_result',   'PHydot_result',...
                                    'FFx_result', 'FFy_result',...
                                    'FHx_result', 'FHy_result',...
                                    'PhaseSwitchingTime',... %Note, Phase Switching Time Start from a constant zero
                                    'CF_result',   'CH_result',...
                                    'result_cost');
        MINLP_OptResult.PhaseSwitchingTime = MINLP_OptResult.PhaseSwitchingTime(2:end);
        
        Trajectory_MINLP  = [MINLP_OptResult.x_result;      MINLP_OptResult.y_result;        MINLP_OptResult.theta_result;...
                             MINLP_OptResult.xdot_result;   MINLP_OptResult.ydot_result;     MINLP_OptResult.thetadot_result;...
                             MINLP_OptResult.PFx_result;    MINLP_OptResult.PFy_result;      MINLP_OptResult.PFxdot_result;       MINLP_OptResult.PFydot_result;...
                             MINLP_OptResult.PHx_result;    MINLP_OptResult.PHy_result;      MINLP_OptResult.PHxdot_result;       MINLP_OptResult.PHydot_result;...
                             MINLP_OptResult.FFx_result;    MINLP_OptResult.FFy_result;...
                             MINLP_OptResult.FHx_result;    MINLP_OptResult.FHy_result;...
                             MINLP_OptResult.PhaseSwitchingTime]';
        
        %Extract Trajectory with Initial Guess
        FileswithInitialGuess = dir(fullfile([InitialGuess_ExpDirectory,'/InitialGuess_StridePeriod_',num2str(StridePeriodVector(InitialGuessFolderIdx)),'_Speed_',num2str(SpeedVector(InitialGuessFolderIdx)),'/'],['StridePeriod-',num2str(StridePeriodVector(InitialGuessExpIdx)),'-Speed-',num2str(SpeedVector(InitialGuessExpIdx)),'-*.mat']));
                         
        Result_withInitialGuess = load([InitialGuess_ExpDirectory,'/InitialGuess_StridePeriod_',num2str(StridePeriodVector(InitialGuessFolderIdx)),'_Speed_',num2str(SpeedVector(InitialGuessFolderIdx)),'/',FileswithInitialGuess(1).name],...
                                    'x_result',   'y_result',   'theta_result',...
                                    'xdot_result','ydot_result','thetadot_result',...
                                    'PFx_result', 'PFy_result', 'PFxdot_result',   'PFydot_result',...
                                    'PHx_result', 'PHy_result', 'PHxdot_result',   'PHydot_result',...
                                    'FFx_result', 'FFy_result',...
                                    'FHx_result', 'FHy_result',...
                                    'PhaseSwitchingTime',...
                                    'result_cost');
        Result_withInitialGuess.PhaseSwitchingTime = Result_withInitialGuess.PhaseSwitchingTime(2:end);
        Trajectory_withInitialGuess = [Result_withInitialGuess.x_result;      Result_withInitialGuess.y_result;        Result_withInitialGuess.theta_result;...
                                       Result_withInitialGuess.xdot_result;   Result_withInitialGuess.ydot_result;     Result_withInitialGuess.thetadot_result;...
                                       Result_withInitialGuess.PFx_result;    Result_withInitialGuess.PFy_result;      Result_withInitialGuess.PFxdot_result;       Result_withInitialGuess.PFydot_result;...
                                       Result_withInitialGuess.PHx_result;    Result_withInitialGuess.PHy_result;      Result_withInitialGuess.PHxdot_result;       Result_withInitialGuess.PHydot_result;...
                                       Result_withInitialGuess.FFx_result;    Result_withInitialGuess.FFy_result;...
                                       Result_withInitialGuess.FHx_result;    Result_withInitialGuess.FHy_result;...
                                       Result_withInitialGuess.PhaseSwitchingTime]';
        %Compare Trajecoties
        DistanceVector(InitialGuessExpIdx) = sum((Trajectory_withInitialGuess - Trajectory_MINLP).^2);
        %Compare Cost
        CostDiffVector(InitialGuessExpIdx) = (Result_withInitialGuess.result_cost - MINLP_OptResult.result_cost)/Result_withInitialGuess.result_cost*100;
    end
    
    distanceCell = {};
    for loopIdx = 1:length(StridePeriodVector)
        distanceCell{loopIdx,1} = StridePeriodVector(loopIdx);
        distanceCell{loopIdx,2} = SpeedVector(loopIdx);
        distanceCell{loopIdx,3} = DistanceVector(loopIdx);
        if DistanceVector(loopIdx) >= 5
            distanceCell{loopIdx,4} = 'BigDistance';
        else
            distanceCell{loopIdx,4} = '-';
        end
        distanceCell{loopIdx,5} = CostDiffVector(loopIdx);
        if abs(CostDiffVector(loopIdx)) >= 0.1
            distanceCell{loopIdx,6} = 'BigCostDifference';
        else
            distanceCell{loopIdx,6} = '-';
        end
    end
    
    distanceTable = cell2table(distanceCell,'VariableNames',["StridePeriod","Speed","DistancetoMINLP","SignificanceFlag","CostDifference","CostDifference_SignificanceFlag"])
    save([InitialGuess_ExpDirectory,'/InitialGuess_StridePeriod_',num2str(StridePeriodVector(InitialGuessFolderIdx)),'_Speed_',num2str(SpeedVector(InitialGuessFolderIdx)),'/DistanceTabletoMINLP.mat'],'distanceTable');
    disp('----------------------------------------------------------------')
    
end