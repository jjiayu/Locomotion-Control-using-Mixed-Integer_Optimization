%Compare with MINLP trajecotries

clear;
clc;

% Set up Working Directories, where the gait discovery results store
disp('===================================================================')
disp('Decide Working Directories, where the Initial Guess Story Happens, the detailed folder (i.e. mean, etc...)')
InitialGuess_ExpDirectory = uigetdir('');
disp('Initial Guess Story Happens in:')
disp(InitialGuess_ExpDirectory);
disp(' ')
disp('-------------------------------------------------------------------')
cd(InitialGuess_ExpDirectory)
%cd ..
disp('Specify Gait Discovery Result Directory')
GaitDiscoveryResult_Path = uigetdir('');
disp('Gait Discovery Result Path:')
disp(GaitDiscoveryResult_Path)
disp('===================================================================')
GroupLabel = input('Specify the group label (with '', like Galloping_Group_A): \n');
FilePostFix = input('Specify the Post Fix of the Group Labeling File (with '', like xlsx): \n');
%use mean collection
GroupLabelsFileName = strcat(GroupLabel,'_MeanInitCollection','.',FilePostFix);

%Load Trajectory Group Table
TrajectoryCollection = readtable([GaitDiscoveryResult_Path,'/',GroupLabelsFileName]);

disp('Success Rate:')
success_stats = load([InitialGuess_ExpDirectory,'/Success_Vector.mat'],'success_vector');
success_rate = sum(success_stats.success_vector)/size(TrajectoryCollection,1)

DistanceVector = [];
CostDiffVector = [];
for ExpIdx = 1:size(TrajectoryCollection,1)
    
    % Load MINLP result
    MINLP_OptResult = load(TrajectoryCollection.ExpFilePath{ExpIdx},...
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
                     
    % Load result with Initial Guess
    FileswithInitialGuess = dir(fullfile(['StridePeriod-',num2str(TrajectoryCollection.StridePeriod(ExpIdx)),'-Speed-',num2str(TrajectoryCollection.Speed(ExpIdx)),'-*.mat']));

    FileswithInitialGuess.name
    
    Result_withInitialGuess = load([InitialGuess_ExpDirectory,'/',FileswithInitialGuess(1).name],...
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
    DistanceVector(ExpIdx) = sum((Trajectory_withInitialGuess - Trajectory_MINLP).^2);
    %Compare Cost
    CostDiffVector(ExpIdx) = (Result_withInitialGuess.result_cost - MINLP_OptResult.result_cost)/Result_withInitialGuess.result_cost*100;
    
end
    
distanceCell = {};
for ExpIdx = 1:size(TrajectoryCollection,1)
    distanceCell{ExpIdx,1} = TrajectoryCollection.StridePeriod(ExpIdx);
    distanceCell{ExpIdx,2} = TrajectoryCollection.Speed(ExpIdx);
    distanceCell{ExpIdx,3} = DistanceVector(ExpIdx);
    if DistanceVector(ExpIdx) >= 5
        distanceCell{ExpIdx,4} = 'BigDistance';
    else
        distanceCell{ExpIdx,4} = '-';
    end
    distanceCell{ExpIdx,5} = CostDiffVector(ExpIdx);
    if abs(CostDiffVector(ExpIdx)) >= 0.5
        distanceCell{ExpIdx,6} = 'BigCostDifference';
    else
        distanceCell{ExpIdx,6} = '-';
    end
end

distanceTable = cell2table(distanceCell,'VariableNames',["StridePeriod","Speed","DistancetoMINLP","SignificanceFlag","CostDifference","CostDifference_SignificanceFlag"])

saveCmpResults = input('Decide if we want to save the resultant distance table :\n');

if saveCmpResults == 1
    save([InitialGuess_ExpDirectory,'/','DistanceTabletoMINLP.mat'],'distanceTable','success_rate')
end
%save([InitialGuess_ExpDirectory,'/InitialGuess_StridePeriod_',num2str(StridePeriodVector(InitialGuessFolderIdx)),'_Speed_',num2str(SpeedVector(InitialGuessFolderIdx)),'/DistanceTabletoMINLP.mat'],'distanceTable');
% disp('----------------------------------------------------------------')
