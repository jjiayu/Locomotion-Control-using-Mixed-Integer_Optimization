% Group Discovered Trajectories into Different Gait Groups

% Given Running the Data Processing Scritps, Loaded with Gait Discovery
% Results.

%Collect stride frequency, step length, etc.
clc;
clear;

%-------------------------------------------------------------------------
% Load Result File
%-------------------------------------------------------------------------
disp('Load Gait Discovery Result Data Files')
[data_file_name,data_file_path] = uigetfile();
full_data_file_path = [data_file_path,data_file_name];
disp(['Selected File: ',full_data_file_path]);
load(full_data_file_path)

%-------------------------------------------------------------------------
%   Build Result Container
%-------------------------------------------------------------------------
TraGroupsMatrix = cell(size(resultMatrix));

%-------------------------------------------------------------------------
% Loop inside Data (resultMatrix)
%-------------------------------------------------------------------------
disp('-------------------------------------------------------------------')
disp('Grouping Trajectories')
disp('-------------------------------------------------------------------')
DataAnalysisResult = {};
for resultIdx = 1:size(resultMatrix,1)*size(resultMatrix,2)
    
    disp(['Stride Period: ', num2str(resultMatrix{resultIdx}.strideperiod_speed_pair(1)),' Speed: ', num2str(resultMatrix{resultIdx}.strideperiod_speed_pair(2))])
    
    %   Structure to store grouped trajectories for different stride period
    %   and speed pairs
    %   Initialize a structure for data storage
    TraGroupTemp = struct();
    TraGroupTemp.StridePeriod = resultMatrix{resultIdx}.strideperiod_speed_pair(1);
    TraGroupTemp.Speed = resultMatrix{resultIdx}.strideperiod_speed_pair(2);
    TraGroupTemp.DiscoveredTrajectoryFileNameList = {};
    TraGroupTemp.GaitGroupNameList = {};
    TraGroupTemp.ExpFilePathList = {};
    
    if strcmp(resultMatrix{resultIdx}.OptimalGaitName,'N/A') ~= 1
       [GaitNameTemp,GaitGroupNameTemp] = TrajectoryGroupClassifier(resultMatrix{resultIdx}.OptimalGaitName,resultMatrix{resultIdx}.OptimalGait,0);
       
        resultMatrix{resultIdx}.OptimalGait
        GaitGroupNameTemp
       
        TraGroupTemp.DiscoveredTrajectoryFileNameList{end+1} = resultMatrix{resultIdx}.OptimalFile;
        TraGroupTemp.GaitGroupNameList{end+1} = GaitGroupNameTemp;
        
        ExpFilePath = strcat(data_file_path,'4Phases_StridePeriod_',num2str(resultMatrix{resultIdx}.strideperiod_speed_pair(1)),'/',resultMatrix{resultIdx}.OptimalFile); 
        TraGroupTemp.ExpFilePathList{end+1} = ExpFilePath;
        
        % Loop Over Files with Similar Cost
        for similiarResultIdx = 1:length(resultMatrix{resultIdx}.GaitNameswithSimilarCost)    
            
            ExpFilePath = strcat(data_file_path,'4Phases_StridePeriod_',num2str(resultMatrix{resultIdx}.strideperiod_speed_pair(1)),'/',resultMatrix{resultIdx}.SimilarLocalMinimaFileNames{similiarResultIdx}); 
            %Load gait
            gaitTemp = load(ExpFilePath,'gait');
            
            [GaitNameTemp,GaitGroupNameTemp] = TrajectoryGroupClassifier(resultMatrix{resultIdx}.GaitNameswithSimilarCost{similiarResultIdx},gaitTemp.gait,0);
            
            gaitTemp.gait
            GaitGroupNameTemp
            
            TraGroupTemp.DiscoveredTrajectoryFileNameList{end+1} = resultMatrix{resultIdx}.SimilarLocalMinimaFileNames{similiarResultIdx};
            TraGroupTemp.GaitGroupNameList{end+1} = GaitGroupNameTemp;
            TraGroupTemp.ExpFilePathList{end+1} = ExpFilePath;
        end
        
    end
    
    TraGroupsMatrix{resultIdx} = TraGroupTemp;
    
end

%   Save Data
save([data_file_path,'/TrajectorywithGroupLabels',datestr(datetime('now'), 30),'.mat']);

%-------------------------------------------------------------------
%   Make Tables
%-------------------------------------------------------------------
GroupedTrajectoriesCollection = {};
for TaskIdx = 1:size(TraGroupsMatrix,1)*size(TraGroupsMatrix,2)
    disp(['Stride Period: ', num2str(TraGroupsMatrix{TaskIdx}.StridePeriod),' Speed: ', num2str(TraGroupsMatrix{TaskIdx}.Speed)]);
    
    for traIdx = 1:length(TraGroupsMatrix{TaskIdx}.GaitGroupNameList)
        TempLine = {TraGroupsMatrix{TaskIdx}.StridePeriod,TraGroupsMatrix{TaskIdx}.Speed,...
                    TraGroupsMatrix{TaskIdx}.GaitGroupNameList{traIdx},...
                    TraGroupsMatrix{TaskIdx}.DiscoveredTrajectoryFileNameList{traIdx},...
                    TraGroupsMatrix{TaskIdx}.ExpFilePathList{traIdx}};
                
        GroupedTrajectoriesCollection(size(GroupedTrajectoriesCollection,1)+1,:) = TempLine;
    end
end

GroupedTrajectoriesCollectionTable = cell2table(GroupedTrajectoriesCollection,'VariableNames',["StridePeriod","Speed",...
                                                                                "GaitGroupName",...
                                                                                "ExpFileName",...
                                                                                "ExpFilePath"]);
writetable(GroupedTrajectoriesCollectionTable,[data_file_path,'/TableForm_TrajectorywithGroupLabels_',datestr(datetime('now'), 30),'.csv']);

disp('-------------------------------------------------------------------')
disp('Data Extraction For Half-Cheetah Finished')
disp('-------------------------------------------------------------------')