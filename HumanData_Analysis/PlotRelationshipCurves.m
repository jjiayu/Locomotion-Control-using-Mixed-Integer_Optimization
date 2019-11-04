% Given Running the Data Processing Scritps, Loaded with Gait Discovery
% Results.

%Plot Step Frequency v.s. Step Length, left and right leg separately
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
% Currently focus on Walking-D Gait Only
GaitofInterest = 'Walking-D';
%-------------------------------------------------------------------------
% Define StridePeriod Range of interest
StridePeriod_Start = 0.4;
StridePeriod_End   = 0.4;
%Search for Stride Period Start and End Index, because of technical
%problem, has to write in this way.....
for StridePeriodLoop_Idx = 1:length(StridePeriod_List)
    if (StridePeriod_Start >= StridePeriod_List(StridePeriodLoop_Idx)-StridePeriod_List(StridePeriodLoop_Idx)*0.1) && ...
       (StridePeriod_Start <= StridePeriod_List(StridePeriodLoop_Idx)+StridePeriod_List(StridePeriodLoop_Idx)*0.1)
        StridePeriod_Start_Idx = StridePeriodLoop_Idx;
    end
    
    if (StridePeriod_End >= StridePeriod_List(StridePeriodLoop_Idx)-StridePeriod_List(StridePeriodLoop_Idx)*0.1) && ...
       (StridePeriod_End <= StridePeriod_List(StridePeriodLoop_Idx)+StridePeriod_List(StridePeriodLoop_Idx)*0.1)
        StridePeriod_End_Idx = StridePeriodLoop_Idx;
    end
end

%-------------------------------------------------------------------------
% Collect all data with the gait of interest
GaitNameMatrix_withSelectedStridePeriod = GaitNameMatrix_withAllLocalMinima(:,StridePeriod_Start_Idx:StridePeriod_End_Idx);
ResultMatrix_withSelectedStridePeriod = resultMatrix(:,StridePeriod_Start_Idx:StridePeriod_End_Idx);

GaitofInterest_Idxs = find(contains(GaitNameMatrix_withAllLocalMinima(:,StridePeriod_Start_Idx:StridePeriod_End_Idx),GaitofInterest));
Result_GaitofInterest_List = ResultMatrix_withSelectedStridePeriod(GaitofInterest_Idxs);

%-------------------------------------------------------------------------
% Analysize Data
%       NOTE: For each result cell, we have multiple solutions, expecially
%       for walking-D, there are many solutions with different length of
%       double support phase
DataAnalysisResult = {};
for iterate_idx = 1:length(Result_GaitofInterest_List)
    
    DataAnalysisResult{iterate_idx}.StridePeriod = Result_GaitofInterest_List{iterate_idx}.strideperiod_speed_pair(1);
    DataAnalysisResult{iterate_idx}.speed = Result_GaitofInterest_List{iterate_idx}.strideperiod_speed_pair(2);
    
    if strcmp(Result_GaitofInterest_List{iterate_idx}.OptimalGaitName,GaitofInterest) == 1 %We have the GaitofInterest as the 
        OptimalTraFileName = Result_GaitofInterest_List{iterate_idx}.OptimalFile;
    elseif strcmp(Result_GaitofInterest_List{iterate_idx}.OptimalGaitName,GaitofInterest) == 0
        for fileloop_idx = 1:length(Result_GaitofInterest_List{iterate_idx}.GaitNameswithSimilarCost)
            if strcmp(Result_GaitofInterest_List{iterate_idx}.GaitNameswithSimilarCost{fileloop_idx},GaitofInterest)
                OptimalTraFileName = Result_GaitofInterest_List{iterate_idx}.SimilarLocalMinimaFileNames{fileloop_idx};
                break
            end
        end
    end
    
    load([data_file_path,'4Phases_StridePeriod_',num2str(Result_GaitofInterest_List{iterate_idx}.strideperiod_speed_pair(1)),'/',OptimalTraFileName]);
    
    %Process trajectories to get step frequency and step length
    %NOTE: Currently for Walking Gait Only
    
    DataAnalysisResult{iterate_idx}.OptimalGait = gait;
    
    DataAnalysisResult{iterate_idx}.Front_Leg_StepPeriod = gait(find(gait(:,1)==0),3);
    DataAnalysisResult{iterate_idx}.Front_Leg_StepFrequency = 1/DataAnalysisResult{iterate_idx}.Front_Leg_StepPeriod;
    DataAnalysisResult{iterate_idx}.Hind_Leg_StepPeriod = gait(find(gait(:,2)==0),3);
    DataAnalysisResult{iterate_idx}.Hind_Leg_StepFrequency = 1/DataAnalysisResult{iterate_idx}.Hind_Leg_StepPeriod;
    
end

