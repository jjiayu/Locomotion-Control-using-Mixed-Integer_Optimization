% Extract Data for Humanoid result, aiming for all the gaits, hopefully

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
% Loop inside Data (resultMatrix)
%-------------------------------------------------------------------------
disp('-------------------------------------------------------------------')
disp('Extracting Important Quantities')
disp('-------------------------------------------------------------------')
DataAnalysisResult = {};
for resultIdx = 1:size(resultMatrix,1)*size(resultMatrix,2)
    
    disp(['Stride Period: ', num2str(resultMatrix{resultIdx}.strideperiod_speed_pair(1)),' Speed: ', num2str(resultMatrix{resultIdx}.strideperiod_speed_pair(2))])
    
    %Dealing with the data with Optimal gait
    %   Initialize a structure for data storage
    DataAnalysisResTemp = struct();
    
    if strcmp(resultMatrix{resultIdx}.OptimalGaitName,'N/A') ~= 1
        %------------------------------------------------------------------
        %   Extract Data for the Optimal Solution
        %------------------------------------------------------------------
        %   Standard Variables
        DataAnalysisResTemp.StridePeriod = resultMatrix{resultIdx}.strideperiod_speed_pair(1);
        DataAnalysisResTemp.Speed = resultMatrix{resultIdx}.strideperiod_speed_pair(2);
        %   Experiment Specific Variables
        DataAnalysisResTemp.DiscoveredOptimalGaitName = resultMatrix{resultIdx}.OptimalGaitName;
        DataAnalysisResTemp.DiscoveredOptimalGait = resultMatrix{resultIdx}.OptimalGait;
        DataAnalysisResTemp.ExpFileName = resultMatrix{resultIdx}.OptimalFile;

        %   Load Experiment Log Data
        ExpFilePath = strcat(data_file_path,'4Phases_StridePeriod_',num2str(resultMatrix{resultIdx}.strideperiod_speed_pair(1)),'/',resultMatrix{resultIdx}.OptimalFile); 
        DataAnalysisResTemp.ExpFilePath = ExpFilePath;
        [LeftStepLengthTemp,RightStepLengthTemp,StrangeGaitFlagTemp] = GetStepLength(ExpFilePath);
        DataAnalysisResTemp.LeftStepLength = LeftStepLengthTemp;
        DataAnalysisResTemp.RightStepLength = RightStepLengthTemp;
        DataAnalysisResTemp.StrangeGaitFlag = StrangeGaitFlagTemp;

        %   Save the struct into DataAnalysisResultList
        DataAnalysisResult{end+1} = DataAnalysisResTemp;
        
        %------------------------------------------------------------------
        %   Extract Data for the Soluitions with Similar Cost
        %------------------------------------------------------------------
        
        for similiarResultIdx = 1:length(resultMatrix{resultIdx}.GaitNameswithSimilarCost)
            %   Initialize a structure for data storage
            DataAnalysisResTemp = struct();
            
                    %   Standard Variables
            DataAnalysisResTemp.StridePeriod = resultMatrix{resultIdx}.strideperiod_speed_pair(1);
            DataAnalysisResTemp.Speed = resultMatrix{resultIdx}.strideperiod_speed_pair(2);
            %   Experiment Specific Variables
            DataAnalysisResTemp.DiscoveredOptimalGaitName = resultMatrix{resultIdx}.GaitNameswithSimilarCost{similiarResultIdx};
            %DataAnalysisResTemp.DiscoveredOptimalGait = resultMatrix{resultIdx}.OptimalGait;
            DataAnalysisResTemp.ExpFileName = resultMatrix{resultIdx}.SimilarLocalMinimaFileNames{similiarResultIdx};
            
            ExpFilePath = strcat(data_file_path,'4Phases_StridePeriod_',num2str(resultMatrix{resultIdx}.strideperiod_speed_pair(1)),'/',resultMatrix{resultIdx}.SimilarLocalMinimaFileNames{similiarResultIdx}); 
            %Load gait
            gaitTemp = load(ExpFilePath,'gait');
            DataAnalysisResTemp.DiscoveredOptimalGait = gaitTemp.gait;
            
            DataAnalysisResTemp.ExpFilePath = ExpFilePath;
            [LeftStepLengthTemp,RightStepLengthTemp,StrangeGaitFlagTemp] = GetStepLength(ExpFilePath);
            DataAnalysisResTemp.LeftStepLength = LeftStepLengthTemp;
            DataAnalysisResTemp.RightStepLength = RightStepLengthTemp;
            DataAnalysisResTemp.StrangeGaitFlag = StrangeGaitFlagTemp;
            
            %   Save the struct into DataAnalysisResultList
            DataAnalysisResult{end+1} = DataAnalysisResTemp;
        end    
    end
end

%Save Data
save([data_file_path,'/HumanoidDataExtractionResult-',datestr(datetime('now'), 30),'.mat']);

disp('-------------------------------------------------------------------')
disp('Making Important Quantities')
disp('-------------------------------------------------------------------')

%   Save Data into CSV file
DataCollection = {};
for dataIdx = 1:length(DataAnalysisResult)
    
    disp(['Stride Period: ', num2str(DataAnalysisResult{dataIdx}.StridePeriod),' Speed: ', num2str(DataAnalysisResult{dataIdx}.Speed)])
    
    TempLine = {DataAnalysisResult{dataIdx}.StridePeriod,DataAnalysisResult{dataIdx}.Speed,...
                DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName, DataAnalysisResult{dataIdx}.ExpFileName,...
                DataAnalysisResult{dataIdx}.ExpFilePath,...
                DataAnalysisResult{dataIdx}.LeftStepLength,DataAnalysisResult{dataIdx}.RightStepLength,...
                DataAnalysisResult{dataIdx}.StrangeGaitFlag};
    DataCollection(size(DataCollection,1)+1,:) = TempLine;
    
end

DataCollectionTable = cell2table(DataCollection,'VariableNames',["StridePeriod","Speed",...
                                                                 "DiscoveredOptimalGaitName","ExpFileName",...
                                                                 "ExpFilePath",...
                                                                 "LeftStepLength","RightStepLength",...
                                                                 "StrangeGaitFlag"]);
writetable(DataCollectionTable,[data_file_path,'/HumanoidDataExtractionResult-',datestr(datetime('now'), 30),'.csv']);

disp('-------------------------------------------------------------------')
disp('Data Extraction Finished')
disp('-------------------------------------------------------------------')