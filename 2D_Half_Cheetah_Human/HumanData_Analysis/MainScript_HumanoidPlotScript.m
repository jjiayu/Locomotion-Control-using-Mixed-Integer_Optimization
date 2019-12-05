% Humanoid Plotting Script
%   Need to load the data analysis result first

clear;
clc;

%Load Data first
disp('Load Humanoid Parameter Extraction Result')
[data_collection_filename,data_collection_filepath] = uigetfile();
full_data_collection_filepath = [data_collection_filepath,data_collection_filename];
disp(['Selected File: ',full_data_collection_filepath]);
load(full_data_collection_filepath)


% Do some checking first to decide if we need to plot left and right foot
% quantities separately
LeftRight_DifferentiateFlag = 0; %No difference between Left and Right Leg
for dataIdx = 1:length(DataAnalysisResult)
    if abs(DataAnalysisResult{dataIdx}.LeftStepLength - DataAnalysisResult{dataIdx}.RightStepLength) > 0.5
        
        %Display Probelmatic Samples
        disp('----------------------------------------------------------')
        disp(['Problem Sample Index: ',num2str(dataIdx)]);
        disp(['Difference between Left Step Length and Right Step Length: ', num2str(DataAnalysisResult{dataIdx}.LeftStepLength - DataAnalysisResult{dataIdx}.RightStepLength)]);
        disp('----------------------------------------------------------')

        %Toggle the flag
        LeftRight_DifferentiateFlag = 1; %There is a difference between left and right leg
        
    end
end

% Plot Mappings
if LeftRight_DifferentiateFlag  == 0
    run('Humanoid_Plot_as_a_Whole.m')
elseif LeftRight_DifferentiateFlag == 1
    run('Humanoid_Plot_Left_and_Right.m')
end